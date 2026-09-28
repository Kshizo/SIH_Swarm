#!/usr/bin/env python3
"""Export coverage and event data from recorded missions, for charting.

Reads the ROS 2 bags a `--bag` run leaves behind and writes two CSVs:

  coverage.csv   one row per drone per sample: cumulative known area of that
                 drone's OWN occupancy grid, and the frontier cell count
  events.csv     one row per takeoff, detection, hazard and landing

Everything comes from the bags. Nothing is interpolated, smoothed or filled -
a field that was not recorded is left empty.

    python3 sim/tools/export_run_data.py sim/logs/2026*/ --out-dir docs/data

Bags are opened by their .mcap file rather than their directory, because a
recorder killed without SIGINT never writes metadata.yaml and rosbag2 refuses
the directory. The file itself is self-describing and complete.
"""

import argparse
import csv
import glob
import math
import os
import re
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ('/map', '/ap/status', '/person_map_pins', '/hazard_markers',
          '/exploration/frontier_cells')


def open_bag(bag_dir):
    """Open the .mcap inside a bag directory. Returns a reader, or None."""
    files = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    if not files:
        return None
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=files[0], storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def read_messages(bag_dir):
    """Yield (topic, message, receive_time_seconds).

    The RECEIVE time, deliberately, not the header stamp. ArduPilot stamps
    /ap/status with unix epoch once it has a satellite fix but with time since
    boot when it does not, while slam_toolbox stamps /map with simulation time
    throughout. Anchoring one against the other produces differences of 1.8
    billion seconds. The recorder's own clock is the one thing every topic
    shares.
    """
    reader = open_bag(bag_dir)
    if reader is None:
        return
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, raw, received_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        message = deserialize_message(raw, get_message(types[topic]))
        yield topic, message, received_ns * 1e-9


def known_area(grid_msg):
    data = np.asarray(grid_msg.data, dtype=np.int8)
    known = int(np.count_nonzero(data != -1))
    return known * grid_msg.info.resolution ** 2


def frontier_count(grid_msg):
    """Distinct frontier regions, not cells.

    /exploration/frontier_cells is a grid of cells, so a raw count would
    describe area rather than how many separate places are left to go. Label
    the column for what it measures.
    """
    try:
        import cv2
    except ImportError:
        return ''
    data = np.asarray(grid_msg.data, dtype=np.int8).reshape(
        grid_msg.info.height, grid_msg.info.width)
    mask = (data > 0).astype(np.uint8)
    if not mask.any():
        return 0
    count, _ = cv2.connectedComponents(mask, connectivity=8)
    return count - 1


def arm_time(bag_dir):
    """Recorder-clock time at which this drone first reports armed."""
    for topic, message, stamp in read_messages(bag_dir):
        if topic == '/ap/status' and message.armed and stamp is not None:
            return stamp
    return None


def mission_window(run_dir):
    """How long this run's mission actually lasted, in seconds.

    A bag can outlive its own run - a recorder that is not killed keeps logging
    whatever appears on the domain next. The recorder log states the mission
    length, so anything past it belongs to a later run and is dropped.
    """
    log = os.path.join(run_dir, 'recorder.log')
    if not os.path.exists(log):
        return None
    # "wrote .../mission.mp4: 17984 frames, 1798.4 s" - take the seconds, not
    # the frame count that precedes it.
    for line in open(log):
        match = re.search(r'wrote .*?([\d.]+)\s*s\s*$', line.strip())
        if match:
            return float(match.group(1))
    return None


def export_run(run_dir, run_label, coverage_rows, event_rows, datum):
    window = mission_window(run_dir)
    if window is None:
        print('  no mission window in recorder.log - skipping run')
        return
    print('  mission window: %.0f s' % window)
    for drone in (1, 2):
        bag_dir = os.path.join(run_dir, 'bag_drone%d' % drone)
        if not os.path.isdir(bag_dir):
            continue

        t0 = arm_time(bag_dir)
        if t0 is None:
            print('  drone %d: never reported armed - skipped' % drone)
            continue

        # The mcap reader returns messages in FILE order, not time order, so
        # nothing here may assume the stream is monotonic: samples are gathered
        # first, sorted, and only then thinned to one a second.
        samples = []
        seen_pins, seen_hazards = set(), set()
        armed_before = False
        last_disarm = {}

        for topic, message, stamp in read_messages(bag_dir):
            if stamp is None:
                continue
            t = stamp - t0
            if t > window:
                continue         # past this run; belongs to a later one

            if topic == '/ap/status':
                # /ap/status flickers; record the first arm and the last
                # disarm rather than every transition.
                if message.armed and not armed_before:
                    armed_before = True
                    if not any(r[0] == run_label and r[1] == drone and r[3] == 'takeoff'
                               for r in event_rows):
                        event_rows.append([run_label, drone, round(t, 2), 'takeoff',
                                           '', '', '', 'first armed sample'])
                elif armed_before and not message.armed:
                    armed_before = False
                    last_disarm[drone] = round(t, 2)

            elif topic == '/map':
                samples.append((t, known_area(message), None))

            elif topic == '/exploration/frontier_cells':
                samples.append((t, None, frontier_count(message)))

            elif topic in ('/person_map_pins', '/hazard_markers'):
                # Both topics republish their whole list continuously, and the
                # hazard positions jitter by centimetres each time. Only a
                # finding far enough from everything already seen is new.
                kind = 'detection' if topic == '/person_map_pins' else 'hazard'
                seen = seen_pins if kind == 'detection' else seen_hazards
                radius = 1.5 if kind == 'detection' else 2.5
                for marker in message.markers:
                    if marker.action != 0 or marker.ns.endswith('_label'):
                        continue
                    x, y = marker.pose.position.x, marker.pose.position.y
                    if any(math.hypot(x - px, y - py) <= radius for px, py in seen):
                        continue
                    seen.add((x, y))
                    lat, lon = to_latlon(x, y, drone, datum)
                    event_rows.append([run_label, drone, round(t, 2), kind,
                                       '%s_%d' % (marker.ns or kind, len(seen)),
                                       round(lat, 7), round(lon, 7),
                                       'first sighting; position in this drone\'s map frame'])

        # Sort, then thin to roughly one sample a second, carrying the most
        # recent frontier count alongside each area sample.
        samples.sort(key=lambda s: s[0])

        # Within one mission the known area only grows - SLAM adds cells and
        # never wholesale removes them. A collapse means a fresh SLAM session,
        # i.e. the next run bleeding into a bag whose recorder outlived it.
        # Cut there, whatever the nominal window said.
        peak, cutoff = 0.0, math.inf
        for t_s, area, _ in samples:
            if area is None:
                continue
            if area < 0.5 * peak and peak > 10.0:
                cutoff = t_s
                break
            peak = max(peak, area)

        emitted = -math.inf
        frontier = ''
        for t_s, area, regions in samples:
            if t_s >= cutoff:
                break
            if regions is not None:
                frontier = regions
            if area is None or t_s - emitted < 1.0:
                continue
            emitted = t_s
            coverage_rows.append([run_label, drone, round(t_s, 2),
                                  round(area, 2), frontier])

        if drone in last_disarm:
            event_rows.append([run_label, drone, last_disarm[drone], 'land',
                               '', '', '', 'final disarm'])

        mine = [r for r in coverage_rows if r[0] == run_label and r[1] == drone]
        print('  drone %d: %d samples, %.0f-%.0f s, final %.1f m2'
              % (drone, len(mine), mine[0][2] if mine else 0,
                 mine[-1][2] if mine else 0, mine[-1][3] if mine else 0))


LAUNCH = {1: (-9.6, -9.6, 0.0), 2: (9.6, 9.6, math.pi)}
EARTH_RADIUS_M = 6378137.0


def to_latlon(x, y, drone, datum):
    ox, oy, yaw = LAUNCH[drone]
    mx = ox + math.cos(yaw) * x - math.sin(yaw) * y
    my = oy + math.sin(yaw) * x + math.cos(yaw) * y
    lat = datum[0] + math.degrees(my / EARTH_RADIUS_M)
    lon = datum[1] + math.degrees(mx / (EARTH_RADIUS_M * math.cos(math.radians(datum[0]))))
    return lat, lon


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', help='run directories')
    parser.add_argument('--out-dir', default='docs/data')
    parser.add_argument('--datum', default='-35.363262,149.165237')
    args = parser.parse_args()

    datum = tuple(float(v) for v in args.datum.split(','))
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    coverage_rows, event_rows = [], []
    for run_dir in args.runs:
        run_dir = run_dir.rstrip('/')
        label = ('gps_enabled' if os.path.exists(os.path.join(run_dir, 'mission-gps.mp4'))
                 else 'gps_denied')
        label = '%s/%s' % (label, os.path.basename(run_dir))
        print('%s  (%s)' % (os.path.basename(run_dir), label.split('/')[0]))
        export_run(run_dir, label, coverage_rows, event_rows, datum)

    with open(out / 'coverage.csv', 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['run', 'drone', 't_s', 'explored_m2', 'frontier_regions_n'])
        writer.writerows(coverage_rows)

    with open(out / 'events.csv', 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['run', 'drone', 't_s', 'type', 'label', 'lat', 'lon', 'note'])
        writer.writerows(event_rows)

    print('\nwrote %s (%d rows) and %s (%d rows)'
          % (out / 'coverage.csv', len(coverage_rows),
             out / 'events.csv', len(event_rows)))


if __name__ == '__main__':
    main()
