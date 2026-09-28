#!/usr/bin/env python3
"""Merge what the drones found into one picture for the operator.

Each drone flies on its own ROS domain and never hears the others, which is the
right call inside a building where the radio link dies a few walls in. The
reconciliation therefore happens here, outside, on the way back - the drones
carry their results home rather than streaming them.

This node joins every drone's domain (one rclpy context each), transforms their
survivor pins out of each drone's own drifting map frame into a shared mission
frame using the launch pose the operator already knows, and de-duplicates what
is left. Two drones that both saw the same person report one survivor, not two.

    ros2 run drone_control ground_station --ros-args \\
        -p drones:="[1, 2]" \\
        -p launch_poses:="[-9.6, -9.6, 0.0,  9.6, 9.6, 3.14159]"

It prints the running team total and, on exit, writes a JSON summary and a
situation report with every survivor geo-tagged and ranked for dispatch.
"""

import json
import math
import os
import signal
import threading
import time
from pathlib import Path

import rclpy
import rclpy.executors
from rclpy.context import Context
from visualization_msgs.msg import Marker, MarkerArray


EARTH_RADIUS_M = 6378137.0


def to_latlon(x, y, datum):
    """Mission frame (metres, x East, y North) -> latitude and longitude.

    A flat-earth approximation about the datum. Over a site a few hundred
    metres across the error is centimetres, and the datum is the same one the
    autopilot is given as its EKF origin, so the coordinates a rescue team
    receives are in the same frame as the aircraft's own.
    """
    lat0, lon0 = datum[0], datum[1]
    lat = lat0 + math.degrees(y / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


class DroneLink:
    """One drone's domain, and the pins it has reported so far."""

    def __init__(self, drone_id, launch_pose, on_change):
        self.drone_id = drone_id
        self.x, self.y, self.yaw = launch_pose
        self.pins = []
        self.hazards = []
        self.on_change = on_change
        self.lock = threading.Lock()

        self.context = Context()
        rclpy.init(context=self.context, domain_id=drone_id)
        self.node = rclpy.create_node('ground_station_d%d' % drone_id, context=self.context)
        self.node.create_subscription(MarkerArray, '/person_map_pins', self.on_pins, 10)
        self.node.create_subscription(MarkerArray, '/hazard_markers', self.on_hazards, 10)

        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def to_mission_frame(self, x, y):
        """Drone's map frame -> the shared mission frame.

        Each drone's map is anchored to where that drone took off, with its
        x axis along its launch heading. The launch pose is known to the
        operator, so the two maps can be brought into one frame without the
        drones ever having agreed on anything.
        """
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + cos_yaw * x - sin_yaw * y,
                self.y + sin_yaw * x + cos_yaw * y)

    def on_pins(self, msg):
        pins = []
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            pins.append(self.to_mission_frame(marker.pose.position.x,
                                              marker.pose.position.y))
        if not pins:
            return
        with self.lock:
            changed = len(pins) != len(self.pins)
            self.pins = pins
        if changed:
            self.on_change()

    def on_hazards(self, msg):
        hazards = []
        for marker in msg.markers:
            # The mapper publishes a body and a label per hazard; take the
            # bodies, which carry the namespace that names the hazard type.
            if marker.action != Marker.ADD or marker.ns.endswith('_label'):
                continue
            x, y = self.to_mission_frame(marker.pose.position.x, marker.pose.position.y)
            hazards.append((x, y, marker.ns))
        with self.lock:
            changed = len(hazards) != len(self.hazards)
            self.hazards = hazards
        if changed:
            self.on_change()

    def snapshot(self):
        with self.lock:
            return list(self.pins)

    def hazard_snapshot(self):
        with self.lock:
            return list(self.hazards)

    def close(self):
        try:
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.try_shutdown(context=self.context)
        except Exception:
            pass


class GroundStation:

    def __init__(self, drones, launch_poses, merge_radius, expected, out_path, datum):
        self.merge_radius = merge_radius
        self.datum = datum
        self.expected = expected
        self.out_path = out_path
        self.reported = 0
        self.reported_hazards = 0
        self.started = time.time()
        self.links = [
            DroneLink(drone_id, launch_poses[index], self.recount)
            for index, drone_id in enumerate(drones)
        ]

    def merged(self):
        """Cluster every drone's pins into unique survivors.

        Greedy single-pass clustering: a pin joins the first cluster within
        merge_radius, otherwise it starts one. The pins come from two maps that
        have each drifted on their own, so this is a best-effort reconciliation,
        not a precise one - merge_radius has to be wider than the drift.
        """
        clusters = []
        for link in self.links:
            for x, y in link.snapshot():
                for cluster in clusters:
                    if math.hypot(x - cluster['x'], y - cluster['y']) <= self.merge_radius:
                        cluster['seen_by'].add(link.drone_id)
                        cluster['n'] += 1
                        cluster['x'] += (x - cluster['x']) / cluster['n']
                        cluster['y'] += (y - cluster['y']) / cluster['n']
                        break
                else:
                    clusters.append({'x': x, 'y': y, 'n': 1, 'seen_by': {link.drone_id}})
        return clusters

    def merged_hazards(self):
        """Same clustering as survivors, keyed by hazard type."""
        clusters = []
        for link in self.links:
            for x, y, kind in link.hazard_snapshot():
                for cluster in clusters:
                    if (cluster['kind'] == kind
                            and math.hypot(x - cluster['x'], y - cluster['y'])
                            <= self.merge_radius):
                        cluster['seen_by'].add(link.drone_id)
                        break
                else:
                    clusters.append({'x': x, 'y': y, 'kind': kind,
                                     'seen_by': {link.drone_id}})
        return clusters

    def recount(self):
        clusters = self.merged()
        hazards = self.merged_hazards()
        if len(clusters) == self.reported and len(hazards) == self.reported_hazards:
            return
        self.reported = len(clusters)
        self.reported_hazards = len(hazards)
        raw = sum(len(link.snapshot()) for link in self.links)
        print('[t+%4d s] TEAM: %d of %d survivors, %d hazards  '
              '(%d raw pins from %d drones)'
              % (time.time() - self.started, len(clusters), self.expected,
                 len(hazards), raw, len(self.links)), flush=True)
        for index, cluster in enumerate(clusters, start=1):
            print('           %d. x=%6.2f y=%6.2f  seen by %s'
                  % (index, cluster['x'], cluster['y'],
                     ', '.join('drone %d' % d for d in sorted(cluster['seen_by']))),
                  flush=True)
        for index, hazard in enumerate(hazards, start=1):
            print('           H%d. %-13s x=%6.2f y=%6.2f  seen by %s'
                  % (index, hazard['kind'], hazard['x'], hazard['y'],
                     ', '.join('drone %d' % d for d in sorted(hazard['seen_by']))),
                  flush=True)

    def triage(self, clusters, hazards, obstruction_radius=4.0):
        """Rank survivors for dispatch, and say why.

        Two things decide the order, because they are the two a dispatcher can
        act on: whether the way in is obstructed, and how confident we are the
        person is really there.
        """
        ranked = []
        for cluster in clusters:
            near = [(math.hypot(cluster['x'] - h['x'], cluster['y'] - h['y']), h)
                    for h in hazards]
            near.sort(key=lambda pair: pair[0])
            blocking = [pair for pair in near if pair[0] <= obstruction_radius]

            if blocking:
                priority, reason = 1, 'access obstructed'
            elif len(cluster['seen_by']) < 2:
                priority, reason = 2, 'single-drone detection, needs verification'
            else:
                priority, reason = 3, 'confirmed by two drones, access clear'

            ranked.append({
                'cluster': cluster,
                'priority': priority,
                'reason': reason,
                'nearest': near[0] if near else None,
                'blocking': blocking,
            })
        ranked.sort(key=lambda r: (r['priority'], -len(r['cluster']['seen_by'])))
        return ranked

    def write_report(self, clusters, hazards, path):
        ranked = self.triage(clusters, hazards)
        kinds = {}
        for hazard in hazards:
            kinds[hazard['kind']] = kinds.get(hazard['kind'], 0) + 1

        lines = []
        add = lines.append
        add('SITUATION REPORT')
        add('=' * 64)
        add('datum        %.7f, %.7f  (WGS84)' % (self.datum[0], self.datum[1]))
        add('elapsed      %d s' % (time.time() - self.started))
        add('drones       %s' % ', '.join('drone %d' % l.drone_id for l in self.links))
        add('survivors    %d of %d located' % (len(clusters), self.expected))
        add('hazards      %d  (%s)' % (len(hazards), ', '.join(
            '%d %s' % (n, k) for k, n in sorted(kinds.items())) or 'none'))
        add('')
        add('Ranked by access first, then by confidence: a survivor behind a hazard')
        add('needs equipment and time, and a single-drone detection needs a second')
        add('look before a team is committed to it.')
        add('')

        for index, entry in enumerate(ranked, start=1):
            cluster = entry['cluster']
            lat, lon = to_latlon(cluster['x'], cluster['y'], self.datum)
            add('-' * 64)
            add('P%d  SURVIVOR %d   %s' % (entry['priority'], index, entry['reason'].upper()))
            add('    position   %.7f, %.7f' % (lat, lon))
            add('    local      x %+.2f m, y %+.2f m from the datum' % (cluster['x'], cluster['y']))
            add('    detections %d, by %s' % (
                cluster['n'], ' and '.join('drone %d' % d for d in sorted(cluster['seen_by']))))
            if entry['blocking']:
                for distance, hazard in entry['blocking']:
                    add('    ACCESS     %s %.1f m away' % (hazard['kind'], distance))
            elif entry['nearest']:
                distance, hazard = entry['nearest']
                add('    access     clear; nearest %s is %.1f m away' % (hazard['kind'], distance))
            else:
                add('    access     clear')

        if hazards:
            add('-' * 64)
            add('HAZARDS')
            for index, hazard in enumerate(sorted(hazards, key=lambda h: h['kind']), start=1):
                lat, lon = to_latlon(hazard['x'], hazard['y'], self.datum)
                add('  H%-2d %-14s %.7f, %.7f' % (index, hazard['kind'], lat, lon))

        add('-' * 64)
        add('Coordinates share the autopilot\'s EKF origin, so they refer to the same')
        add('frame the aircraft flew in. Generated without any network connection.')

        Path(path).write_text('\n'.join(lines) + '\n')
        return ranked

    def write_summary(self):
        clusters = self.merged()
        hazards = self.merged_hazards()
        summary = {
            'expected': self.expected,
            'found': len(clusters),
            'elapsed_s': round(time.time() - self.started, 1),
            'merge_radius_m': self.merge_radius,
            'survivors': [
                {'x': round(c['x'], 2), 'y': round(c['y'], 2),
                 'lat': round(to_latlon(c['x'], c['y'], self.datum)[0], 7),
                 'lon': round(to_latlon(c['x'], c['y'], self.datum)[1], 7),
                 'seen_by': sorted(c['seen_by']), 'pins': c['n']}
                for c in clusters
            ],
            'datum': {'lat': self.datum[0], 'lon': self.datum[1], 'alt': self.datum[2]},
            'per_drone_raw_pins': {str(l.drone_id): len(l.snapshot()) for l in self.links},
            'hazards': [
                {'kind': h['kind'], 'x': round(h['x'], 2), 'y': round(h['y'], 2),
                 'lat': round(to_latlon(h['x'], h['y'], self.datum)[0], 7),
                 'lon': round(to_latlon(h['x'], h['y'], self.datum)[1], 7),
                 'seen_by': sorted(h['seen_by'])}
                for h in hazards
            ],
        }
        with open(self.out_path, 'w') as handle:
            json.dump(summary, handle, indent=2)
        report_path = os.path.splitext(self.out_path)[0].replace('survivors', 'situation')
        report_path = (report_path if report_path != os.path.splitext(self.out_path)[0]
                       else os.path.splitext(self.out_path)[0] + '-report') + '.txt'
        self.write_report(clusters, hazards, report_path)
        print('FINAL: %d of %d survivors, %d hazards -> %s and %s'
              % (len(clusters), self.expected, len(hazards),
                 self.out_path, report_path), flush=True)

    def close(self):
        for link in self.links:
            link.close()


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drones', default='1,2',
                        help='comma-separated ROS_DOMAIN_IDs, one per drone')
    parser.add_argument('--launch-poses', default='-9.6,-9.6,0 9.6,9.6,3.14159',
                        help='x,y,yaw per drone in the mission frame, space separated')
    parser.add_argument('--merge-radius', type=float, default=3.0,
                        help='pins closer than this are the same person (metres)')
    parser.add_argument('--expected', type=int, default=4)
    parser.add_argument('--datum', default='-35.363262,149.165237,584',
                        help='lat,lon,alt of the mission frame origin - the same '
                             'datum the autopilot is given as its EKF origin')
    parser.add_argument('--out', default='survivors.json')
    args = parser.parse_args()

    drones = [int(d) for d in args.drones.split(',') if d.strip()]
    poses = []
    for chunk in args.launch_poses.split():
        x, y, yaw = (float(v) for v in chunk.split(','))
        poses.append((x, y, yaw))
    if len(poses) != len(drones):
        raise SystemExit('need one launch pose per drone')

    datum = tuple(float(v) for v in args.datum.split(','))
    station = GroundStation(drones, poses, args.merge_radius, args.expected,
                            os.path.abspath(args.out), datum)
    print('ground station up: domains %s, merging within %.1f m'
          % (drones, args.merge_radius), flush=True)
    # The run script stops components with SIGTERM, which skipped the summary
    # write; turn it into the same exception Ctrl-C raises.
    def on_term(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        station.write_summary()
        station.close()


if __name__ == '__main__':
    main()
