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

It prints the running team total and, on exit, writes a JSON summary.
"""

import json
import math
import os
import signal
import threading
import time

import rclpy
import rclpy.executors
from rclpy.context import Context
from visualization_msgs.msg import Marker, MarkerArray


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

    def __init__(self, drones, launch_poses, merge_radius, expected, out_path):
        self.merge_radius = merge_radius
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
                 'seen_by': sorted(c['seen_by']), 'pins': c['n']}
                for c in clusters
            ],
            'per_drone_raw_pins': {str(l.drone_id): len(l.snapshot()) for l in self.links},
            'hazards': [
                {'kind': h['kind'], 'x': round(h['x'], 2), 'y': round(h['y'], 2),
                 'seen_by': sorted(h['seen_by'])}
                for h in hazards
            ],
        }
        with open(self.out_path, 'w') as handle:
            json.dump(summary, handle, indent=2)
        print('FINAL: %d of %d survivors, %d hazards -> %s'
              % (len(clusters), self.expected, len(hazards), self.out_path), flush=True)

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
    parser.add_argument('--out', default='survivors.json')
    args = parser.parse_args()

    drones = [int(d) for d in args.drones.split(',') if d.strip()]
    poses = []
    for chunk in args.launch_poses.split():
        x, y, yaw = (float(v) for v in chunk.split(','))
        poses.append((x, y, yaw))
    if len(poses) != len(drones):
        raise SystemExit('need one launch pose per drone')

    station = GroundStation(drones, poses, args.merge_radius, args.expected,
                            os.path.abspath(args.out))
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
