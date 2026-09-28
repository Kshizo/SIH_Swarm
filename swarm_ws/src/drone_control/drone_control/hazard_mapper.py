#!/usr/bin/env python3
"""Turn the occupancy grid into hazards a responder can act on.

The map the drone is already building says more than "where the walls are". Two
things fall out of it that matter to someone about to walk in:

  CONSTRICTION  a passage narrowed by debris - still flyable, but not a route to
                send a person down. Found by measuring the free width along the
                middle of every corridor and flagging anywhere it drops below a
                safe clearance.

  UNREACHABLE   somewhere the drone tried to reach and could not path to. The
                explorer already gives up on those; this publishes them instead
                of discarding them.

Neither needs a new sensor or a new model - they come from the map and from the
planner's own failures. Thermal and gas hazards need hardware the airframe does
not carry yet.

Publishes visualization_msgs/MarkerArray on /hazard_markers, in the map frame.
"""

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

CONSTRICTION = 'constriction'
UNREACHABLE = 'unreachable'

COLOURS = {
    CONSTRICTION: ColorRGBA(r=0.93, g=0.62, b=0.24, a=0.9),
    UNREACHABLE: ColorRGBA(r=0.85, g=0.27, b=0.20, a=0.9),
}


class HazardMapper(Node):

    def __init__(self):
        super().__init__('hazard_mapper')

        defaults = {
            # Half-width below which a passage is called a constriction. The
            # corridors in this building are 2.4 m (1.2 m clearance); debris
            # leaves about 1.1 m (0.55 m clearance).
            'min_clearance_m': 0.75,
            # Ignore slivers: a constriction has to persist along the corridor.
            'min_constriction_cells': 3,
            # Two hazards closer than this are one hazard.
            'merge_radius_m': 1.2,
            'publish_period_s': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {n: self.get_parameter(n).value for n in defaults}

        self.map_msg = None
        self.hazards = []
        self.unreachable = []

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'hazard_markers', latched)

        self.create_subscription(OccupancyGrid, 'map', self.on_map, 10)
        # The explorer blacklists frontiers it cannot path to; it republishes
        # them here so they are reported rather than silently dropped.
        self.create_subscription(MarkerArray, 'exploration/unreachable',
                                 self.on_unreachable, 10)

        self.create_timer(self.settings['publish_period_s'], self.publish_hazards)
        self.get_logger().info('Hazard mapper running: constrictions below %.2f m clearance'
                               % self.settings['min_clearance_m'])

    # ----------------------------------------------------------------- input
    def on_map(self, msg):
        self.map_msg = msg

    def on_unreachable(self, msg):
        self.unreachable = [(m.pose.position.x, m.pose.position.y)
                            for m in msg.markers if m.action != Marker.DELETE]

    # ----------------------------------------------------------- the analysis
    def find_constrictions(self):
        """Measure free width along the middle of each corridor.

        A distance transform of the free space gives every free cell's distance
        to the nearest wall. Along the centre line of a corridor that distance
        is a local maximum and equals the corridor's half-width, so picking the
        ridge and thresholding it measures passages rather than just finding
        cells that happen to sit near a wall.
        """
        msg = self.map_msg
        if msg is None:
            return []

        info = msg.info
        grid = np.asarray(msg.data, np.int8).reshape(info.height, info.width)
        free = (grid == 0).astype(np.uint8)
        if not free.any():
            return []

        clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * info.resolution

        # The ridge: cells at least as clear as everything in a 5x5 around them.
        dilated = cv2.dilate(clearance, np.ones((5, 5), np.uint8))
        ridge = (clearance >= dilated - 1e-6) & (clearance > 0.0)

        tight = ridge & (clearance < self.settings['min_clearance_m'])
        # Drop the ridge that hugs unknown space - an unexplored edge is not a
        # constriction, it is just the end of the map so far.
        known = cv2.erode((grid >= 0).astype(np.uint8), np.ones((5, 5), np.uint8))
        tight = tight & (known > 0)
        if not tight.any():
            return []

        count, labels, statistics, centroids = cv2.connectedComponentsWithStats(
            tight.astype(np.uint8), connectivity=8)

        found = []
        for label_id in range(1, count):
            if statistics[label_id, cv2.CC_STAT_AREA] < self.settings['min_constriction_cells']:
                continue
            column, row = centroids[label_id]
            x = info.origin.position.x + (column + 0.5) * info.resolution
            y = info.origin.position.y + (row + 0.5) * info.resolution
            width = 2.0 * float(clearance[labels == label_id].max())
            found.append((x, y, width))
        return found

    @staticmethod
    def merge(points, radius):
        """Collapse hazards that are really one hazard seen repeatedly."""
        clusters = []
        for point in points:
            for cluster in clusters:
                if math.hypot(point[0] - cluster[0], point[1] - cluster[1]) <= radius:
                    cluster[2] = min(cluster[2], point[2])
                    break
            else:
                clusters.append(list(point))
        return clusters

    # ---------------------------------------------------------------- output
    def publish_hazards(self):
        constrictions = self.merge(self.find_constrictions(),
                                   self.settings['merge_radius_m'])
        frame = self.map_msg.header.frame_id if self.map_msg else 'map'

        markers = MarkerArray()
        marker_id = 0

        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for x, y, width in constrictions:
            markers.markers.extend(
                self.hazard_markers(marker_id, frame, x, y, CONSTRICTION,
                                    'passage %.1f m' % width))
            marker_id += 2

        for x, y in self.unreachable:
            markers.markers.extend(
                self.hazard_markers(marker_id, frame, x, y, UNREACHABLE,
                                    'no route'))
            marker_id += 2

        self.marker_pub.publish(markers)

        if len(constrictions) != len(self.hazards):
            self.hazards = constrictions
            for x, y, width in constrictions:
                self.get_logger().warning(
                    'HAZARD constriction at X: %.2f, Y: %.2f (%.1f m gap)'
                    % (x, y, width))

    def hazard_markers(self, marker_id, frame, x, y, kind, label):
        stamp = self.get_clock().now().to_msg()

        body = Marker()
        body.header.frame_id = frame
        body.header.stamp = stamp
        body.ns = kind
        body.id = marker_id
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position = Point(x=float(x), y=float(y), z=0.4)
        body.pose.orientation.w = 1.0
        body.scale.x = body.scale.y = 0.8
        body.scale.z = 0.8
        body.color = COLOURS[kind]

        text = Marker()
        text.header.frame_id = frame
        text.header.stamp = stamp
        text.ns = kind + '_label'
        text.id = marker_id + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = Point(x=float(x), y=float(y), z=1.2)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.45
        text.color = COLOURS[kind]
        text.text = '%s: %s' % (kind, label)
        return [body, text]


def main():
    rclpy.init()
    node = HazardMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
