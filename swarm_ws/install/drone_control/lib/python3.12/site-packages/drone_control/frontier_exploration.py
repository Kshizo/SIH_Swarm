#!/usr/bin/env python3
"""Frontier navigation with a separate forward-view coverage layer."""

import copy
import heapq
import math
from collections import deque

import cv2
import numpy as np
import rclpy
import tf2_ros
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def planar_transform(transform):
    quaternion = transform.rotation
    roll = math.atan2(
        2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
        1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2))
    pitch = math.asin(max(-1.0, min(
        1.0, 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x))))
    if max(abs(roll), abs(pitch)) > math.radians(10.0):
        raise ValueError('Scan exceeds planar coverage tilt limit')
    return (transform.translation.x, transform.translation.y,
            quaternion_yaw(quaternion))


def interpolate_planar(first, last, fraction):
    return (first[0] + fraction * (last[0] - first[0]),
            first[1] + fraction * (last[1] - first[1]),
            first[2] + fraction * wrap_angle(last[2] - first[2]))


class ViewCoverage:
    """Binary view history, independent of frontier and path costs."""

    def __init__(self, occupied_threshold=50):
        self.occupied_threshold = occupied_threshold
        self.info = None
        self.frame = None
        self.data = None
        self.seen = None
        self.geometry = None
        self.map_stamp = None

    @staticmethod
    def geometry_key(info):
        return (
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            quaternion_yaw(info.origin.orientation),
        )

    @staticmethod
    def world_coordinates(info, rows, columns):
        origin_yaw = quaternion_yaw(info.origin.orientation)
        local_x = (columns + 0.5) * info.resolution
        local_y = (rows + 0.5) * info.resolution
        world_x = (info.origin.position.x
                   + math.cos(origin_yaw) * local_x
                   - math.sin(origin_yaw) * local_y)
        world_y = (info.origin.position.y
                   + math.sin(origin_yaw) * local_x
                   + math.cos(origin_yaw) * local_y)
        return world_x, world_y

    @staticmethod
    def grid_coordinates(info, world_x, world_y):
        origin_yaw = quaternion_yaw(info.origin.orientation)
        delta_x = np.asarray(world_x) - info.origin.position.x
        delta_y = np.asarray(world_y) - info.origin.position.y
        columns = np.floor(
            (math.cos(origin_yaw) * delta_x
             + math.sin(origin_yaw) * delta_y) / info.resolution
        ).astype(int)
        rows = np.floor(
            (-math.sin(origin_yaw) * delta_x
             + math.cos(origin_yaw) * delta_y) / info.resolution
        ).astype(int)
        return rows, columns

    def set_map(self, message):
        info = message.info
        if (info.width <= 0 or info.height <= 0
                or not math.isfinite(info.resolution)
                or info.resolution <= 0.0
                or len(message.data) != info.width * info.height):
            raise ValueError("Invalid occupancy grid dimensions/resolution")
        if not message.header.frame_id:
            raise ValueError("Occupancy grid has no frame")
        data = np.asarray(message.data, dtype=np.int8).reshape(
            info.height, info.width)
        geometry = self.geometry_key(info)
        timestamp = stamp_seconds(message.header.stamp)
        reset = (
            self.info is None or self.frame != message.header.frame_id
            or self.info.resolution != info.resolution
            or (self.map_stamp is not None and timestamp < self.map_stamp)
        )
        if not reset:
            origin_yaw = quaternion_yaw(info.origin.orientation)
            delta_x = self.info.origin.position.x - info.origin.position.x
            delta_y = self.info.origin.position.y - info.origin.position.y
            shift_column = (math.cos(origin_yaw) * delta_x
                            + math.sin(origin_yaw) * delta_y) / info.resolution
            shift_row = (-math.sin(origin_yaw) * delta_x
                         + math.cos(origin_yaw) * delta_y) / info.resolution
            reset = (
                abs(wrap_angle(origin_yaw - quaternion_yaw(
                    self.info.origin.orientation))) > 1e-9
                or abs(shift_column - round(shift_column)) > 1e-6
                or abs(shift_row - round(shift_row)) > 1e-6)
        if reset:
            self.seen = np.zeros(data.shape, dtype=np.uint8)
        elif geometry != self.geometry:
            old_rows, old_columns = np.nonzero(self.seen)
            world_x, world_y = self.world_coordinates(
                self.info, old_rows, old_columns)
            new_rows, new_columns = self.grid_coordinates(
                info, world_x, world_y)
            inside = ((new_rows >= 0) & (new_rows < info.height)
                      & (new_columns >= 0) & (new_columns < info.width))
            old_rows, old_columns = old_rows[inside], old_columns[inside]
            new_rows, new_columns = new_rows[inside], new_columns[inside]
            unchanged = (
                (self.data[old_rows, old_columns] >= self.occupied_threshold)
                == (data[new_rows, new_columns] >= self.occupied_threshold))
            self.seen = np.zeros(data.shape, dtype=np.uint8)
            self.seen[new_rows[unchanged], new_columns[unchanged]] = 1
        else:
            changed_wall = (
                (self.data >= self.occupied_threshold)
                != (data >= self.occupied_threshold)
            )
            self.seen[changed_wall] = 0
        self.info = copy.deepcopy(info)
        self.geometry = geometry
        self.frame = message.header.frame_id
        self.data = data
        self.map_stamp = timestamp

    def ray_cells(self, world_x, world_y, angle, distance,
                  stop_at_unknown):
        """Grid DDA; include the first blocker, not cells behind it."""
        rows, columns = self.grid_coordinates(
            self.info, world_x, world_y)
        row, column = int(rows), int(columns)
        if not (0 <= row < self.info.height
                and 0 <= column < self.info.width):
            return []
        origin_yaw = quaternion_yaw(self.info.origin.orientation)
        local_angle = angle - origin_yaw
        direction_x = math.cos(local_angle)
        direction_y = math.sin(local_angle)
        offset_x = world_x - self.info.origin.position.x
        offset_y = world_y - self.info.origin.position.y
        local_x = (math.cos(origin_yaw) * offset_x
                   + math.sin(origin_yaw) * offset_y)
        local_y = (-math.sin(origin_yaw) * offset_x
                   + math.cos(origin_yaw) * offset_y)
        resolution = self.info.resolution
        step_column = 1 if direction_x >= 0.0 else -1
        step_row = 1 if direction_y >= 0.0 else -1
        boundary_x = (column + (step_column > 0)) * resolution
        boundary_y = (row + (step_row > 0)) * resolution
        next_x = ((boundary_x - local_x) / direction_x
                  if abs(direction_x) > 1e-12 else math.inf)
        next_y = ((boundary_y - local_y) / direction_y
                  if abs(direction_y) > 1e-12 else math.inf)
        stride_x = (resolution / abs(direction_x)
                    if abs(direction_x) > 1e-12 else math.inf)
        stride_y = (resolution / abs(direction_y)
                    if abs(direction_y) > 1e-12 else math.inf)
        cells = []
        while (0 <= row < self.info.height
               and 0 <= column < self.info.width):
            cells.append((row, column))
            value = self.data[row, column]
            if (value >= self.occupied_threshold
                    or (stop_at_unknown and value < 0)):
                break
            travel = min(next_x, next_y)
            if travel > distance:
                break
            if abs(next_x - next_y) < 1e-10:
                side_cells = ((row, column + step_column),
                              (row + step_row, column))
                blocked = False
                for side_row, side_column in side_cells:
                    if not (0 <= side_row < self.info.height
                            and 0 <= side_column < self.info.width):
                        blocked = True
                        continue
                    value = self.data[side_row, side_column]
                    if (value >= self.occupied_threshold
                            or (stop_at_unknown and value < 0)):
                        cells.append((side_row, side_column))
                        blocked = True
                if blocked:
                    break
                column += step_column
                row += step_row
                next_x += stride_x
                next_y += stride_y
            elif next_x < next_y:
                column += step_column
                next_x += stride_x
            else:
                row += step_row
                next_y += stride_y
        return cells

    def observe_scan(self, scan, laser_first, laser_last, body_first, body_last,
                     fov, yaw_offset, view_range, inf_is_clear):
        """Credit measured beam cells inside the forward viewing FOV."""
        if self.info is None:
            return False
        observed = set()
        for index, measured_range in enumerate(scan.ranges):
            if math.isinf(measured_range) and measured_range > 0.0:
                if not inf_is_clear:
                    continue
                measured_range = scan.range_max
            if (not math.isfinite(measured_range)
                    or not scan.range_min <= measured_range <= scan.range_max):
                continue
            fraction = index / max(1, len(scan.ranges) - 1)
            laser_x, laser_y, laser_yaw = interpolate_planar(
                laser_first, laser_last, fraction)
            view_yaw = interpolate_planar(body_first, body_last, fraction)[2] + yaw_offset
            angle = laser_yaw + scan.angle_min + index * scan.angle_increment
            if abs(wrap_angle(angle - view_yaw)) > fov * 0.5:
                continue
            cells = self.ray_cells(
                laser_x, laser_y,
                angle, min(measured_range, view_range), False)
            for row, column in cells:
                cell_x, cell_y = self.world_coordinates(
                    self.info, row, column)
                if math.hypot(
                        cell_x - laser_x, cell_y - laser_y
                ) + self.info.resolution * math.sqrt(2.0) * 0.5 >= scan.range_min:
                    observed.add((row, column))
        if observed:
            rows, columns = np.asarray(list(observed)).T
            self.seen[rows, columns] = 1
        return bool(observed)

    def best_direction(self, world_x, world_y, body_yaw, fov,
                       yaw_offset, view_range, min_area, excluded):
        """Predict views through mapped free space, stopping at unknown."""
        if self.info is None or fov >= 2.0 * math.pi - 1e-6:
            return None
        visible = set()
        ray_count = max(72, math.ceil(
            4.0 * math.pi * view_range / self.info.resolution))
        for angle in np.linspace(-math.pi, math.pi, ray_count, endpoint=False):
            visible.update(self.ray_cells(
                world_x, world_y, angle, view_range, True))
        if not visible:
            return None
        rows, columns = np.asarray(list(visible)).T
        unseen = self.seen[rows, columns] == 0
        rows, columns = rows[unseen], columns[unseen]
        cell_x, cell_y = self.world_coordinates(self.info, rows, columns)
        distances = np.hypot(cell_x - world_x, cell_y - world_y)
        nearby = (distances >= 0.4) & (distances <= view_range)
        bearings = np.arctan2(cell_y[nearby] - world_y,
                             cell_x[nearby] - world_x)
        best_yaw = None
        best_score = -math.inf
        for desired_yaw in np.linspace(-math.pi, math.pi, 36, endpoint=False):
            turn = abs(wrap_angle(desired_yaw - body_yaw))
            if turn < math.radians(20.0) or excluded(desired_yaw):
                continue
            errors = np.arctan2(
                np.sin(bearings - desired_yaw - yaw_offset),
                np.cos(bearings - desired_yaw - yaw_offset))
            area = np.count_nonzero(np.abs(errors) <= fov * 0.5)
            area *= self.info.resolution ** 2
            if area < min_area:
                continue
            score = area / (1.0 + 0.3 * turn)
            if score > best_score:
                best_score = score
                best_yaw = float(desired_yaw)
        return best_yaw


class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_exploration')
        defaults = {
            'coverage_enabled': True,
            'coverage_scan_topic': 'scan_raw',
            'coverage_fov_deg': 59.0, # Updated to match RGBD camera FOV
            'coverage_yaw_offset_deg': 0.0,
            'coverage_range_m': 3.0,
            'coverage_min_area_m2': 0.15,
            'coverage_dwell_s': 1.0,
            'coverage_cooldown_s': 5.0,
            'coverage_scan_timeout_s': 1.0,
            'coverage_inf_is_clear': True,
            'map_timeout_s': 15.0,
            'tf_timeout_s': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {
            name: self.get_parameter(name).value for name in defaults
        }
        positive = (
            'coverage_range_m', 'coverage_min_area_m2',
            'coverage_dwell_s', 'coverage_cooldown_s',
            'coverage_scan_timeout_s', 'map_timeout_s', 'tf_timeout_s',
        )
        if any(not math.isfinite(self.settings[name])
               or self.settings[name] <= 0.0 for name in positive):
            raise ValueError('Coverage distances and times must be positive')
        if not 0.0 < self.settings['coverage_fov_deg'] <= 360.0:
            raise ValueError('coverage_fov_deg must be in (0, 360]')
        if not math.isfinite(self.settings['coverage_yaw_offset_deg']):
            raise ValueError('coverage_yaw_offset_deg must be finite')
        self.view_fov = math.radians(self.settings['coverage_fov_deg'])
        self.view_offset = math.radians(
            self.settings['coverage_yaw_offset_deg'])

        latched = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.frontier_grid_pub = self.create_publisher(
            OccupancyGrid, 'exploration/frontier_cells', latched)
        self.target_marker_pub = self.create_publisher(
            Marker, 'exploration/target_marker', 10)
        self.frontier_markers_pub = self.create_publisher(
            MarkerArray, 'exploration/frontier_markers', 10)
        self.path_pub = self.create_publisher(
            Path, 'exploration/planned_path', 10)
        self.seen_pub = self.create_publisher(
            OccupancyGrid, 'exploration/seen_cells', latched)
        self.seen_viz_pub = self.create_publisher(
            OccupancyGrid, 'exploration/seen_cells_viz', latched)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, 'ap/cmd_vel', 10)
        self.scan_pub = self.create_publisher(
            LaserScan, 'scan', qos_profile_sensor_data)

        self.status_sub = self.create_subscription(
            Status, 'ap/status', self.status_callback, qos_profile_sensor_data)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.person_pins_sub = self.create_subscription(
            MarkerArray, 'person_map_pins', self.person_pins_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.settings['coverage_scan_topic'],
            self.scan_callback, qos_profile_sensor_data)
        self.reset_service = self.create_service(
            Empty, 'exploration/reset_coverage', self.reset_coverage)
        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.arm_client = self.create_client(ArmMotors, 'ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, 'ap/mode_switch')
        self.takeoff_client = self.create_client(
            Takeoff, 'ap/experimental/takeoff')
        self.service_pending = {}
        self.service_attempts = {}

        self.current_status = None
        self.status_received_at = -math.inf
        self.state = 'INIT'
        self.latest_map = None
        self.map_received_at = -math.inf
        self.map_received_count = 0
        self.known_survivors = set()
        self.survivor_locations = []
        self.takeoff_altitude = 1.0
        self.entry_x = None
        self.entry_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.measured_speed = math.inf
        self.previous_pose = None
        self.tf_valid = False
        self.target_x = None
        self.target_y = None
        self.last_target_time = 0.0
        self.cached_world_path = []
        self.last_plan_time = 0.0
        self.blacklisted_frontiers = []
        self.no_frontier_count = 0
        self.NO_FRONTIER_PATIENCE = 80
        self.STABILISE_SECONDS = 5.0
        self.stabilise_until = 0.0
        self.takeoff_time = 0.0
        self.max_speed = 0.3
        self.max_yaw_rate = 0.4
        self.FREE_THRESH = 50
        self.OCC_THRESH = 50
        self.latest_scan_msg = None

        self.coverage = ViewCoverage(self.OCC_THRESH)
        self.pending_scans = deque(maxlen=10)
        self.last_scan_stamp = -math.inf
        self.scan_sequence = 0
        self.effective_view_range = self.settings['coverage_range_m']
        self.look_phase = 'IDLE'
        self.look_yaw = None
        self.resume_yaw = None
        self.look_phase_started = 0.0
        self.look_started = 0.0
        self.look_count = 0
        self.look_scan_sequence = 0
        self.look_aligned_at = None
        self.look_cooldown_until = 0.0
        self.look_checked_at = -math.inf
        self.look_history = []
        self.last_coverage_publish = -math.inf
        self.previous_loop_time = self.now_seconds()
        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info('Frontier exploration with view coverage ready')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def status_callback(self, message):
        self.current_status = message
        self.status_received_at = self.now_seconds()

    def map_callback(self, message):
        try:
            self.coverage.set_map(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self.latest_map = message
        self.map_received_at = self.now_seconds()
        self.map_received_count += 1

    def person_pins_callback(self, message):
        for marker in message.markers:
            identity = (marker.ns, marker.id)
            if marker.action == Marker.ADD and identity not in self.known_survivors:
                self.known_survivors.add(identity)
                point = (marker.points[1] if len(marker.points) > 1
                         else marker.pose.position)
                self.survivor_locations.append((point.x, point.y))
                self.get_logger().info(
                    f'SURVIVOR FOUND at X: {point.x:.2f}, Y: {point.y:.2f}')

    def scan_callback(self, message):
        self.pending_scans.append(message)
        self.latest_scan_msg = message
        # Pitch-gating for SLAM
        try:
            # Use current time to avoid dropping scans due to minor TF lag
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            planar_transform(transform.transform)
            self.scan_pub.publish(message)
        except (TransformException, ValueError):
            pass

    def reset_coverage(self, request, response):
        if self.coverage.seen is not None:
            self.coverage.seen.fill(0)
        self.look_history.clear()
        self.pending_scans.clear()
        self.last_scan_stamp = -math.inf
        self.look_cooldown_until = 0.0
        self.get_logger().info('View coverage reset; navigation target retained')
        return response

    def process_scans(self, now):
        if self.coverage.info is None:
            return
        for pending_index in range(3):
            if not self.pending_scans:
                break
            scan = self.pending_scans[0]
            timestamp = stamp_seconds(scan.header.stamp)
            age = now - timestamp
            if (timestamp <= self.last_scan_stamp or age < 0.0
                    or age > self.settings['coverage_scan_timeout_s']):
                self.pending_scans.popleft()
                continue
            if (not scan.header.frame_id or not scan.ranges
                    or not math.isfinite(scan.angle_min)
                    or not math.isfinite(scan.angle_increment)
                    or scan.angle_increment == 0.0
                    or not math.isfinite(scan.time_increment)
                    or scan.time_increment < 0.0
                    or not math.isfinite(scan.range_min)
                    or not math.isfinite(scan.range_max)
                    or not 0.0 <= scan.range_min < scan.range_max):
                self.pending_scans.popleft()
                continue
            duration = (len(scan.ranges) - 1) * scan.time_increment
            if duration > self.settings['coverage_scan_timeout_s']:
                self.pending_scans.popleft()
                continue
            if age < duration:
                break
            try:
                scan_time = Time.from_msg(scan.header.stamp)
                end_time = scan_time + Duration(seconds=duration)
                laser = self.tf_buffer.lookup_transform(
                    self.coverage.frame, scan.header.frame_id, scan_time)
                body = self.tf_buffer.lookup_transform(
                    self.coverage.frame, 'base_link', scan_time)
                laser_end = self.tf_buffer.lookup_transform(
                    self.coverage.frame, scan.header.frame_id, end_time)
                body_end = self.tf_buffer.lookup_transform(
                    self.coverage.frame, 'base_link', end_time)
                laser_first = planar_transform(laser.transform)
                laser_last = planar_transform(laser_end.transform)
                body_first = planar_transform(body.transform)
                body_last = planar_transform(body_end.transform)
            except TransformException:
                break
            except ValueError:
                self.pending_scans.popleft()
                continue
            self.pending_scans.popleft()
            self.effective_view_range = min(
                self.settings['coverage_range_m'], scan.range_max)
            observed = self.coverage.observe_scan(
                scan, laser_first, laser_last, body_first, body_last,
                self.view_fov, self.view_offset, self.effective_view_range,
                self.settings['coverage_inf_is_clear'])
            if observed:
                self.last_scan_stamp = timestamp
                self.scan_sequence += 1

    def publish_coverage(self, now):
        if self.coverage.info is None or now - self.last_coverage_publish < 0.5:
            return
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.coverage.frame
        message.info = copy.deepcopy(self.coverage.info)
        message.data = self.coverage.seen.astype(np.int8).ravel().tolist()
        self.seen_pub.publish(message)
        visual = copy.deepcopy(message)
        visual.data = (self.coverage.seen * 100).astype(np.int8).ravel().tolist()
        self.seen_viz_pub.publish(visual)
        self.last_coverage_publish = now

    def next_look_direction(self, now):
        self.look_history = [
            record for record in self.look_history if now - record[3] < 30.0
        ]

        def excluded(desired_yaw):
            return any(
                math.hypot(self.current_x - position_x,
                           self.current_y - position_y) < 0.75
                and abs(wrap_angle(desired_yaw - yaw)) < self.view_fov * 0.5
                for position_x, position_y, yaw, timestamp in self.look_history
            )

        return self.coverage.best_direction(
            self.current_x, self.current_y, self.current_yaw,
            self.view_fov, self.view_offset, self.effective_view_range,
            self.settings['coverage_min_area_m2'], excluded)

    def set_look_phase(self, phase, now):
        self.look_phase = phase
        self.look_phase_started = now
        self.look_aligned_at = None
        self.get_logger().info(f'Coverage: {phase}')

    def freeze_navigation_clocks(self, elapsed):
        self.last_target_time += elapsed
        self.last_plan_time += elapsed

    def finish_look(self, now):
        self.set_look_phase('IDLE', now)
        self.look_cooldown_until = now + self.settings['coverage_cooldown_s']
        self.publish_velocity(0.0, 0.0, 0.0)

    def turn_in_place(self, desired_yaw):
        error = wrap_angle(desired_yaw - self.current_yaw)
        rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, 1.5 * error))
        aligned = abs(error) <= math.radians(5.0)
        self.publish_velocity(0.0, 0.0, 0.0 if aligned else rate)
        return aligned

    def coverage_step(self, now, elapsed):
        """Return True while coverage owns commands; never change the path."""
        if not self.settings['coverage_enabled']:
            return False
        if self.look_phase == 'IDLE':
            if now < self.look_cooldown_until or now - self.look_checked_at < 0.5:
                return False
            self.look_checked_at = now
            desired_yaw = self.next_look_direction(now)
            if desired_yaw is None:
                return False
            self.resume_yaw = self.current_yaw
            self.look_yaw = desired_yaw
            self.look_count = 0
            self.look_started = now
            self.set_look_phase('BRAKE', now)

        self.freeze_navigation_clocks(elapsed)
        phase_age = now - self.look_phase_started
        if self.look_phase == 'BRAKE':
            self.publish_velocity(0.0, 0.0, 0.0)
            if self.measured_speed < 0.08:
                if self.look_aligned_at is None:
                    self.look_aligned_at = now
                if phase_age >= 1.0 and now - self.look_aligned_at >= 0.6:
                    self.set_look_phase('TURN', now)
            else:
                self.look_aligned_at = None
            if phase_age > 5.0 and self.look_phase == 'BRAKE':
                self.get_logger().warning('Unable to settle; postponing look')
                self.finish_look(now)

        elif self.look_phase == 'TURN':
            if self.turn_in_place(self.look_yaw):
                self.set_look_phase('DWELL', now)
                self.look_scan_sequence = self.scan_sequence
            elif phase_age > 20.0:
                self.look_history.append((
                    self.current_x, self.current_y, self.look_yaw, now))
                self.get_logger().warning('Look timed out; unseen cells stay zero')
                self.set_look_phase('RESTORE', now)

        elif self.look_phase == 'DWELL':
            if not self.turn_in_place(self.look_yaw):
                self.set_look_phase('TURN', now)
            elif (phase_age >= self.settings['coverage_dwell_s']
                  and self.scan_sequence > self.look_scan_sequence
                  and self.last_scan_stamp >= self.look_phase_started):
                self.look_history.append((
                    self.current_x, self.current_y, self.look_yaw, now))
                self.look_count += 1
                desired_yaw = self.next_look_direction(now)
                if (desired_yaw is not None and self.look_count < 3
                        and now - self.look_started < 40.0):
                    self.look_yaw = desired_yaw
                    self.set_look_phase('TURN', now)
                else:
                    self.set_look_phase('RESTORE', now)

        elif self.look_phase == 'RESTORE':
            if self.turn_in_place(self.resume_yaw):
                if self.look_aligned_at is None:
                    self.look_aligned_at = now
                if now - self.look_aligned_at >= 0.5:
                    self.finish_look(now)
            else:
                self.look_aligned_at = None
            if phase_age > 20.0:
                self.get_logger().warning(
                    'Yaw restoration delayed; translation remains stopped',
                    throttle_duration_sec=5.0)
        if (self.look_phase in ('TURN', 'DWELL')
                and now - self.look_started > 45.0):
            self.set_look_phase('RESTORE', now)
        return True

    def get_reachable_mask(self, free_mask, drone_row, drone_column):
        height, width = free_mask.shape
        row = max(0, min(height - 1, drone_row))
        column = max(0, min(width - 1, drone_column))
        if not free_mask[row, column]:
            found = None
            for radius in range(1, 10):
                nearby = np.argwhere(free_mask[
                    max(0, row - radius):min(height, row + radius + 1),
                    max(0, column - radius):min(width, column + radius + 1)])
                if len(nearby):
                    found = (int(nearby[0, 0]) + max(0, row - radius),
                             int(nearby[0, 1]) + max(0, column - radius))
                    break
            if found is None:
                return np.zeros_like(free_mask)
            row, column = found
        image = free_mask.astype(np.uint8) * 255
        mask = np.zeros((height + 2, width + 2), dtype=np.uint8)
        cv2.floodFill(image, mask, (column, row), 128)
        return image == 128

    def find_frontiers(self):
        if self.latest_map is None:
            return None, []
        info = self.latest_map.info
        data = self.coverage.data
        unknown = (data == -1).astype(np.uint8) * 255
        free_mask = (data >= 0) & (data < self.FREE_THRESH)
        obstacles = (data >= self.OCC_THRESH).astype(np.uint8) * 255
        
        # Inject survivor obstacles
        for px, py in self.survivor_locations:
            r, c = ViewCoverage.grid_coordinates(info, px, py)
            r, c = int(r), int(c)
            if 0 <= r < info.height and 0 <= c < info.width:
                cv2.circle(obstacles, (c, r), int(0.50 / info.resolution), 255, -1)
                
        clearance = int(0.30 / info.resolution)
        safe_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * clearance + 1, 2 * clearance + 1))
        obstacles_dilated = cv2.dilate(obstacles, safe_kernel)
        rows, columns = ViewCoverage.grid_coordinates(
            info, self.current_x, self.current_y)
        drone_row, drone_column = int(rows), int(columns)
        reachable = self.get_reachable_mask(free_mask, drone_row, drone_column)
        unknown_dilated = cv2.dilate(unknown, np.ones((3, 3), np.uint8))
        frontiers = cv2.bitwise_and(
            unknown_dilated, reachable.astype(np.uint8) * 255)
        frontiers = cv2.bitwise_and(frontiers, cv2.bitwise_not(obstacles_dilated))
        cv2.circle(frontiers, (drone_column, drone_row),
                   int(0.4 / info.resolution), 0, -1)
        self.publish_frontier_grid(frontiers, info)
        if np.count_nonzero(frontiers) < 30:
            return None, []
        label_count, labels, statistics, centroids = cv2.connectedComponentsWithStats(
            frontiers, connectivity=8)
        candidates, all_centroids = [], []
        for label_id in range(1, label_count):
            area = float(statistics[label_id, cv2.CC_STAT_AREA])
            if area < 2:
                continue
            rows, columns = np.where(labels == label_id)
            center_column, center_row = centroids[label_id]
            nearest = np.argmin((columns - center_column) ** 2
                                + (rows - center_row) ** 2)
            world_x, world_y = ViewCoverage.world_coordinates(
                info, rows[nearest], columns[nearest])
            world_x, world_y = float(world_x), float(world_y)
            
            # Dynamic bounding to keep exploration strictly inside 15m of start
            if self.entry_x is not None:
                if math.hypot(world_x - self.entry_x, world_y - self.entry_y) > 15.0:
                    continue
                
            distance = math.hypot(world_x - self.current_x, world_y - self.current_y)
            all_centroids.append((world_x, world_y, area))
            if distance <= 0.3:
                continue
            if any(math.hypot(world_x - blocked_x, world_y - blocked_y) < 1.0
                   for blocked_x, blocked_y in self.blacklisted_frontiers):
                continue
            distance_home = (math.hypot(world_x - self.entry_x, world_y - self.entry_y)
                             if self.entry_x is not None else 0.0)
            score = area / (distance + 0.5) * (distance_home + 1.0)
            candidates.append((score, world_x, world_y))
        if not candidates:
            return None, all_centroids
        best = max(candidates, key=lambda candidate: candidate[0])
        return (best[1], best[2]), all_centroids

    def publish_frontier_grid(self, image, info):
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.coverage.frame
        message.info = copy.deepcopy(info)
        data = np.full(image.shape, -1, dtype=np.int8)
        data[image > 0] = 100
        message.data = data.ravel().tolist()
        self.frontier_grid_pub.publish(message)

    def publish_target_marker(self, world_x, world_y):
        marker = Marker()
        marker.header.frame_id = self.coverage.frame or 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'exploration_target'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [
            Point(x=float(self.current_x), y=float(self.current_y), z=float(self.current_z)),
            Point(x=float(world_x), y=float(world_y), z=float(self.current_z)),
        ]
        marker.scale.x = 0.08
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)
        marker.lifetime.sec = 2
        self.target_marker_pub.publish(marker)

    def publish_frontier_centroid_markers(self, centroids):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.coverage.frame
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, (world_x, world_y, area) in enumerate(centroids):
            marker = Marker()
            marker.header.frame_id = self.coverage.frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontier_centroids'
            marker.id = index + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(world_x)
            marker.pose.position.y = float(world_y)
            marker.pose.position.z = float(self.current_z)
            marker.pose.orientation.w = 1.0
            size = max(0.1, min(0.5, area * 0.02))
            marker.scale.x = marker.scale.y = marker.scale.z = size
            marker.color = ColorRGBA(r=0.2, g=min(1.0, area / 30.0), b=0.1, a=0.8)
            marker.lifetime.sec = 3
            markers.markers.append(marker)
        self.frontier_markers_pub.publish(markers)

    def update_pose(self, now):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.coverage.frame or 'map', 'base_link', Time())
        except TransformException:
            self.tf_valid = False
            return False
        timestamp = stamp_seconds(transform.header.stamp)
        if not 0.0 <= now - timestamp <= self.settings['tf_timeout_s']:
            self.tf_valid = False
            return False
        position = transform.transform.translation
        self.current_x, self.current_y, self.current_z = position.x, position.y, position.z
        self.current_yaw = quaternion_yaw(transform.transform.rotation)
        if self.previous_pose is not None:
            old_x, old_y, old_time = self.previous_pose
            if timestamp > old_time:
                self.measured_speed = math.hypot(
                    position.x - old_x, position.y - old_y) / (timestamp - old_time)
        self.previous_pose = (position.x, position.y, timestamp)
        self.tf_valid = True
        return True

    def control_loop(self):
        now = self.now_seconds()
        elapsed = max(0.0, now - self.previous_loop_time)
        if now < self.previous_loop_time:
            self.publish_velocity(0.0, 0.0, 0.0)
            self.state = 'CLOCK_RESET_HOLD'
            self.pending_scans.clear()
            self.last_scan_stamp = -math.inf
            if self.coverage.seen is not None:
                self.coverage.seen.fill(0)
            self.get_logger().error('ROS clock reset: restart mission after verifying vehicle state')
        self.previous_loop_time = now
        if self.state == 'CLOCK_RESET_HOLD':
            self.publish_velocity(0.0, 0.0, 0.0)
            return
        if self.current_status is None or now - self.status_received_at > 3.0:
            self.publish_velocity(0.0, 0.0, 0.0)
            self.freeze_navigation_clocks(elapsed)
            return
        if self.state == 'FINISHED':
            self.timer.cancel()
            rclpy.shutdown()
            return
        if self.state == 'LANDING':
            if not self.current_status.armed:
                self.state = 'FINISHED'
                self.get_logger().info('Disarmed after landing; mission complete')
            elif self.current_status.mode != 9:
                self.request_mode(9)
            return
        if not self.update_pose(now) and self.state != 'CLIMBING':
            self.publish_velocity(0.0, 0.0, 0.0)
            self.freeze_navigation_clocks(elapsed)
            return

        if self.state == 'INIT':
            self.state = 'SWITCH_TO_GUIDED'
            self.request_mode(4)
        elif self.state == 'SWITCH_TO_GUIDED':
            if self.current_status.mode == 4:
                self.state = 'ARMING'
                self.request_arm(True)
            else:
                self.request_mode(4)
        elif self.state == 'ARMING':
            if self.current_status.armed:
                self.state = 'TAKEOFF_REQUEST'
            else:
                self.request_arm(True)
        elif self.state == 'TAKEOFF_REQUEST':
            if self.request_takeoff(self.takeoff_altitude):
                self.takeoff_time = now
                self.state = 'CLIMBING'
        elif self.state == 'CLIMBING':
            if now - self.takeoff_time > 10.0 and self.tf_valid:
                self.entry_x, self.entry_y = self.current_x, self.current_y
                self.stabilise_until = now + self.STABILISE_SECONDS
                self.state = 'STABILISE'
        elif self.state == 'STABILISE':
            self.publish_velocity(0.0, 0.0, 0.0)
            if now >= self.stabilise_until and self.latest_map is not None:
                self.state = 'EXPLORE'
                self.no_frontier_count = 0
        elif self.state in ('EXPLORE', 'RETURN_HOME'):
            if (self.current_status.mode != 4 or not self.current_status.armed
                    or self.latest_map is None
                    or now - self.map_received_at > self.settings['map_timeout_s']):
                self.publish_velocity(0.0, 0.0, 0.0)
                self.freeze_navigation_clocks(elapsed)
                return
            self.process_scans(now)
            self.publish_coverage(now)
            if (self.settings['coverage_enabled']
                    and now - self.last_scan_stamp > self.settings['coverage_scan_timeout_s']):
                self.get_logger().warning(
                    'Scan timeout; suspending coverage but continuing navigation',
                    throttle_duration_sec=3.0)
            elif self.state == 'EXPLORE' and self.coverage_step(now, elapsed):
                if self.target_x is not None:
                    self.publish_target_marker(self.target_x, self.target_y)
                return
            if self.state == 'EXPLORE':
                self.explore_step(now)
            else:
                self.return_home_step()

    def explore_step(self, now):
        need_target = self.target_x is None
        if not need_target:
            need_target = (
                math.hypot(self.target_x - self.current_x,
                           self.target_y - self.current_y) < 0.5
                or now - self.last_target_time > 15.0)
        if need_target:
            target, centroids = self.find_frontiers()
            self.publish_frontier_centroid_markers(centroids)
            if target is not None:
                self.target_x, self.target_y = target
                self.last_target_time = now
                self.no_frontier_count = 0
                self.cached_world_path = []
            else:
                self.no_frontier_count += 1
                if self.no_frontier_count >= self.NO_FRONTIER_PATIENCE:
                    self.target_x, self.target_y = self.entry_x, self.entry_y
                    self.cached_world_path = []
                    self.state = 'RETURN_HOME'
                    self.publish_velocity(0.0, 0.0, 0.0)
                    self.get_logger().info('Frontier heuristic complete; returning home')
                    return
                if self.target_x is None:
                    self.publish_velocity(0.0, 0.0, 0.0)
                    return
        self.publish_target_marker(self.target_x, self.target_y)
        self.navigate_to_target()

    def return_home_step(self):
        distance = math.hypot(self.entry_x - self.current_x,
                              self.entry_y - self.current_y)
        self.publish_target_marker(self.entry_x, self.entry_y)
        if distance < 0.5:
            self.publish_velocity(0.0, 0.0, 0.0)
            self.state = 'LANDING'
            self.request_mode(9)
        else:
            self.target_x, self.target_y = self.entry_x, self.entry_y
            self.navigate_to_target()

    def navigate_to_target(self):
        if self.target_x is None or self.target_y is None:
            self.publish_velocity(0.0, 0.0, 0.0)
            return
        now = self.now_seconds()
        if not self.cached_world_path or now - self.last_plan_time > 1.0:
            info = self.latest_map.info
            obstacles = (self.coverage.data >= self.OCC_THRESH).astype(np.uint8) * 255
            
            # Inject survivor obstacles
            for px, py in self.survivor_locations:
                r, c = ViewCoverage.grid_coordinates(info, px, py)
                r, c = int(r), int(c)
                if 0 <= r < info.height and 0 <= c < info.width:
                    cv2.circle(obstacles, (c, r), int(0.50 / info.resolution), 255, -1)
                    
            distance_transform = cv2.distanceTransform(
                cv2.bitwise_not(obstacles), cv2.DIST_L2, 5)
            start_rows, start_columns = ViewCoverage.grid_coordinates(
                info, self.current_x, self.current_y)
            goal_rows, goal_columns = ViewCoverage.grid_coordinates(
                info, self.target_x, self.target_y)
            start_column = max(0, min(info.width - 1, int(start_columns)))
            start_row = max(0, min(info.height - 1, int(start_rows)))
            goal_column = max(0, min(info.width - 1, int(goal_columns)))
            goal_row = max(0, min(info.height - 1, int(goal_rows)))
            path = self.plan_path_astar(
                start_column, start_row, goal_column, goal_row,
                distance_transform, info.resolution)
            if not path:
                self.cached_world_path = []
                if self.state == 'EXPLORE':
                    self.blacklisted_frontiers.append((self.target_x, self.target_y))
                    self.target_x = self.target_y = None
                self.publish_velocity(0.0, 0.0, 0.0)
                self.get_logger().warning('A* failed; holding or selecting another frontier',
                                          throttle_duration_sec=3.0)
                return
            self.cached_world_path = [
                ViewCoverage.world_coordinates(info, row, column)
                for column, row in path
            ]
            self.last_plan_time = now
        path_message = Path()
        path_message.header.stamp = self.get_clock().now().to_msg()
        path_message.header.frame_id = self.coverage.frame
        for world_x, world_y in self.cached_world_path:
            pose = PoseStamped()
            pose.header = copy.deepcopy(path_message.header)
            pose.pose.position.x = float(world_x)
            pose.pose.position.y = float(world_y)
            pose.pose.orientation.w = 1.0
            path_message.poses.append(pose)
        self.path_pub.publish(path_message)
        closest = min(range(len(self.cached_world_path)), key=lambda index: math.hypot(
            self.cached_world_path[index][0] - self.current_x,
            self.cached_world_path[index][1] - self.current_y))
        pursuit_x, pursuit_y = self.cached_world_path[-1]
        for world_x, world_y in self.cached_world_path[closest:]:
            if math.hypot(world_x - self.current_x, world_y - self.current_y) >= 0.6:
                pursuit_x, pursuit_y = world_x, world_y
                break
        desired_yaw = math.atan2(pursuit_y - self.current_y,
                                 pursuit_x - self.current_x)
        yaw_error = wrap_angle(desired_yaw - self.current_yaw)
        distance = math.hypot(self.target_x - self.current_x,
                              self.target_y - self.current_y)
        speed = min(self.max_speed, distance * 0.3)
        body_vx = math.cos(yaw_error) * speed
        body_vy = math.sin(yaw_error) * speed
        if abs(yaw_error) > 0.5:
            body_vx *= 0.2
            body_vy *= 0.2
        change_x, change_y = body_vx - self.current_vx, body_vy - self.current_vy
        magnitude = math.hypot(change_x, change_y)
        maximum_change = 0.2 * 0.2
        if magnitude > maximum_change:
            change_x *= maximum_change / magnitude
            change_y *= maximum_change / magnitude
        self.current_vx += change_x
        self.current_vy += change_y
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_error * 1.5))
        self.publish_velocity(self.current_vx, self.current_vy, yaw_rate)

    def plan_path_astar(self, start_column, start_row, goal_column,
                       goal_row, distance_transform, resolution):
        height, width = distance_transform.shape
        start, goal = (start_column, start_row), (goal_column, goal_row)
        if start == goal:
            return [start]
        open_set = [(0.0, start)]
        costs, parents = {start: 0.0}, {}
        directions = [
            (0, -1, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (1, 0, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
        ]
        safe_radius = 0.30 / resolution
        preferred_distance = 1.0 / resolution
        while open_set:
            priority, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in parents:
                    path.append(current)
                    current = parents[current]
                return list(reversed(path))
            column, row = current
            for delta_column, delta_row, base_cost in directions:
                next_column, next_row = column + delta_column, row + delta_row
                if not (0 <= next_column < width and 0 <= next_row < height):
                    continue
                distance = distance_transform[next_row, next_column]
                if distance <= safe_radius:
                    continue
                penalty = 0.0
                if distance < preferred_distance:
                    ratio = (preferred_distance - distance) / (preferred_distance - safe_radius)
                    penalty = 20.0 * ratio ** 3
                tentative_cost = costs[current] + base_cost + penalty
                neighbor = (next_column, next_row)
                if tentative_cost < costs.get(neighbor, math.inf):
                    costs[neighbor] = tentative_cost
                    parents[neighbor] = current
                    heuristic = math.hypot(goal_column - next_column, goal_row - next_row)
                    heapq.heappush(open_set, (tentative_cost + heuristic, neighbor))
        return []

    def publish_velocity(self, velocity_x, velocity_y, yaw_rate):
        if self.latest_scan_msg is not None and (velocity_x != 0.0 or velocity_y != 0.0):
            # Reactive collision avoidance layer
            scan = self.latest_scan_msg
            travel_yaw = math.atan2(velocity_y, velocity_x)
            safe = True
            for i, r in enumerate(scan.ranges):
                if not (scan.range_min <= r <= scan.range_max) or not math.isfinite(r):
                    continue
                angle = scan.angle_min + i * scan.angle_increment
                # Only care about obstacles in our travel cone (+/- 45 deg)
                if abs(wrap_angle(angle - travel_yaw)) < math.radians(45.0):
                    # Project range onto travel direction
                    if r * math.cos(wrap_angle(angle - travel_yaw)) < 0.35:
                        safe = False
                        break
            if not safe:
                self.get_logger().warning('Reactive safety triggered; stopping', throttle_duration_sec=1.0)
                velocity_x = velocity_y = 0.0

        if velocity_x == 0.0 and velocity_y == 0.0:
            self.current_vx = self.current_vy = 0.0
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = float(velocity_x)
        message.twist.linear.y = float(velocity_y)
        message.twist.linear.z = 0.0
        message.twist.angular.z = float(yaw_rate)
        self.cmd_vel_pub.publish(message)

    def send_service(self, name, client, request):
        now = self.now_seconds()
        pending = self.service_pending.get(name)
        if pending is not None and not pending.done():
            return False
        if now - self.service_attempts.get(name, -math.inf) < 1.0:
            return False
        self.service_attempts[name] = now
        if not client.service_is_ready():
            return False
        future = client.call_async(request)
        self.service_pending[name] = future

        def completed(result):
            try:
                response = result.result()
                self.get_logger().info(f'{name} response: {response}')
            except Exception as error:
                self.get_logger().error(f'{name} request failed: {error}')

        future.add_done_callback(completed)
        return True

    def request_arm(self, value):
        request = ArmMotors.Request()
        request.arm = value
        return self.send_service('arm', self.arm_client, request)

    def request_mode(self, value):
        request = ModeSwitch.Request()
        request.mode = int(value)
        return self.send_service('mode', self.mode_client, request)

    def request_takeoff(self, altitude):
        request = Takeoff.Request()
        request.alt = float(altitude)
        return self.send_service('takeoff', self.takeoff_client, request)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.publish_velocity(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
