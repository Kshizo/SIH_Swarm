#!/usr/bin/env python3

import os
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

from ai_edge_litert.interpreter import Interpreter
from ament_index_python.packages import get_package_share_directory


def quat_to_matrix(q):
    """Converts [x, y, z, w] quaternion to a 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
    ])


class PersonTrack:
    """Tracks a candidate person across consecutive frames."""
    def __init__(self, track_id, pos):
        self.track_id = track_id
        self.positions = [pos]
        self.misses = 0
        self.confirmed = False

    def update(self, pos):
        self.positions.append(pos)
        if len(self.positions) > 10:
            self.positions.pop(0)
        self.misses = 0

    def get_mean_position(self):
        return np.mean(self.positions, axis=0)

    def is_stable(self, min_frames=5, max_spread=1.0):
        if len(self.positions) < min_frames:
            return False
        mean_pos = self.get_mean_position()
        spread = np.max(np.linalg.norm(np.array(self.positions) - mean_pos, axis=1))
        return spread <= max_spread


class PersonDetector(Node):

    def __init__(self):
        super().__init__('person_detector')

        # ---------------- CONFIGURATION ----------------
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('pose_topic', '/ap/pose/filtered')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('camera_pitch', 0.4109)
        self.declare_parameter('camera_hfov', 1.03)

        camera_topic = self.get_parameter('camera_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.camera_pitch = self.get_parameter('camera_pitch').value
        self.hfov = self.get_parameter('camera_hfov').value

        # Camera geometry relative to drone base_link
        self.cam_offset_body = np.array([0.10, 0.0, 0.03])
        p = self.camera_pitch
        self.R_body_cam = np.array([
            [math.cos(p), 0.0, math.sin(p)],
            [0.0,         1.0, 0.0],
            [-math.sin(p), 0.0, math.cos(p)]
        ])

        # State & Tracking
        self.drone_pos = None
        self.drone_rot = None
        self.tracks = []
        self.next_track_id = 0
        self.confirmed_survivors = []  # Spatial database of [x,y,z] for all unique survivors

        # ---------------- MODEL ----------------
        package_dir = get_package_share_directory('ssd_lite_ros')
        model_path = os.path.join(package_dir, 'models', 'detect.tflite')

        self.interpreter = Interpreter(model_path=model_path, num_threads=2)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_idx = self.input_details[0]['index']
        self.input_h = self.input_details[0]['shape'][1]
        self.input_w = self.input_details[0]['shape'][2]

        # ---------------- ROS COMMUNICATIONS ----------------
        self.bridge = CvBridge()

        # TF2 Setup for precise location instead of ArduPilot pose
        import tf2_ros
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=60.0), node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.image_sub = message_filters.Subscriber(self, Image, camera_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        self.person_pub = self.create_publisher(Bool, '/person_detected', 10)
        self.detection_image_pub = self.create_publisher(Image, '/detection/image', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/person_map_pins', 10)

        self.get_logger().info('Person Detector (RGB-D) & Multi-Person Locator running...')

    def backproject_depth(self, u, v, depth, w, h):
        """Projects a 2D pixel with a depth value into 3D world space."""
        if self.drone_pos is None or self.drone_rot is None:
            return None

        fx = (w / 2.0) / math.tan(self.hfov / 2.0)
        fy = fx
        cx, cy = w / 2.0, h / 2.0

        # Point in camera frame (X forward, Y left, Z up convention)
        ray_cam = np.array([depth, -(u - cx) * depth / fx, -(v - cy) * depth / fy])

        # Transform to body frame then world frame
        pt_world = self.drone_rot @ (self.R_body_cam @ ray_cam)
        cam_world = self.drone_pos + (self.drone_rot @ self.cam_offset_body)

        return cam_world + pt_world

    def update_tracks(self, detected_positions):
        """Associates new detections to existing tracks or creates new ones."""
        assigned_tracks = set()

        for pos in detected_positions:
            matched_track = None
            min_dist = 1.5  # Max 1.5 m threshold to associate with same person

            for track in self.tracks:
                if track.track_id in assigned_tracks:
                    continue
                dist = np.linalg.norm(track.get_mean_position() - pos)
                if dist < min_dist:
                    min_dist = dist
                    matched_track = track

            if matched_track:
                matched_track.update(pos)
                assigned_tracks.add(matched_track.track_id)
            else:
                # New person detected
                new_track = PersonTrack(self.next_track_id, pos)
                self.next_track_id += 1
                self.tracks.append(new_track)
                assigned_tracks.add(new_track.track_id)

        # Increment misses for unseen tracks, remove if lost > 6 frames
        for track in self.tracks:
            if track.track_id not in assigned_tracks:
                track.misses += 1

        self.tracks = [t for t in self.tracks if t.misses <= 6]

        # Check if any tracks are newly confirmed
        for track in self.tracks:
            if not track.confirmed and track.is_stable(min_frames=5, max_spread=1.0):
                track.confirmed = True
                mean_pos = track.get_mean_position()
                
                # Check against spatial database to prevent duplicates
                is_duplicate = False
                for existing_pos in self.confirmed_survivors:
                    if np.linalg.norm(existing_pos - mean_pos) < 2.0:
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    self.confirmed_survivors.append(mean_pos)
                    self.get_logger().info(
                        f'📍 [NEW SURVIVOR FOUND] Total: {len(self.confirmed_survivors)} -> X: {mean_pos[0]:.2f}m, Y: {mean_pos[1]:.2f}m'
                    )
                else:
                    self.get_logger().info('⚠️ [DUPLICATE SURVIVOR] Already mapped.')

    def publish_map_pins(self):
        """Publishes pin markers for all confirmed people."""
        marker_array = MarkerArray()

        for idx, pos in enumerate(self.confirmed_survivors):
            # 2D Disc acting as a map pin
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'people_pins'
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Place flat on the ground
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = 0.05
            
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.60  # 60cm diameter circle
            marker.scale.y = 0.60
            marker.scale.z = 0.05  # Flat height

            # Bright Red Pin
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)

    def sync_callback(self, rgb_msg: Image, depth_msg: Image):
        # 0. Get Drone Pose from TF (Manual Composition to avoid SLAM time lag errors)
        try:
            # Get latest odom to base_link (fast, 20Hz)
            t_base = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            pos_b = t_base.transform.translation
            rot_b = t_base.transform.rotation
            pos_odom = np.array([pos_b.x, pos_b.y, pos_b.z])
            rot_odom = quat_to_matrix([rot_b.x, rot_b.y, rot_b.z, rot_b.w])
            
            # Get latest map to odom (slow, SLAM dependent)
            t_map = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            pos_m = t_map.transform.translation
            rot_m = t_map.transform.rotation
            map_trans = np.array([pos_m.x, pos_m.y, pos_m.z])
            map_rot = quat_to_matrix([rot_m.x, rot_m.y, rot_m.z, rot_m.w])
            
            # Combine to get instantaneous base_link in map frame
            self.drone_pos = map_trans + (map_rot @ pos_odom)
            self.drone_rot = map_rot @ rot_odom
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            # Depth comes in as 32FC1 (meters)
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        h, w = frame.shape[:2]

        # 1. Run Inference
        resized = cv2.resize(frame, (self.input_w, self.input_h))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        input_data = np.expand_dims(rgb.astype(np.uint8), axis=0)

        self.interpreter.set_tensor(self.input_idx, input_data)
        self.interpreter.invoke()

        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
        num_dets = int(self.interpreter.get_tensor(self.output_details[3]['index'])[0])

        current_frame_positions = []
        person_detected = False

        # 2. Extract Detections & Project using Depth
        for i in range(num_dets):
            if int(classes[i]) == 0 and float(scores[i]) >= self.confidence_threshold:
                person_detected = True
                conf = float(scores[i])

                ymin, xmin, ymax, xmax = boxes[i]
                x1, y1 = max(0, int(xmin * w)), max(0, int(ymin * h))
                x2, y2 = min(w - 1, int(xmax * w)), min(h - 1, int(ymax * h))

                # Use the center of the bounding box to sample depth
                center_u = int((x1 + x2) / 2.0)
                center_v = int((y1 + y2) / 2.0)
                
                # Protect against out-of-bounds
                center_u = min(max(center_u, 0), w - 1)
                center_v = min(max(center_v, 0), h - 1)

                depth_val = depth_img[center_v, center_u]

                pos = None
                dist_str = ""
                # Check for valid depth (not NaN or Inf, > 0.1m, < 10m)
                if not np.isnan(depth_val) and 0.1 < depth_val < 10.0:
                    pos = self.backproject_depth(center_u, center_v, float(depth_val), w, h)
                    
                if pos is not None:
                    current_frame_positions.append(pos)
                    if self.drone_pos is not None:
                        dist = np.linalg.norm(pos - self.drone_pos)
                        dist_str = f" | {dist:.1f}m"

                # Draw bounding box & label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw center dot where depth was sampled
                cv2.circle(frame, (center_u, center_v), 4, (0, 0, 255), -1)
                
                label = f"Person {conf:.2f}{dist_str}"
                cv2.putText(frame, label, (x1, max(20, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 3. Update Multi-Person Tracks & Drop Markers
        if current_frame_positions:
            self.update_tracks(current_frame_positions)
            self.publish_map_pins()

        # Display confirmed status on frame
        confirmed_count = len(self.confirmed_survivors)
        if confirmed_count > 0:
            cv2.putText(frame, f"Confirmed Pins: {confirmed_count}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Publish detections image and flag
        self.person_pub.publish(Bool(data=person_detected))

        det_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        det_msg.header = rgb_msg.header
        self.detection_image_pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()