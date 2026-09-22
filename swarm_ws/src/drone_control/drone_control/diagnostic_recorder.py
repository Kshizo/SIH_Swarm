#!/usr/bin/env python3
import os
import time
import json
import csv
import math
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String

class DiagnosticRecorder(Node):
    def __init__(self):
        super().__init__('diagnostic_recorder')

        # ── Setup logging directory ─────────────────────────────
        home_dir = os.path.expanduser('~')
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(home_dir, 'nidar_diagnostics', f'session_{timestamp}')
        os.makedirs(self.session_dir, exist_ok=True)

        self.telemetry_path = os.path.join(self.session_dir, 'telemetry.csv')
        self.events_path = os.path.join(self.session_dir, 'events.log')

        self.get_logger().info(f"Diagnostic Recorder started. Logging to {self.session_dir}")

        # Open files
        self.telemetry_file = open(self.telemetry_path, 'w', newline='')
        self.csv_writer = csv.writer(self.telemetry_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'state', 'target_x', 'target_y'])
        
        self.events_file = open(self.events_path, 'w')
        self.events_file.write(f"--- NIDAR Session {timestamp} ---\n\n")

        # ── State variables ─────────────────────────────────────
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        
        self.last_state = None
        self.last_target = (None, None)
        self.last_emergency_stop = False
        
        self.current_diag_state = "INIT"
        self.current_target_x = None
        self.current_target_y = None

        # ── Subscribers ─────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            PoseStamped, '/ap/pose/filtered', self.pose_callback, qos_profile_sensor_data)
        self.vel_sub = self.create_subscription(
            TwistStamped, '/ap/cmd_vel', self.vel_callback, 10)
        self.diag_sub = self.create_subscription(
            String, '/exploration/diagnostics', self.diag_callback, 10)

        # ── Telemetry Timer (10Hz) ──────────────────────────────
        self.timer = self.create_timer(0.1, self.telemetry_callback)

    def log_event(self, message):
        t = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted_msg = f"[{t}] {message}\n"
        self.events_file.write(formatted_msg)
        self.events_file.flush()
        self.get_logger().info(message)

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_yaw = self.euler_from_quaternion(msg.pose.orientation)

    def vel_callback(self, msg):
        self.current_vx = msg.twist.linear.x
        self.current_vy = msg.twist.linear.y
        self.current_wz = msg.twist.angular.z

    def diag_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        new_state = data.get("state")
        new_target_x = data.get("target_x")
        new_target_y = data.get("target_y")
        emergency_stop = data.get("emergency_stop", False)

        self.current_diag_state = new_state
        self.current_target_x = new_target_x
        self.current_target_y = new_target_y

        # Detect and log state changes
        if new_state != self.last_state:
            self.log_event(f"STATE CHANGE: {self.last_state} -> {new_state}")
            self.last_state = new_state

        # Detect and log target changes
        if (new_target_x, new_target_y) != self.last_target:
            if new_target_x is not None:
                self.log_event(f"NEW TARGET: Navigating to ({new_target_x:.2f}, {new_target_y:.2f})")
            else:
                self.log_event("TARGET CLEARED.")
            self.last_target = (new_target_x, new_target_y)

        # Detect and log emergency stop transitions
        if emergency_stop and not self.last_emergency_stop:
            self.log_event(f"CRITICAL: Emergency Stop TRIGGERED! Drone at ({self.current_x:.2f}, {self.current_y:.2f})")
        elif not emergency_stop and self.last_emergency_stop:
            self.log_event("CRITICAL: Emergency Stop RESOLVED.")
        self.last_emergency_stop = emergency_stop

    def telemetry_callback(self):
        t = time.time()
        self.csv_writer.writerow([
            f"{t:.3f}", 
            f"{self.current_x:.3f}", 
            f"{self.current_y:.3f}", 
            f"{self.current_yaw:.3f}", 
            f"{self.current_vx:.3f}", 
            f"{self.current_vy:.3f}", 
            f"{self.current_wz:.3f}", 
            self.current_diag_state,
            f"{self.current_target_x:.3f}" if self.current_target_x is not None else "",
            f"{self.current_target_y:.3f}" if self.current_target_y is not None else ""
        ])
        self.telemetry_file.flush()

    def destroy_node(self):
        self.telemetry_file.close()
        self.events_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
