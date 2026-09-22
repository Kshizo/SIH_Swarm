#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import math
import time

def euler_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class VioToArduPilot(Node):
    def __init__(self):
        super().__init__('vio_to_ardupilot')
        
        self.declare_parameter('sysid', 1)
        self.declare_parameter('initial_yaw', 0.0)
        sysid = self.get_parameter('sysid').value
        self.initial_yaw_rad = math.radians(self.get_parameter('initial_yaw').value)
        port = 14540 + (sysid * 10)
        self.get_logger().info(f"Connecting to ArduPilot via MAVLink on udp:127.0.0.1:{port}")
        self.master = mavutil.mavlink_connection(f'udp:127.0.0.1:{port}')
        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat received. Ready to forward Odometry!")
        
        # Periodically set the EKF Global Origin and Home until ArduPilot accepts it
        self.origin_timer = self.create_timer(5.0, self.send_origin_and_home)
        self.origin_send_count = 0

    def send_origin_and_home(self):
        if self.origin_send_count >= 5:
            self.origin_timer.cancel()
            self.get_logger().info("Finished sending EKF Origin and Home.")
            return
            
        self.get_logger().info("Sending EKF Global Origin and Home to default SITL coordinates...")
        # Set EKF Origin
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            int(-35.363262 * 1e7),   # latitude * 1E7
            int(149.165237 * 1e7),   # longitude * 1E7
            int(584.0 * 1000)        # altitude in mm
        )
        # Set Home Position
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0,
            0, 0, 0, 0,
            -35.363262, 149.165237, 584.0
        )
        
        self.origin_send_count += 1

        if not hasattr(self, 'odom_sub'):
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
        
        self.start_time = time.time()
        self.msg_count = 0

    def odom_callback(self, msg):
        # rf2o odometry is in FLU (Front-Left-Up) frame relative to the starting position.
        # ArduPilot expects FRD (Front-Right-Down) frame for VISION_POSITION_ESTIMATE.
        x_flu = msg.pose.pose.position.x
        y_flu = msg.pose.pose.position.y
        z_flu = msg.pose.pose.position.z 
        
        # FLU to local FRD
        x_local_frd = x_flu
        y_local_frd = -y_flu
        z_frd = -z_flu
        
        q = msg.pose.pose.orientation
        roll_flu, pitch_flu, yaw_flu = euler_from_quaternion(q)
        
        roll_frd = roll_flu
        pitch_frd = -pitch_flu
        yaw_local_frd = -yaw_flu
        
        # Rotate local FRD to global FRD using initial_yaw_rad
        cos_yaw = math.cos(self.initial_yaw_rad)
        sin_yaw = math.sin(self.initial_yaw_rad)
        
        x_frd = x_local_frd * cos_yaw - y_local_frd * sin_yaw
        y_frd = x_local_frd * sin_yaw + y_local_frd * cos_yaw
        yaw_frd = yaw_local_frd + self.initial_yaw_rad
        
        # Extract precise simulation time from the Odometry message header
        # This is CRITICAL for the EKF. If we use wall time, the EKF will reject
        # the measurements because they desynchronize from ArduPilot's clock.
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        usec = int(sec * 1e6 + nanosec / 1000)
        
        self.master.mav.vision_position_estimate_send(
            usec,            # us Timestamp
            x_frd,           # Global X position
            y_frd,           # Global Y position
            z_frd,           # Global Z position
            roll_frd,        # Roll angle
            pitch_frd,       # Pitch angle
            yaw_frd,         # Yaw angle
            [0]*21           # Covariance matrix
        )
        
        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(f"Forwarded {self.msg_count} odometry messages to AP...")

def main(args=None):
    rclpy.init(args=args)
    node = VioToArduPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
