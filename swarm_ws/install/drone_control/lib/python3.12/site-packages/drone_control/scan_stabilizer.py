#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
import math

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

class ScanStabilizer(Node):
    def __init__(self):
        super().__init__('scan_stabilizer')
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            qos_profile_sensor_data)
            
        self.current_roll = 0.0
        self.current_pitch = 0.0
        
        self.PITCH_LIMIT = math.radians(5.0)
        self.ROLL_LIMIT = math.radians(5.0)
        
        self.get_logger().info("Scan Stabilizer Node initialized. Filtering /scan > 5 deg pitch/roll.")

    def pose_callback(self, msg):
        r, p, y = euler_from_quaternion(msg.pose.orientation)
        self.current_roll = r
        self.current_pitch = p
        
    def scan_callback(self, msg):
        if abs(self.current_pitch) > self.PITCH_LIMIT or abs(self.current_roll) > self.ROLL_LIMIT:
            # Drone is tilted. Drop this scan to prevent SLAM ghost walls.
            return
            
        # Drone is stable. Republish.
        self.scan_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
