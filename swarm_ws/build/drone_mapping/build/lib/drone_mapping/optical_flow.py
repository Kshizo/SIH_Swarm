#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class OpticalFlow(Node):
    def __init__(self):
        super().__init__('optical_flow')

        self.declare_parameter('fov', 1.047)  # Match the 1.047 rad FOV from Gazebo
        self.declare_parameter('fps', 30.0)
        self.fov = self.get_parameter('fov').value
        self.fps = self.get_parameter('fps').value

        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image, '/downward_camera/image', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/downward_lidar/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/optical_flow/odom', 10)

        self.prev_gray = None
        self.prev_time = None
        self.current_altitude = 1.0  # Default to 1m to avoid div by zero initially
        self.wx = 0.0
        self.wy = 0.0

    def imu_callback(self, msg):
        self.wx = msg.angular_velocity.x
        self.wy = msg.angular_velocity.y

    def lidar_callback(self, msg):
        # Update the altitude from the downward-facing 1D LiDAR
        if msg.ranges and math.isfinite(msg.ranges[0]):
            self.current_altitude = msg.ranges[0]

    def image_callback(self, msg):
        current_time = msg.header.stamp
        current_gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        if self.prev_gray is not None and self.prev_time is not None:
            # Calculate dense optical flow using Farneback algorithm
            flow = cv2.calcOpticalFlowFarneback(
                self.prev_gray, current_gray, None, 
                0.5, 3, 15, 3, 5, 1.2, 0)
            
            # Average flow in pixels/frame
            avg_flow_x = np.mean(flow[..., 0])
            avg_flow_y = np.mean(flow[..., 1])
            
            # Calculate time difference
            dt = (current_time.sec - self.prev_time.sec) + \
                 (current_time.nanosec - self.prev_time.nanosec) * 1e-9
            
            if dt > 0:
                # Convert pixel flow to metric velocity
                # Focal length in pixels = (width / 2) / tan(fov / 2)
                width = current_gray.shape[1]
                focal_length = (width / 2.0) / math.tan(self.fov / 2.0)
                
                # Get the apparent angular velocity of the image in radians/sec
                flow_rad_x = avg_flow_x / focal_length / dt
                flow_rad_y = avg_flow_y / focal_length / dt
                
                # To prevent rotation-induced flow from causing a positive feedback loop,
                # we ignore optical flow when the drone is actively rotating.
                # 0.05 rad/s is approx 3 deg/s.
                if abs(self.wx) > 0.05 or abs(self.wy) > 0.05:
                    vx_metric = 0.0
                    vy_metric = 0.0
                    cov_val = 10.0 # High covariance so EKF ignores it
                else:
                    vx_metric = -flow_rad_y * self.current_altitude
                    vy_metric = -flow_rad_x * self.current_altitude
                    cov_val = 0.05 # Normal covariance

                # Publish Odometry
                odom = Odometry()
                odom.header.stamp = current_time
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                
                # We only populate the twist (velocity) part.
                # The EKF will fuse this with the pose from rf2o.
                odom.twist.twist.linear.x = vx_metric
                odom.twist.twist.linear.y = vy_metric
                odom.twist.twist.linear.z = 0.0
                
                cov = np.zeros((6, 6))
                cov[0, 0] = cov_val
                cov[1, 1] = cov_val
                odom.twist.covariance = cov.flatten().tolist()
                
                self.odom_pub.publish(odom)

        self.prev_gray = current_gray
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
