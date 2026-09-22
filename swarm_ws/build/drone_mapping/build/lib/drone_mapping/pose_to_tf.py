#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Pose to TF Broadcaster Initialized.')

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f'Received pose at time {msg.header.stamp.sec}.{msg.header.stamp.nanosec}', throttle_duration_sec=2.0)
        t = TransformStamped()

        # Update the timestamp using ROS 2 sim time to avoid ArduPilot's system time mismatch
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Copy the pose to the transform
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
