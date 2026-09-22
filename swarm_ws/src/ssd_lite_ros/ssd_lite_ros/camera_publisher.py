#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class USBCameraPublisher(Node):

    def __init__(self):
        super().__init__('usb_camera_publisher')

        self.publisher = self.create_publisher(Image, '/front_camera/image_raw',10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

        # Request a lightweight stream
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open USB camera')
            return

        self.timer = self.create_timer(1.0 / 15.0,self.publish_frame)

        self.get_logger().info('USB camera publisher started')

    def publish_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning('Failed to capture frame')
            return

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera'

        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = USBCameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
