#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

import sys
import termios
import tty
import select
import threading
import time

msg = """
Control Your ArduPilot Copter!
---------------------------
Moving around:
        w (Forward)
   a (Left)  s (Back)  d (Right)

z : move up
c : move down

space or k : force stop

Flight Commands:
   1 : ARM Motors
   2 : Set Mode to GUIDED (Required for control)
   3 : TAKEOFF to 3.0m
   4 : Set Mode to LAND
   5 : DISARM Motors

q : quit
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)
        self.callback_group = ReentrantCallbackGroup()
        
        # Service Clients
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors', callback_group=self.callback_group)
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch', callback_group=self.callback_group)
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff', callback_group=self.callback_group)
        
        self.speed = 1.0  # m/s
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw_rate = 0.0
        
        # Lock for velocity updates
        self.lock = threading.Lock()
        self.last_key_time = time.time()
        
        # Start a publisher timer at 20Hz (50ms)
        self.timer = self.create_timer(0.05, self.publish_velocity)
        self.get_logger().info("Teleop Node Initialized. Ready to control.")

    def publish_velocity(self):
        with self.lock:
            now = time.time()
            time_since_key = now - self.last_key_time
            
            # If no key has been pressed for > 0.3 seconds, reset velocity to 0
            if time_since_key > 0.3:
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.yaw_rate = 0.0

            # Only publish if we are actively commanding, OR if we recently released a key
            # (within 0.8s) to send zero-velocity messages briefly so the drone decelerates to hover.
            if (self.x != 0.0 or self.y != 0.0 or self.z != 0.0 or self.yaw_rate != 0.0) or (time_since_key <= 0.8):
                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = 'base_link'
                twist_msg.twist.linear.x = self.x
                twist_msg.twist.linear.y = self.y
                twist_msg.twist.linear.z = self.z
                twist_msg.twist.angular.z = self.yaw_rate
                self.publisher.publish(twist_msg)

    def call_arm(self, arm_state):
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arm service not available")
            return
        req = ArmMotors.Request()
        req.arm = arm_state
        self.get_logger().info(f"Calling ARM service with {arm_state}...")
        self.arm_client.call_async(req)

    def call_mode_switch(self, mode):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Mode switch service not available")
            return
        req = ModeSwitch.Request()
        req.mode = mode
        self.get_logger().info(f"Calling Mode Switch service with mode={mode}...")
        self.mode_client.call_async(req)

    def call_takeoff(self, alt):
        if not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Takeoff service not available")
            return
        req = Takeoff.Request()
        req.alt = float(alt)
        self.get_logger().info(f"Calling Takeoff service with alt={alt}m...")
        self.takeoff_client.call_async(req)


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    
    node = TeleopNode()
    
    # Run the ROS executor in a background thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    print(msg)
    
    try:
        while True:
            key = getKey(settings, 0.1)
            
            if key == 'q':
                break
            
            with node.lock:
                if key == 'w':
                    node.x = node.speed
                    node.last_key_time = time.time()
                elif key == 's':
                    node.x = -node.speed
                    node.last_key_time = time.time()
                elif key == 'a':
                    node.y = -node.speed
                    node.last_key_time = time.time()
                elif key == 'd':
                    node.y = node.speed
                    node.last_key_time = time.time()
                elif key == 'z':
                    node.z = node.speed
                    node.last_key_time = time.time()
                elif key == 'c':
                    node.z = -node.speed
                    node.last_key_time = time.time()
                elif key == ' ' or key == 'k':
                    node.x = 0.0
                    node.y = 0.0
                    node.z = 0.0
                    node.yaw_rate = 0.0
                    node.last_key_time = time.time()
                
            # Service triggers (outside lock, thread-safe asynchronous calls)
            if key == '1':
                node.call_arm(True)
            elif key == '5':
                node.call_arm(False)
            elif key == '2':
                node.call_mode_switch(4)  # GUIDED
            elif key == '3':
                node.call_takeoff(3.0)
            elif key == '4':
                node.call_mode_switch(9)  # LAND

    except Exception as e:
        print(e)
    finally:
        # Reset terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        # Stop node and shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
