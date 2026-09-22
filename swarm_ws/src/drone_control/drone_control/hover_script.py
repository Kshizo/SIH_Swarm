#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time
from rclpy.qos import qos_profile_sensor_data

class HoverScript(Node):
    def __init__(self):
        super().__init__('hover_script')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile_sensor_data)
        
        # Service Clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # State variables
        self.current_pose = None
        self.current_state = None
        self.state = 'INIT'
        
        # Target parameters
        self.takeoff_altitude = 3.0
        self.hover_duration = 15.0
        
        # Flight tracking variables
        self.hover_start_time = None
        
        # Start state machine loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Hover Node Initialized. State: INIT")

    def pose_callback(self, msg):
        self.current_pose = msg

    def state_callback(self, msg):
        self.current_state = msg

    def control_loop(self):
        # We need telemetry to start
        if self.current_pose is None or self.current_state is None:
            self.get_logger().info("Waiting for pose and state telemetry...", throttle_duration_sec=3.0)
            return

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        self.get_logger().info(f"State: {self.state} | Pos: ({x:.2f}, {y:.2f}, {z:.2f}) | Armed: {self.current_state.armed} | Mode: {self.current_state.mode}", throttle_duration_sec=2.0)

        if self.state == 'INIT':
            # Baseline received, request GUIDED mode
            self.state = 'SWITCH_TO_GUIDED'
            self.request_mode("GUIDED")

        elif self.state == 'SWITCH_TO_GUIDED':
            if self.current_state.mode == "GUIDED":
                self.get_logger().info("Successfully switched to GUIDED mode. Requesting ARM...")
                self.state = 'ARMING'
                self.request_arm(True)
            else:
                self.request_mode("GUIDED")  # Retry

        elif self.state == 'ARMING':
            if self.current_state.armed:
                self.get_logger().info("Motors ARMED. Requesting Takeoff...")
                self.state = 'TAKEOFF_REQUEST'
                self.request_takeoff(self.takeoff_altitude)
            else:
                if not hasattr(self, 'last_arm_request_time') or (time.time() - self.last_arm_request_time > 2.0):
                    self.get_logger().info("Requesting ARM...", throttle_duration_sec=2.0)
                    self.request_arm(True)
                    self.last_arm_request_time = time.time()

        elif self.state == 'TAKEOFF_REQUEST':
            # Takeoff command sent. Switch to climbing.
            self.state = 'CLIMBING'

        elif self.state == 'CLIMBING':
            # Wait for target altitude
            if z >= (self.takeoff_altitude - 0.2):
                self.get_logger().info(f"Reached takeoff altitude. Starting hover for {self.hover_duration} seconds.")
                self.hover_start_time = time.time()
                self.state = 'HOVERING'
            else:
                # Do not publish velocities during climb to avoid overriding guided takeoff
                pass

        elif self.state == 'HOVERING':
            # Hold for hover_duration to stabilize/hover
            self.publish_velocity(0.0, 0.0, 0.0)
            elapsed_time = time.time() - self.hover_start_time
            self.get_logger().info(f"Hovering... {elapsed_time:.1f}s / {self.hover_duration}s", throttle_duration_sec=1.0)

            if elapsed_time >= self.hover_duration:
                self.get_logger().info("Hover complete. Requesting LAND mode...")
                self.state = 'LANDING'
                self.request_mode("LAND")

        elif self.state == 'LANDING':
            # Check if landed (disarmed or altitude near 0)
            if not self.current_state.armed or z <= 0.15:
                self.get_logger().info("Drone landed successfully. Shutting down node.")
                self.state = 'FINISHED'
            else:
                # If mode switched away from LAND somehow, request LAND again
                if self.current_state.mode != "LAND":
                    self.request_mode("LAND")

        elif self.state == 'FINISHED':
            self.timer.cancel()
            rclpy.shutdown()

    def publish_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def request_arm(self, value):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            return
        req = CommandBool.Request()
        req.value = value
        self.arm_client.call_async(req)

    def request_mode(self, custom_mode):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetMode.Request()
        req.custom_mode = str(custom_mode)
        self.mode_client.call_async(req)

    def request_takeoff(self, alt):
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            return
        req = CommandTOL.Request()
        req.altitude = float(alt)
        self.takeoff_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = HoverScript()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
