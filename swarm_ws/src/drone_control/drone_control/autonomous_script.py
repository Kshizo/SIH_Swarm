#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
import math
import time
from rclpy.qos import qos_profile_sensor_data

class AutonomousScript(Node):
    def __init__(self):
        super().__init__('autonomous_script')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/ap/pose/filtered', self.pose_callback, qos_profile_sensor_data)
        self.status_sub = self.create_subscription(Status, '/ap/status', self.status_callback, qos_profile_sensor_data)
        
        # Service Clients
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')
        
        # State variables
        self.current_pose = None
        self.current_status = None
        self.state = 'INIT'
        
        # Target parameters
        self.takeoff_altitude = 3.0
        self.target_distance = 10.0
        self.forward_speed = 1.0  # m/s
        
        # Flight tracking variables
        self.start_x = None
        self.start_y = None
        self.stop_start_time = None
        
        # Start state machine loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Autonomous Node Initialized. State: INIT")

    def pose_callback(self, msg):
        self.current_pose = msg

    def status_callback(self, msg):
        self.current_status = msg

    def control_loop(self):
        # We need telemetry to start
        if self.current_pose is None or self.current_status is None:
            self.get_logger().info("Waiting for pose and status telemetry...", throttle_duration_sec=3.0)
            return

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        self.get_logger().info(f"State: {self.state} | Pos: ({x:.2f}, {y:.2f}, {z:.2f}) | Armed: {self.current_status.armed} | Mode: {self.current_status.mode}", throttle_duration_sec=2.0)

        if self.state == 'INIT':
            # Baseline received, request GUIDED mode (mode = 4)
            self.state = 'SWITCH_TO_GUIDED'
            self.request_mode(4)

        elif self.state == 'SWITCH_TO_GUIDED':
            if self.current_status.mode == 4:
                self.get_logger().info("Successfully switched to GUIDED mode. Requesting ARM...")
                self.state = 'ARMING'
                self.request_arm(True)
            else:
                self.request_mode(4)  # Retry

        elif self.state == 'ARMING':
            if self.current_status.armed:
                self.get_logger().info("Motors ARMED. Requesting Takeoff...")
                self.state = 'TAKEOFF_REQUEST'
                self.request_takeoff(self.takeoff_altitude)
            else:
                self.request_arm(True)  # Retry

        elif self.state == 'TAKEOFF_REQUEST':
            # Takeoff command sent. Switch to climbing.
            self.state = 'CLIMBING'

        elif self.state == 'CLIMBING':
            # Wait for target altitude
            if z >= (self.takeoff_altitude - 0.2):
                self.get_logger().info(f"Reached takeoff altitude. Preparing to move forward by {self.target_distance}m.")
                self.start_x = x
                self.start_y = y
                self.state = 'FLYING_FORWARD'
            else:
                # Do not publish velocities during climb to avoid overriding guided takeoff
                pass

        elif self.state == 'FLYING_FORWARD':
            # Calculate distance travelled
            dx = x - self.start_x
            dy = y - self.start_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            self.get_logger().info(f"Distance travelled: {dist:.2f}m / {self.target_distance}m", throttle_duration_sec=1.0)
            
            if dist >= self.target_distance:
                self.get_logger().info("Target distance reached. Stopping and stabilizing...")
                self.state = 'STOPPING'
                self.stop_start_time = time.time()
                self.publish_velocity(0.0, 0.0, 0.0)
            else:
                # Move forward in body frame (base_link)
                self.publish_velocity(self.forward_speed, 0.0, 0.0)

        elif self.state == 'STOPPING':
            # Hold for 2 seconds to stabilize
            self.publish_velocity(0.0, 0.0, 0.0)
            if time.time() - self.stop_start_time >= 2.0:
                self.get_logger().info("Stabilized. Requesting LAND mode...")
                self.state = 'LANDING'
                self.request_mode(9)  # LAND = 9

        elif self.state == 'LANDING':
            # Check if landed (disarmed or altitude near 0)
            if not self.current_status.armed or z <= 0.15:
                self.get_logger().info("Drone landed successfully. Shutting down node.")
                self.state = 'FINISHED'
            else:
                # If mode switched away from LAND somehow, request LAND again
                if self.current_status.mode != 9:
                    self.request_mode(9)

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
        req = ArmMotors.Request()
        req.arm = value
        self.arm_client.call_async(req)

    def request_mode(self, mode_val):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = ModeSwitch.Request()
        req.mode = int(mode_val)
        self.mode_client.call_async(req)

    def request_takeoff(self, alt):
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            return
        req = Takeoff.Request()
        req.alt = float(alt)
        self.takeoff_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousScript()
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
