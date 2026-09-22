import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
import time

class ArmDisarmNode(Node):
    def __init__(self):
        super().__init__('arm_disarm_script')
        
        self.get_logger().info('Initializing Arm/Disarm Node...')
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for the services to be available
        while not self.arming_client.wait_for_service(timeout_sec=1.0) or not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MAVROS services...')
            
        self.get_logger().info('Services are available! Proceeding...')
        self.execute_sequence()

    def execute_sequence(self):
        # 1. Set mode to GUIDED
        self.get_logger().info('Setting mode to GUIDED...')
        self.set_mode('GUIDED')
        time.sleep(1.0)
        
        # 2. Arm the drone
        self.get_logger().info('ARMING the drone...')
        self.arm_drone(True)
        
        # 3. Wait 5 seconds
        self.get_logger().info('Waiting 5 seconds...')
        time.sleep(5.0)
        
        # 4. Disarm the drone
        self.get_logger().info('DISARMING the drone...')
        self.arm_drone(False)
        
        self.get_logger().info('Sequence complete. Shutting down.')

    def set_mode(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info(f'Successfully set mode to {custom_mode}')
        else:
            self.get_logger().error(f'Failed to set mode to {custom_mode}')

    def arm_drone(self, arm_state):
        req = CommandBool.Request()
        req.value = arm_state
        
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res.success:
            state_str = "ARMED" if arm_state else "DISARMED"
            self.get_logger().info(f'Successfully {state_str} the drone (Result code: {res.result}).')
        else:
            self.get_logger().error(f'Failed to change arming state! (Result code: {res.result}). CHECK TERMINAL 1 FOR PRE-ARM ERRORS!')

def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarmNode()
    
    # We don't need to spin continuously since we execute the sequence in __init__
    # and then we just shut down.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
