import rclpy
import time
import math
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Twist
from dance_interfaces.action import Dance
from dance_manager.dance_moves import stepping_forward, stepping_backward


class DanceActionServer(Node):

    def __init__(self):
        super().__init__('dance_action_server')
        self._action_server = ActionServer(
            self,
            Dance,
            'dance',
            self.execute_callback)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Flag to track if an action is currently executing
        self.action_active = False

        # Timer for default motion (runs every 0.05 seconds)
        self.default_timer = self.create_timer(0.1, self.default_motion_callback)
        
        # For sophisticated default motion
        self.start_time = time.time()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Disable default motion while executing action
        self.action_active = True

        if goal_handle.request.dance_move == "SteppingForward":
            stepping_forward(self.twist_pub)
        elif goal_handle.request.dance_move == "SteppingBackward":
            stepping_backward(self.twist_pub)
        goal_handle.succeed()
        
        # Re-enable default motion after action completes
        self.action_active = False
        
        result = Dance.Result()
        result.result_code = 1
        return result
    
    def default_motion_callback(self):
        """Execute default motion when no action is active"""
        if not self.action_active:
            # Create a sinusoidal swaying motion
            twist = Twist()
            self.twist_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    dance_action_server = DanceActionServer()

    rclpy.spin(dance_action_server)


if __name__ == '__main__':
    main()