import rclpy
import time
import math
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Twist
from dance_interfaces.action import Dance
from dance_manager.dance_moves import *


class DanceActionServer(Node):

    def __init__(self):
        super().__init__('dance_action_server')
        self._action_server = ActionServer(
            self,
            Dance,
            'dance',
            self.execute_callback)
        self.twist_pub = self.create_publisher(Twist, '/dance_manager/cmd_vel', 10)
        
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

        # Dictionary-based switch for dance moves
        dance_moves = {
            "InchForward": lambda: inch_forward(self.twist_pub),
            "InchBackward": lambda: inch_backward(self.twist_pub),
            "TapOnLeft": lambda: tap_on_side(self.twist_pub, side="left"),
            "TapOnRight": lambda: tap_on_side(self.twist_pub, side="right"),
            "ZigZaggingForward": lambda: zigzag(self.twist_pub, direction="forward"),
            "ZigZaggingBackward": lambda: zigzag(self.twist_pub, direction="backward"),
            "PivotLeft": lambda: pivot(self.twist_pub, side="left"),
            "PivotRight": lambda: pivot(self.twist_pub, side="right"),
            "SlalomForward": lambda: slalom(self.twist_pub, direction="forward"),
            "SlalomBackward": lambda: slalom(self.twist_pub, direction="backward")
        }
        
        # Execute the requested dance move
        requested_move = goal_handle.request.dance_move
        if requested_move in dance_moves:
            dance_moves[requested_move]()
            self.get_logger().info(f'Executed dance move: {requested_move}')
        else:
            self.get_logger().warn(f'Unknown dance move: {requested_move}')
        
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