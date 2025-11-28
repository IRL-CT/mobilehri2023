import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node

from dance_interfaces.action import Dance


class DanceActionClient(Node):

    def __init__(self):
        super().__init__('dance_action_client')
        self._action_client = ActionClient(self, Dance, 'dance')

    def send_goal(self, dance_move):
        goal_msg = Dance.Goal()
        goal_msg.dance_move = dance_move

        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal: {dance_move}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        return send_goal_future
    
    def send_goal_and_wait(self, dance_move):
        """Send a goal and wait for completion with proper result handling"""
        goal_msg = Dance.Goal()
        goal_msg.dance_move = dance_move

        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal: {dance_move}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return False
            
        self.get_logger().info('Goal accepted')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        self.get_logger().info(f'Result: {result.result_code}')
        return True


def main(args=None):
    rclpy.init(args=args)

    action_client = DanceActionClient()
    
    # Greeting (6 seconds)
    action_client.send_goal_and_wait("Greeting")

    # Tap 4 times (4 seconds)
    for _ in range(4):
        action_client.send_goal_and_wait("TapOnRight")
    
    # Slalom backward (4 seconds)
    action_client.send_goal_and_wait("SlalomBackward")
    
    # Pirouette left (3 seconds)
    action_client.send_goal_and_wait("PirouetteLeft")

    # Slalom forward (4 seconds)
    action_client.send_goal_and_wait("SlalomForward")

    # Pirouette right (3 seconds)
    action_client.send_goal_and_wait("PirouetteRight")

    # Slalom backward (4 seconds)
    action_client.send_goal_and_wait("ZigZaggingBackward")

    # Teacup spin right (3 seconds)
    action_client.send_goal_and_wait("TeacupSpinRight")

    # Tap 4 times (4 seconds)
    for _ in range(4):
        action_client.send_goal_and_wait("TapOnRight")

    # Teacup spin left (3 seconds)
    action_client.send_goal_and_wait("TeacupSpinLeft")

    # Tap 4 times (4 seconds)
    for _ in range(4):
        action_client.send_goal_and_wait("TapOnLeft")
    
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()