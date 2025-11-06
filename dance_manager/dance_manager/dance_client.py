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
    
    # Send goals using the improved method
    print("Sending first goal...")
    success1 = action_client.send_goal_and_wait("ZigZaggingForward")
    print(f"First goal completed: {success1}")
    
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()