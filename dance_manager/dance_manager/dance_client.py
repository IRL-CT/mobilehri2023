import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node

from dance_interfaces.action import Dance


class DanceActionClient(Node):

    def __init__(self):
        super().__init__('dance_action_client')
        self._action_client = ActionClient(self, Dance, 'dance')

    def send_goal(self, dance_move, energy=0.5, texture="neutral", params=None):
        import json as _json
        goal_msg = Dance.Goal()
        goal_msg.dance_move = dance_move
        goal_msg.energy = float(energy)
        goal_msg.texture = str(texture)
        goal_msg.params = _json.dumps(params) if params else ""

        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: {dance_move} (e={energy}, t={texture})')
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        return send_goal_future

    def send_goal_and_wait(self, dance_move, energy=0.5, texture="neutral", params=None):
        """Send a goal and wait for completion with proper result handling."""
        import json as _json
        goal_msg = Dance.Goal()
        goal_msg.dance_move = dance_move
        goal_msg.energy = float(energy)
        goal_msg.texture = str(texture)
        goal_msg.params = _json.dumps(params) if params else ""

        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: {dance_move} (e={energy}, t={texture})')
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
    
    # Glance (6 seconds)
    action_client.send_goal_and_wait("Glance")

    # Tap right 4 times (4 seconds)
    for _ in range(4):
        action_client.send_goal_and_wait("Tap", params={"side": "right"})

    # Slalom backward (4 seconds)
    action_client.send_goal_and_wait("Slalom", params={"direction": "backward"})

    # Pirouette left (3 seconds)
    action_client.send_goal_and_wait("Pirouette", params={"side": "left"})

    # Slalom forward (4 seconds)
    action_client.send_goal_and_wait("Slalom", params={"direction": "forward"})

    # Pirouette right (3 seconds)
    action_client.send_goal_and_wait("Pirouette", params={"side": "right"})

    # Zigzag backward (4 seconds)
    action_client.send_goal_and_wait("Zigzag", params={"direction": "backward"})

    # Teacup spin right (3 seconds)
    action_client.send_goal_and_wait("TeacupSpin", params={"side": "right"})

    # Tap right 4 times (4 seconds)
    for _ in range(4):
        action_client.send_goal_and_wait("Tap", params={"side": "right"})

    # Teacup spin left (3 seconds)
    action_client.send_goal_and_wait("TeacupSpin", params={"side": "left"})

    # Tap left 4 times (4 seconds)
    for _ in range(4):
        action_client.send_goal_and_wait("Tap", params={"side": "left"})
    
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()