import time
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from dance_interfaces.action import Dance


class TeleopTwistJoy(Node):
    def __init__(self):
        super().__init__('Teleop_Keymapping_node')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joyCallback, 10)
        self.twist_pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self._action_client = ActionClient(self, Dance, 'dance')
        self.prev_button0 = 0
        self.prev_button1 = 0
        self.prev_button2 = 0
        self.prev_button3 = 0
        self.prev_button5 = 0
        self.max_linear_speed = 4
        self.max_angular_speed = 10
        self.dance_subprocess = None
        self._current_goal_handle = None


    def joyCallback(self, msg):
        self.sendCommand(msg)

    def sendCommand(self, msg):
        # Check for button press (e.g. button 0) to trigger dance
        if msg.buttons[0] == 1 and self.prev_button0 == 0:
            self.send_dance_goal("Bow")
        self.prev_button0 = msg.buttons[0]

        # Check for button [O] to trigger PeekLeftRight
        if msg.buttons[1] == 1 and self.prev_button1 == 0:
            self.send_dance_goal("PeekLeftRight")
        self.prev_button1 = msg.buttons[1]

        # Check for button [△] to trigger dance_action_client
        if msg.buttons[2] == 1 and self.prev_button2 == 0:
            self.get_logger().info("Starting dance_action_client...")
            if self.dance_subprocess and self.dance_subprocess.poll() is None:
                self.get_logger().warn("Dance client already running.")
            else:
                self.dance_subprocess = subprocess.Popen(["ros2", "run", "dance_manager", "dance_action_client"])
        self.prev_button2 = msg.buttons[2]

        # Check for button [□] to trigger dance_action_client
        if msg.buttons[3] == 1 and self.prev_button3 == 0:
            self.get_logger().info("Starting dance_action_client...")
            if self.dance_subprocess and self.dance_subprocess.poll() is None:
                self.get_logger().warn("Dance client already running.")
            else:
                self.dance_subprocess = subprocess.Popen(["ros2", "run", "dance_manager", "dance_client_wander"])
        self.prev_button3 = msg.buttons[3]

        # Check for button [R1] to trigger dance_action_client
        if msg.buttons[5] == 1 and self.prev_button5 == 0:
            self.get_logger().info("Starting dance_action_client...")
            if self.dance_subprocess and self.dance_subprocess.poll() is None:
                self.get_logger().warn("Dance client already running.")
            else:
                self.dance_subprocess = subprocess.Popen(["ros2", "run", "dance_manager", "dance_client_floaty"])
        self.prev_button5 = msg.buttons[5]

        t = Twist()
        # safety lock, press top left button
        if msg.buttons[6] == 1.0:
            t.linear.x = msg.axes[1]
            # use to represent angular velocity
            t.angular.z = msg.axes[3]
            self.twist_pub.publish(t)
        else:
            pass

    def send_dance_goal(self, dance_move):
        goal_msg = Dance.Goal()
        goal_msg.dance_move = dance_move
        
        if self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Sending goal: {dance_move}')
            future = self._action_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().warn('Dance action server not available')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._current_goal_handle = goal_handle

def main(args = None):
    rclpy.init(args=args)
    ttj = TeleopTwistJoy()
    rclpy.spin(ttj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
