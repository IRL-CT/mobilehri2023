import rclpy
import time
import math
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from dance_interfaces.action import Dance
from dance_manager.dance_moves import *

# Global cancel flag that dance_moves can check
cancel_requested = False


class DanceActionServer(Node):

    def __init__(self):
        super().__init__('dance_action_server')
        self._action_server = ActionServer(
            self,
            Dance,
            'dance',
            self.execute_callback)
        self.twist_pub = self.create_publisher(Twist, '/dance_manager/cmd_vel', 10)
        self.cancel_sub = self.create_subscription(Bool, '/dance_cancel', self.cancel_callback, 10)
        
        # Flag to track if an action is currently executing
        self.action_active = False

        # Timer for default motion (runs every 0.05 seconds)
        self.default_timer = self.create_timer(0.1, self.default_motion_callback)
        
        # For sophisticated default motion
        self.start_time = time.time()

    def cancel_callback(self, msg):
        """Handle cancel requests from joystick."""
        global cancel_requested
        if msg.data:
            self.get_logger().info('Cancel requested!')
            cancel_requested = True
            # Immediately stop the robot
            stop_twist = Twist()
            self.twist_pub.publish(stop_twist)

    def execute_callback(self, goal_handle):
        global cancel_requested
        
        # Check if cancel was requested BEFORE starting this move
        # This skips queued goals when cancel was pressed
        if cancel_requested:
            self.get_logger().info(f'Skipping goal {goal_handle.request.dance_move} - cancel was requested')
            goal_handle.succeed()
            result = Dance.Result()
            result.result_code = 0  # Cancelled
            return result
        
        self.get_logger().info(f'Executing goal: {goal_handle.request.dance_move}')
        
        # Disable default motion while executing action
        self.action_active = True

        # Dictionary-based switch for dance moves
        dance_moves = {
            # ── Social gestures ──────────────────────────────────────────────
            "Greeting":             lambda: glance(self.twist_pub, turn_duration=1.0, pause_duration=1.0),
            "PeekLeftRight":        lambda: glance(self.twist_pub, turn_duration=0.6, pause_duration=1.0),
            "Bow":                  lambda: bow_sequence(self.twist_pub),
            # ── Linear steps ─────────────────────────────────────────────────
            "InchForward":          lambda: step(self.twist_pub, direction="forward",  ramp_up_duration=0.2, ramp_down_duration=0.2),
            "StepForward":          lambda: step(self.twist_pub, direction="forward",  ramp_up_duration=0.7, ramp_down_duration=0.3, profile="exponential"),
            "RollForward":          lambda: step(self.twist_pub, direction="forward",  ramp_up_duration=1.5, ramp_down_duration=0.8),
            "InchBackward":         lambda: step(self.twist_pub, direction="backward", ramp_up_duration=0.2, ramp_down_duration=0.2),
            "StepBackward":         lambda: step(self.twist_pub, direction="backward", ramp_up_duration=0.7, ramp_down_duration=0.3, profile="exponential"),
            "GlideForward":         lambda: glide(self.twist_pub, direction="forward",  duration=2.0, speed=0.3),
            "GlideBackward":        lambda: glide(self.twist_pub, direction="backward", duration=2.0, speed=0.3),
            # ── Expressive in-place ───────────────────────────────────────────
            "Shimmy":               lambda: shimmy(self.twist_pub, duration=3.0, frequency=3.0),
            "ShimmyFast":           lambda: shimmy(self.twist_pub, duration=2.0, frequency=5.0, magnitude=1.2),
            "Pulse":                lambda: pulse(self.twist_pub, n=4),
            "Vibrate":              lambda: vibrate(self.twist_pub, duration=2.0),
            # ── Pivots and taps ───────────────────────────────────────────────
            "TapOnLeft":            lambda: tap_on_side(self.twist_pub, side="left"),
            "TapOnRight":           lambda: tap_on_side(self.twist_pub, side="right"),
            "PirouetteLeft":        lambda: pirouette(self.twist_pub, side="left"),
            "PirouetteRight":       lambda: pirouette(self.twist_pub, side="right"),
            # ── Axis spins ────────────────────────────────────────────────────
            "SpinClockwise":        lambda: spin_on_axis(self.twist_pub, clockwise=True),
            "SpinCounterClockwise": lambda: spin_on_axis(self.twist_pub, clockwise=False),
            "Spin180CW":            lambda: spin_on_axis(self.twist_pub, clockwise=True,  rotations=0.5,  spin_duration=2.5),
            "Spin180CCW":           lambda: spin_on_axis(self.twist_pub, clockwise=False, rotations=0.5,  spin_duration=2.5),
            "Spin90CW":             lambda: spin_on_axis(self.twist_pub, clockwise=True,  rotations=0.25, spin_duration=2.0),
            "Spin90CCW":            lambda: spin_on_axis(self.twist_pub, clockwise=False, rotations=0.25, spin_duration=2.0),
            "Spin15CW":             lambda: spin_on_axis(self.twist_pub, clockwise=True,  rotations=0.04, spin_duration=1.0),
            "Spin15CCW":            lambda: spin_on_axis(self.twist_pub, clockwise=False, rotations=0.04, spin_duration=1.0),
            # ── Weaving paths ─────────────────────────────────────────────────
            "ZigZaggingForward":    lambda: zigzag(self.twist_pub, direction="forward"),
            "ZigZaggingBackward":   lambda: zigzag(self.twist_pub, direction="backward"),
            "SlalomForward":        lambda: slalom(self.twist_pub, direction="forward"),
            "SlalomBackward":       lambda: slalom(self.twist_pub, direction="backward"),
            "WagWalk":              lambda: wag_walking(self.twist_pub),
            # ── Arc / circle patterns ─────────────────────────────────────────
            "ArcLeft":              lambda: drive_arc(self.twist_pub, radius=0.5, angle=math.pi, direction="left"),
            "ArcRight":             lambda: drive_arc(self.twist_pub, radius=0.5, angle=math.pi, direction="right"),
            "TeacupSpinLeft":       lambda: teacup_spin(self.twist_pub, side="left"),
            "TeacupSpinRight":      lambda: teacup_spin(self.twist_pub, side="right"),
            "TeacupCircleLeft":     lambda: teacup(self.twist_pub, direction="left"),
            "TeacupCircleRight":    lambda: teacup(self.twist_pub, direction="right"),
            # ── Complex paths ─────────────────────────────────────────────────
            "SpiralLeft":           lambda: spiral(self.twist_pub, direction="left"),
            "SpiralRight":          lambda: spiral(self.twist_pub, direction="right"),
            "FigureEight":          lambda: figure_eight(self.twist_pub),
            "FlowerDance":          lambda: flower(self.twist_pub),
        }
        
        # Execute the requested dance move
        requested_move = goal_handle.request.dance_move
        if requested_move in dance_moves:
            dance_moves[requested_move]()
            if cancel_requested:
                self.get_logger().info(f'Dance move {requested_move} was cancelled')
            else:
                self.get_logger().info(f'Executed dance move: {requested_move}')
        else:
            self.get_logger().warn(f'Unknown dance move: {requested_move}')
        
        # Stop the robot after dance completes or is cancelled
        stop_twist = Twist()
        self.twist_pub.publish(stop_twist)
        
        goal_handle.succeed()
        
        # Re-enable default motion after action completes
        self.action_active = False
        cancel_requested = False
        
        result = Dance.Result()
        result.result_code = 0 if cancel_requested else 1
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