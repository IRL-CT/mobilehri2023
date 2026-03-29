"""
Platform-Agnostic Dance Action Server

Dispatches dance move requests to the loaded RobotPlatform implementation.
The server no longer knows about Twist, differential drive, or any specific
robot type — it delegates everything to the platform.

Platform selection is controlled by the 'platform' ROS parameter:
    ros2 run dance_manager dance_server --ros-args -p platform:=differential_drive

Architecture
------------
    DanceActionClient / Choreographer
            │  (Dance.action goal: move_name + energy + texture)
            ▼
    DanceActionServer  ← this file (platform-agnostic dispatcher)
            │  (platform.execute_move(name, context))
            ▼
    RobotPlatform impl  (e.g. DiffDrivePlatform — publishes Twist)
"""

import rclpy
import time
from rclpy.action import ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from dance_interfaces.action import Dance
from dance_manager.platform import MoveContext, Texture

# Global cancel flag that dance_moves can check
cancel_requested = False


# ── Platform registry ────────────────────────────────────────────────────────

def _load_platform(platform_name: str, twist_pub):
    """Load a RobotPlatform implementation by name.

    Args:
        platform_name: Platform identifier (e.g. 'differential_drive').
        twist_pub: ROS 2 Twist publisher (passed to platforms that need it).

    Returns:
        A RobotPlatform instance.
    """
    if platform_name == "differential_drive":
        from dance_manager.platforms.differential_drive import DiffDrivePlatform
        return DiffDrivePlatform(twist_pub)
    else:
        raise ValueError(
            f"Unknown platform '{platform_name}'. "
            f"Available: differential_drive"
        )


class DanceActionServer(Node):

    def __init__(self):
        super().__init__('dance_action_server')

        # Declare platform parameter with default
        self.declare_parameter('platform', 'differential_drive')
        platform_name = self.get_parameter('platform').get_parameter_value().string_value

        # Create Twist publisher (used by diff-drive and similar platforms)
        self.twist_pub = self.create_publisher(Twist, '/dance_manager/cmd_vel', 10)

        # Load the platform
        self._platform = _load_platform(platform_name, self.twist_pub)
        self.get_logger().info(
            f'Loaded platform: {platform_name} '
            f'({len(self._platform.get_available_moves())} moves available)'
        )

        # Stage tracking (optional)
        self.declare_parameter('stage_width', 0.0)
        self.declare_parameter('stage_depth', 0.0)
        stage_width = self.get_parameter('stage_width').get_parameter_value().double_value
        stage_depth = self.get_parameter('stage_depth').get_parameter_value().double_value

        self._stage_tracker = None
        if stage_width > 0 or stage_depth > 0:
            from dance_manager.stage_tracker import StageTracker
            self._stage_tracker = StageTracker(
                self, stage_width=stage_width, stage_depth=stage_depth)
            self.get_logger().info(
                f'Stage tracker active: {stage_width}m x {stage_depth}m')

        self._action_server = ActionServer(
            self,
            Dance,
            'dance',
            self.execute_callback)
        self.cancel_sub = self.create_subscription(Bool, '/dance_cancel', self.cancel_callback, 10)

        # Flag to track if an action is currently executing
        self.action_active = False

        # Timer for default motion (runs every 0.1 seconds)
        self.default_timer = self.create_timer(0.1, self.default_motion_callback)

        # For sophisticated default motion
        self.start_time = time.time()

    def cancel_callback(self, msg):
        """Handle cancel requests from joystick."""
        global cancel_requested
        if msg.data:
            self.get_logger().info('Cancel requested!')
            cancel_requested = True
            self._platform.stop()

    def execute_callback(self, goal_handle):
        global cancel_requested

        # Check if cancel was requested BEFORE starting this move
        if cancel_requested:
            self.get_logger().info(f'Skipping goal {goal_handle.request.dance_move} - cancel was requested')
            goal_handle.succeed()
            result = Dance.Result()
            result.result_code = 0  # Cancelled
            return result

        requested_move = goal_handle.request.dance_move
        self.get_logger().info(f'Executing goal: {requested_move}')

        # Disable default motion while executing action
        self.action_active = True

        # Parse move-specific params (JSON string → dict)
        params = {}
        if goal_handle.request.params:
            try:
                import json
                params = json.loads(goal_handle.request.params)
            except (json.JSONDecodeError, TypeError):
                self.get_logger().warn(
                    f'Invalid params JSON: {goal_handle.request.params!r}, using defaults')

        # Build MoveContext from action goal fields
        context = MoveContext(
            energy=goal_handle.request.energy,
            texture=self._parse_texture(goal_handle.request.texture),
            params=params,
        )

        # Dispatch to platform
        if requested_move in self._platform.get_available_moves():
            self._platform.execute_move(requested_move, context)
            if cancel_requested:
                self.get_logger().info(f'Dance move {requested_move} was cancelled')
            else:
                self.get_logger().info(f'Executed dance move: {requested_move}')
                if self._stage_tracker is not None:
                    self._stage_tracker.log_position(label=requested_move)
        else:
            self.get_logger().warn(
                f'Unknown dance move: {requested_move}. '
                f'Available: {self._platform.get_available_moves()}'
            )

        # Ensure robot is stopped after move completes
        self._platform.stop()

        goal_handle.succeed()

        # Re-enable default motion after action completes
        self.action_active = False
        cancel_requested = False

        result = Dance.Result()
        result.result_code = 0 if cancel_requested else 1
        return result

    def default_motion_callback(self):
        """Execute default motion when no action is active."""
        if not self.action_active:
            twist = Twist()
            self.twist_pub.publish(twist)

    @staticmethod
    def _parse_texture(texture_str: str) -> Texture:
        """Parse a texture string from the action goal into a Texture enum."""
        try:
            return Texture(texture_str.lower())
        except ValueError:
            return Texture.NEUTRAL


def main(args=None):
    rclpy.init(args=args)
    dance_action_server = DanceActionServer()
    rclpy.spin(dance_action_server)


if __name__ == '__main__':
    main()
