"""
Stage Tracker — Runtime position awareness during dance execution.

Subscribes to /odom and maintains the robot's position relative to stage
center. Used by run_sequence() to log position between moves and could
support future safety interventions.

The tracker handles the offset between odometry origin (where the robot
powered on) and stage center (where we want 0,0 to be).
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

from nav_msgs.msg import Odometry


@dataclass
class StagePosition:
    """Robot position relative to stage center."""
    x: float = 0.0          # forward from stage center [m]
    y: float = 0.0          # left from stage center [m]
    theta: float = 0.0      # heading [rad], 0 = initial facing direction
    timestamp: float = 0.0  # ROS time [s]


class StageTracker:
    """Track robot position on stage using wheel odometry.

    Subscribes to /odom and transforms readings into stage-center coordinates
    using a configurable offset. The offset can be set at construction or
    reset at any time via reset_origin().

    Args:
        node: A rclpy.node.Node to create the subscription on.
        stage_width: Stage width [m]. 0 = no boundary checking.
        stage_depth: Stage depth [m]. 0 = no boundary checking.
        start_x: Robot's starting x position relative to stage center [m].
        start_y: Robot's starting y position relative to stage center [m].
        start_theta: Robot's starting heading relative to stage frame [rad].
    """

    def __init__(self, node, stage_width: float = 0.0, stage_depth: float = 0.0,
                 start_x: float = 0.0, start_y: float = 0.0, start_theta: float = 0.0):
        self._node = node
        self._stage_width = stage_width
        self._stage_depth = stage_depth

        # Offset: odom frame → stage frame
        self._offset_x = start_x
        self._offset_y = start_y
        self._offset_theta = start_theta

        # Latest odom reading
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._lock = threading.Lock()
        self._has_data = False

        self._sub = node.create_subscription(Odometry, '/odom', self._odom_callback, 10)

    def _odom_callback(self, msg: Odometry):
        """Update position from odometry message."""
        with self._lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y
            # Extract yaw from quaternion (only z and w needed for 2D)
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self._odom_theta = 2.0 * math.atan2(qz, qw)
            self._has_data = True

    @property
    def has_data(self) -> bool:
        """True once at least one odom message has been received."""
        return self._has_data

    @property
    def stage_width(self) -> float:
        return self._stage_width

    @property
    def stage_depth(self) -> float:
        return self._stage_depth

    def get_position(self) -> StagePosition:
        """Get current position relative to stage center."""
        with self._lock:
            cos_t = math.cos(self._offset_theta)
            sin_t = math.sin(self._offset_theta)
            stage_x = self._offset_x + cos_t * self._odom_x - sin_t * self._odom_y
            stage_y = self._offset_y + sin_t * self._odom_x + cos_t * self._odom_y
            stage_theta = self._offset_theta + self._odom_theta
            return StagePosition(x=stage_x, y=stage_y, theta=stage_theta)

    def is_within_bounds(self, margin: float = 0.0) -> bool:
        """Check if robot is within stage boundaries (with optional margin).

        Args:
            margin: Safety margin [m] inset from each edge.

        Returns:
            True if within bounds (or no bounds configured).
        """
        if self._stage_width <= 0 and self._stage_depth <= 0:
            return True
        pos = self.get_position()
        if self._stage_depth > 0:
            if abs(pos.x) > (self._stage_depth / 2.0 - margin):
                return False
        if self._stage_width > 0:
            if abs(pos.y) > (self._stage_width / 2.0 - margin):
                return False
        return True

    def distance_to_boundary(self) -> float:
        """Return minimum distance to any stage boundary [m].

        Returns float('inf') if no bounds are configured.
        """
        if self._stage_width <= 0 and self._stage_depth <= 0:
            return float('inf')
        pos = self.get_position()
        distances = []
        if self._stage_depth > 0:
            half_d = self._stage_depth / 2.0
            distances.extend([half_d - pos.x, half_d + pos.x])
        if self._stage_width > 0:
            half_w = self._stage_width / 2.0
            distances.extend([half_w - pos.y, half_w + pos.y])
        return min(distances) if distances else float('inf')

    def reset_origin(self):
        """Reset: treat current odom position as stage center (0, 0)."""
        with self._lock:
            self._offset_x = -self._odom_x
            self._offset_y = -self._odom_y
            self._offset_theta = -self._odom_theta

    def log_position(self, label: str = ""):
        """Log current stage position to ROS logger."""
        pos = self.get_position()
        boundary_str = ""
        if self._stage_width > 0 or self._stage_depth > 0:
            dist = self.distance_to_boundary()
            boundary_str = f"  boundary_dist={dist:.2f}m"
        prefix = f"[{label}] " if label else ""
        self._node.get_logger().info(
            f'{prefix}Stage pos: x={pos.x:.2f}m y={pos.y:.2f}m '
            f'theta={math.degrees(pos.theta):.0f}deg{boundary_str}'
        )

    def destroy(self):
        """Clean up the subscription."""
        self._node.destroy_subscription(self._sub)
