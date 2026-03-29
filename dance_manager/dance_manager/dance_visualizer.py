"""
Dance Visualizer — RViz/Gazebo marker visualization for dance choreography.

Publishes visualization_msgs/MarkerArray on /dance_viz with:
- Stage boundary rectangle
- Robot path trace (accumulated from /odom)
- Current move label (from /dance_manager/current_move)
- Grid lines on the stage floor
- Boundary distance warning (color changes near edges)

Launch with stage dimensions:
    ros2 run dance_manager dance_visualizer --ros-args \
        -p stage_width:=6.0 -p stage_depth:=4.0
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class DanceVisualizer(Node):

    def __init__(self):
        super().__init__('dance_visualizer')

        # Parameters
        self.declare_parameter('stage_width', 6.0)
        self.declare_parameter('stage_depth', 4.0)
        self.declare_parameter('trail_max_points', 2000)

        self._stage_width = self.get_parameter('stage_width').get_parameter_value().double_value
        self._stage_depth = self.get_parameter('stage_depth').get_parameter_value().double_value
        self._trail_max = self.get_parameter('trail_max_points').get_parameter_value().integer_value

        # State
        self._trail_points = []
        self._current_move = ''
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0

        # Publishers
        self._marker_pub = self.create_publisher(MarkerArray, '/dance_viz', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Timer — publish markers at 10 Hz
        self.create_timer(0.1, self._publish_markers)

        self.get_logger().info(
            f'Dance visualizer active: {self._stage_width}m x {self._stage_depth}m stage')

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._robot_theta = 2.0 * math.atan2(qz, qw)

        # Append to trail (thin if too many points)
        pt = Point(x=self._robot_x, y=self._robot_y, z=0.02)
        if len(self._trail_points) == 0 or self._dist_to_last(pt) > 0.01:
            self._trail_points.append(pt)
            if len(self._trail_points) > self._trail_max:
                self._trail_points = self._trail_points[-self._trail_max:]

    def _dist_to_last(self, pt):
        last = self._trail_points[-1]
        return math.sqrt((pt.x - last.x)**2 + (pt.y - last.y)**2)

    def _publish_markers(self):
        ma = MarkerArray()
        ma.markers.append(self._make_stage_boundary())
        ma.markers.append(self._make_grid())
        ma.markers.append(self._make_trail())
        ma.markers.append(self._make_robot_heading())
        ma.markers.append(self._make_position_text())
        self._marker_pub.publish(ma)

    # ── Stage boundary ────────────────────────────────────────────────────

    def _make_stage_boundary(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dance_stage'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03  # line width

        hw = self._stage_width / 2.0
        hd = self._stage_depth / 2.0
        z = 0.01

        corners = [
            Point(x=hd, y=hw, z=z),
            Point(x=hd, y=-hw, z=z),
            Point(x=-hd, y=-hw, z=z),
            Point(x=-hd, y=hw, z=z),
            Point(x=hd, y=hw, z=z),  # close the rectangle
        ]
        m.points = corners
        m.color = ColorRGBA(r=0.34, g=0.61, b=0.84, a=0.9)  # #569cd6
        m.lifetime.sec = 0  # persistent
        return m

    # ── Grid ──────────────────────────────────────────────────────────────

    def _make_grid(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dance_grid'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.005  # thin lines

        hw = self._stage_width / 2.0
        hd = self._stage_depth / 2.0
        z = 0.005
        step = 0.5

        # Lines parallel to X (depth direction)
        y = -hw
        while y <= hw + 0.001:
            m.points.append(Point(x=-hd, y=y, z=z))
            m.points.append(Point(x=hd, y=y, z=z))
            y += step

        # Lines parallel to Y (width direction)
        x = -hd
        while x <= hd + 0.001:
            m.points.append(Point(x=x, y=-hw, z=z))
            m.points.append(Point(x=x, y=hw, z=z))
            x += step

        m.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.4)
        return m

    # ── Path trail ────────────────────────────────────────────────────────

    def _make_trail(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dance_trail'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02

        if len(self._trail_points) < 2:
            m.points = [Point(x=0.0, y=0.0, z=0.02)] * 2
            m.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
            return m

        m.points = list(self._trail_points)

        # Per-vertex color: fade from dim to bright
        n = len(m.points)
        for i in range(n):
            t = i / max(1, n - 1)  # 0 at start, 1 at current position
            m.colors.append(ColorRGBA(
                r=0.34 * (0.2 + 0.8 * t),
                g=0.61 * (0.2 + 0.8 * t),
                b=0.84 * (0.2 + 0.8 * t),
                a=0.3 + 0.7 * t,
            ))

        return m

    # ── Robot heading arrow ───────────────────────────────────────────────

    def _make_robot_heading(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dance_heading'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # Arrow from robot position, pointing in heading direction
        length = 0.3
        m.points.append(Point(x=self._robot_x, y=self._robot_y, z=0.15))
        m.points.append(Point(
            x=self._robot_x + length * math.cos(self._robot_theta),
            y=self._robot_y + length * math.sin(self._robot_theta),
            z=0.15,
        ))
        m.scale.x = 0.04  # shaft diameter
        m.scale.y = 0.08  # head diameter
        m.scale.z = 0.06  # head length
        m.color = ColorRGBA(r=0.34, g=0.61, b=0.84, a=1.0)
        return m

    # ── Position text overlay ─────────────────────────────────────────────

    def _make_position_text(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dance_info'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = self._robot_x
        m.pose.position.y = self._robot_y
        m.pose.position.z = 0.7  # above the robot

        m.scale.z = 0.08  # text height

        # Compute boundary distance for color
        hw = self._stage_width / 2.0
        hd = self._stage_depth / 2.0
        distances = []
        if self._stage_depth > 0:
            distances.extend([hd - self._robot_x, hd + self._robot_x])
        if self._stage_width > 0:
            distances.extend([hw - self._robot_y, hw + self._robot_y])
        boundary_dist = min(distances) if distances else float('inf')

        # Color: green when safe, yellow when close, red when at edge
        if boundary_dist > 1.0:
            m.color = ColorRGBA(r=0.4, g=0.8, b=0.4, a=0.9)
        elif boundary_dist > 0.3:
            m.color = ColorRGBA(r=0.9, g=0.8, b=0.2, a=0.9)
        else:
            m.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.9)

        m.text = (
            f'x={self._robot_x:.2f}  y={self._robot_y:.2f}  '
            f'θ={math.degrees(self._robot_theta):.0f}°  '
            f'edge={boundary_dist:.1f}m'
        )
        return m


def main(args=None):
    rclpy.init(args=args)
    node = DanceVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
