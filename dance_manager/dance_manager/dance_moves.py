"""
Dance movement primitives used by the dance_manager server.

This module provides small, composable motion routines that publish geometry_msgs/Twist
commands via a provided ROS 2 publisher (twist_pub). Each function takes a publisher
and issues velocity commands for a short, bounded period to realize a specific move.

Conventions
- Units: meters (m), seconds (s), radians (rad).
- Coordinate frame: linear.x forward (+), angular.z counter-clockwise (+).
- Publisher: twist_pub is expected to be a rclpy.Publisher[geometry_msgs.msg.Twist]
  typically targeting the robot's velocity command topic (e.g., "/cmd_vel").

Safety
- These routines issue open-loop velocity commands. Ensure you run them in a safe area
  and that your robot stack enforces limits. Consider adding collision monitors.

Extending
- Add new primitives here following the same pattern: keep them short, parameterized,
  and side-effect free except for publishing Twist. For higher-level choreography,
  compose these primitives in the server logic.
"""
import time
from geometry_msgs.msg import Twist

def abs_brake(twist_pub, direction, brake_times=10, pause_duration=0.05):
    """Pulse a small opposite linear command to quickly damp motion.

    This mimics an anti-lock braking (ABS) effect by publishing a small linear.x
    command repeatedly. A positive ``direction`` (1) sends a small forward pulse;
    negative ``direction`` (-1) sends a small backward pulse.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        direction (int): 1 for forward pulses, -1 for backward pulses.
        brake_times (int): number of pulses to send.
        pause_duration (float): seconds to wait between pulses.

    Returns:
        None
    """
    t = Twist()
    for i in range(brake_times):
        t.linear.x = direction * 0.1
        twist_pub.publish(t)
        time.sleep(pause_duration)


def inch_forward(twist_pub):
    """Short forward "inch" motion: accelerate briefly, then decelerate.

    Generates a small forward movement by linearly ramping up ``linear.x`` over
    ~1s, then ramping down over ~0.5s. Concludes with a brief braking pulse to
    counter residual motion.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.

    Returns:
        None
    """
    t = Twist()
    start = time.time()
    while time.time() <= start + 1:
        t.linear.x = time.time() - start
        twist_pub.publish(t)
        time.sleep(0.1)
    start = time.time()
    while time.time() <= start + 0.5:
        t.linear.x = 1 - (time.time() - start) * 2
        twist_pub.publish(t)
        time.sleep(0.05)

    abs_brake(twist_pub, direction=-1)

def inch_backward(twist_pub):
    """Short backward "inch" motion: accelerate briefly, then decelerate.

    Like :func:`inch_forward` but mirrored in the negative x direction. Ends with
    a small forward braking pulse to settle.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.

    Returns:
        None
    """
    t = Twist()
    start = time.time()
    while time.time() <= start + 1:
        t.linear.x = -(time.time() - start)
        twist_pub.publish(t)
        time.sleep(0.1)
    
    start = time.time()
    while time.time() <= start + 0.5:
        t.linear.x = -1 * (1 - (time.time() - start) * 2)
        twist_pub.publish(t)
        time.sleep(0.05)

    abs_brake(twist_pub, direction=1)

def tap_on_side(
    twist_pub,
    side="left",          # "left" or "right"
    tap_times=1,
    tap_duration=0.5,     # seconds per stroke (forward or backward)
    v_mag=0.40,           # m/s linear speed of the moving wheel
    track=0.51,           # meters; distance between wheels
    cmd_dt=0.05           # command period
):
    """Tap on one wheel while the other stays fixed, producing a pivot motion.

    For a differential drive with track width ``track``, we approximate the base
    linear and angular velocities from the active wheel's linear speed (``v_mag``):

    - side == "left": pivots about the right wheel (v_R = 0)
        v =  v_L / 2,  w = -v_L / track
    - side == "right": pivots about the left wheel (v_L = 0)
        v =  v_R / 2,  w =  v_R / track

    The routine performs ``tap_times`` strokes forward and backward, with small
    braking pulses between strokes to settle motion.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        side (str): "left" or "right" — which wheel performs the tap.
        tap_times (int): number of forward/backward tap pairs.
        tap_duration (float): seconds for each stroke (forward or backward).
        v_mag (float): linear speed [m/s] applied to the active wheel.
        track (float): track width [m] between wheels.
        cmd_dt (float): command period [s].

    Returns:
        None
    """
    t = Twist()

    def send_for(duration, v_side):
        # Determine v, w based on which wheel is active
        if side.lower() == "left":
            v = 0.5 * v_side
            w = -v_side / track
        elif side.lower() == "right":
            v = 0.5 * v_side
            w =  v_side / track
        else:
            raise ValueError("side must be 'left' or 'right'")
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = v
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    # Perform tap motions
    for _ in range(tap_times):
        # forward stroke
        send_for(tap_duration, v_mag)
        abs_brake(twist_pub, direction=-1)

        # backward stroke
        send_for(tap_duration, -v_mag)
        abs_brake(twist_pub, direction=1)


def zigzagging_forward(
    twist_pub, 
    turns=1,
    tap_duration=0.5,     # seconds per stroke (forward or backward)
    v_mag=0.40,           # m/s linear speed of the moving wheel
    track=0.51,           # meters; distance between wheels
    cmd_dt=0.05           # command period
):
    """Advance forward while alternating left/right arcs (zig-zag pattern).

    The robot performs alternating arcs by commanding differential velocities
    that approximate pivoting about one wheel then the other, creating a
    zig-zag trajectory. Each arc lasts ``tap_duration`` seconds.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        turns (int): number of left/right arc pairs to perform.
        tap_duration (float): seconds for each arc.
        v_mag (float): base magnitude used to compute v and w.
        track (float): track width [m].
        cmd_dt (float): command period [s].

    Returns:
        None
    """
    t = Twist()

    def left_feet_forward(duration):
        v = 0.5 * v_mag
        w = -v_mag / track
        
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = v
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    def right_feet_forward(duration):
        v = 0.5 * v_mag
        w =  v_mag / track
        
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = v
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    # Perform tap motions
    for _ in range(turns):
        # forward stroke
        left_feet_forward(tap_duration)
        abs_brake(twist_pub, direction=-1)

        # backward stroke
        right_feet_forward(tap_duration)
        abs_brake(twist_pub, direction=-1)

# Simple roll
# Pivoting 
# zigzaggingforward and backwards
# Sweeping with pause (left and right) the looking
# Slalom (front and back)
# Spinning in place 
# Spinning in one foot (front and back)
# Free form skating around
# Pacing or patrolling