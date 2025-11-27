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
import time, math
from geometry_msgs.msg import Twist

def greeting(
    twist_pub,
    turn_duration=1.0,
    pause_duration=1.0,
    cmd_dt=0.05
):
    """Look left, then right, then center (greeting gesture).

    Sequence:
    1. Turn Left ~45 deg
    2. Pause
    3. Turn Right ~90 deg (to 45 deg Right)
    4. Pause
    5. Turn Left ~45 deg (to Center)

    Args:
        twist_pub: ROS 2 publisher.
        turn_duration (float): Time to complete a 45 degree turn.
        pause_duration (float): Time to wait between turns.
        cmd_dt (float): Command period.
    """
    t = Twist()
    
    # 45 degrees = pi/4 radians
    target_angle = math.pi / 4.0
    w = target_angle / turn_duration  # rad/s

    def perform_turn(speed, duration):
        end = time.time() + duration
        while time.time() < end:
            t.angular.z = speed
            twist_pub.publish(t)
            time.sleep(cmd_dt)
        
        # Stop rotation
        t.angular.z = 0.0
        twist_pub.publish(t)

        # Rotational brake (similar to abs_brake but for angular.z)
        brake_sign = -1.0 if speed > 0 else 1.0
        for _ in range(5):
            t.angular.z = brake_sign * 0.2
            twist_pub.publish(t)
            time.sleep(0.05)
        
        t.angular.z = 0.0
        twist_pub.publish(t)

    # 1. Turn Left 45 deg
    perform_turn(w, turn_duration)
    time.sleep(pause_duration)

    # 2. Turn Right 90 deg (2 * 45 deg)
    # We use same angular speed, so double the duration
    perform_turn(-w, 2.0 * turn_duration)
    time.sleep(pause_duration)

    # 3. Recenter (Turn Left 45 deg)
    perform_turn(w, turn_duration)
    time.sleep(pause_duration)

def abs_brake(twist_pub, direction, brake_times=5, pause_duration=0.05):
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
    # ramp up over 0.5s
    while time.time() <= start + 0.5:
        t.linear.x = 2.0 * (time.time() - start)  # scale to reach 1.0 in 0.5s
        twist_pub.publish(t)
        time.sleep(0.1)
    start = time.time()
    # ramp down over 0.25s
    while time.time() <= start + 0.25:
        t.linear.x = 1 - 4.0 * (time.time() - start)  # scale to go from 1.0 to 0.0 in 0.25s
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
    while time.time() <= start + 0.5:
        t.linear.x = -2.0 * (time.time() - start)
        twist_pub.publish(t)
        time.sleep(0.1)
    
    start = time.time()
    while time.time() <= start + 0.25:
        t.linear.x = -1 * (1 - 4.0 * (time.time() - start))
        twist_pub.publish(t)
        time.sleep(0.05)

    abs_brake(twist_pub, direction=1)

def tap_on_side(
    twist_pub,
    side="left",          # "left" or "right"
    tap_times=1,
    tap_duration=0.25,     # seconds per stroke (forward or backward)
    v_mag=0.40,           # m/s linear speed of the moving wheel
    track=0.6,           # meters; distance between wheels
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


def zigzag(
    twist_pub, 
    direction="forward",   # "forward" or "backward"
    turns=4,
    tap_duration=0.5,     # seconds per stroke (forward or backward)
    v_mag=0.40,           # m/s linear speed of the moving wheel
    track=0.6,           # meters; distance between wheels
    cmd_dt=0.05           # command period
):
    """Advance forward or backward while alternating left/right arcs (zig-zag pattern).

    The robot performs alternating arcs by commanding differential velocities
    that approximate pivoting about one wheel then the other, creating a
    zig-zag trajectory. Each arc lasts ``tap_duration`` seconds.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        direction (str): "forward" or "backward" - direction of zigzag motion.
        turns (int): number of left/right arc pairs to perform.
        tap_duration (float): seconds for each arc.
        v_mag (float): base magnitude used to compute v and w.
        track (float): track width [m].
        cmd_dt (float): command period [s].

    Returns:
        None
    """
    t = Twist()
    
    # Determine direction multiplier
    if direction.lower() == "forward":
        dir_multiplier = 1
        brake_direction = -1
    elif direction.lower() == "backward":
        dir_multiplier = -1
        brake_direction = 1
    else:
        raise ValueError("direction must be 'forward' or 'backward'")

    def left_arc(duration):
        v = 0.5 * v_mag * dir_multiplier
        w = -v_mag / track
        
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = v
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    def right_arc(duration):
        v = 0.5 * v_mag * dir_multiplier
        w = v_mag / track
        
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = v
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    # Perform zigzag motions
    # Sequence: Half-Left -> Full-Right -> [Full-Left -> Full-Right]* -> Half-Left
    if turns > 0:
        # Initial half arc to start from center
        left_arc(tap_duration / 2.0)
        abs_brake(twist_pub, direction=brake_direction)

        # First full opposite arc
        right_arc(tap_duration)
        abs_brake(twist_pub, direction=brake_direction)

        # Remaining full cycles
        for _ in range(turns - 1):
            left_arc(tap_duration)
            abs_brake(twist_pub, direction=brake_direction)

            right_arc(tap_duration)
            abs_brake(twist_pub, direction=brake_direction)

        # Final half arc to return to center
        left_arc(tap_duration / 2.0)
        abs_brake(twist_pub, direction=brake_direction)

def pirouette(twist_pub, side="left", spin_duration=3.0, track=0.60, cmd_dt=0.05):
    """Pivot the robot around one wheel (left or right) for a full 360° turn.

    This commands a center linear velocity and an angular velocity such that the
    robot's center describes a circle around the stationary wheel. The motion
    completes one full rotation in ``spin_duration`` seconds.

    Args:
        twist_pub: ROS 2 publisher for geometry_msgs.msg.Twist
        side (str): "left" or "right" — which wheel remains (approximately) fixed
        spin_duration (float): seconds to complete 360 degrees
        track (float): distance between wheels [m]
        cmd_dt (float): command period [s]
    """
    t = Twist()

    if side.lower() == "left":
        w_sign = 1.0
    elif side.lower() == "right":
        w_sign = -1.0
    else:
        raise ValueError("side must be 'left' or 'right'")

    # angular speed to complete 2*pi radians in spin_duration
    w_target = 2.0 * math.pi / float(spin_duration)

    # linear speed of the robot center so it pivots about the wheel at radius = track/2
    v_center = w_target * (track / 2.0)

    end = time.time() + float(spin_duration)
    while time.time() < end:
        t.linear.x = v_center
        t.angular.z = w_sign * w_target
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    # small brake pulses to settle linear motion
    abs_brake(twist_pub, direction=-1)


def slalom(
    twist_pub,
    direction="forward",
    duration=4.0,
    linear_speed=0.5,
    oscillation_amp=1.2,
    frequency=0.5,
    cmd_dt=0.05
):
    """Move continuously while weaving side-to-side in a sine-wave pattern.

    Adjusts the actual duration to be an integer multiple of the oscillation period
    so that the net rotation is zero (robot ends facing the same way it started).

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        direction (str): "forward" or "backward".
        duration (float): Approximate total duration [s].
        linear_speed (float): Constant linear speed [m/s].
        oscillation_amp (float): Maximum angular velocity [rad/s].
        frequency (float): How fast to weave left/right [Hz].
        cmd_dt (float): Command period [s].
    """
    t = Twist()
    
    if direction.lower() == "forward":
        lin_sign = 1.0
    elif direction.lower() == "backward":
        lin_sign = -1.0
    else:
        raise ValueError("direction must be 'forward' or 'backward'")

    # Enforce full cycles to ensure zero net rotation
    # Period T = 1/f. We want duration = k * T where k is an integer.
    period = 1.0 / frequency
    num_cycles = max(1, round(duration / period))
    actual_duration = num_cycles * period
    
    start_time = time.time()
    end_time = start_time + actual_duration
    
    while time.time() < end_time:
        elapsed = time.time() - start_time
        
        # Constant linear motion
        t.linear.x = linear_speed * lin_sign
        
        # Sinusoidal angular motion: w = A * cos(2*pi*f*t)
        # Integral of cos(t) over full periods is 0 -> 0 net rotation
        # Using cos instead of sin makes the heading oscillate symmetrically around 0
        t.angular.z = oscillation_amp * math.cos(2 * math.pi * frequency * elapsed)
        
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    # Stop cleanly
    t.linear.x = 0.0
    t.angular.z = 0.0
    twist_pub.publish(t)
    
    # Quick brake to kill momentum
    abs_brake(twist_pub, direction=-lin_sign)

# Simple roll
# Pivoting 
# Sweeping with pause (left and right) the looking
# Slalom (front and back)
# Spinning in place 
# Spinning in one foot (front and back)
# Free form skating around
# Pacing or patrolling