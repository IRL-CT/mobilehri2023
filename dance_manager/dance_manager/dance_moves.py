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


def abs_brake_angular(twist_pub, direction, brake_times=5, pause_duration=0.05, intensity=0.2):
    """Pulse a small opposite angular command to quickly damp rotation.

    Args:
        twist_pub: ROS 2 publisher.
        direction (int): 1 for positive z pulses, -1 for negative z pulses.
        brake_times (int): number of pulses to send.
        pause_duration (float): seconds to wait between pulses.
        intensity (float): angular velocity magnitude for brake pulses [rad/s].
    """
    t = Twist()
    for i in range(brake_times):
        t.angular.z = direction * intensity
        twist_pub.publish(t)
        time.sleep(pause_duration)
    t.angular.z = 0.0
    twist_pub.publish(t)

def inch_forward(twist_pub, ramp_up_duration=0.5, ramp_down_duration=0.25):
    """Short forward "inch" motion: accelerate briefly, then decelerate.

    Generates a small forward movement by linearly ramping up ``linear.x`` over
    ``ramp_up_duration``, then ramping down over ``ramp_down_duration``. 
    Concludes with a brief braking pulse to counter residual motion.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        ramp_up_duration (float): seconds to accelerate.
        ramp_down_duration (float): seconds to decelerate.

    Returns:
        None
    """
    t = Twist()
    start = time.time()
    # ramp up
    while time.time() <= start + ramp_up_duration:
        t.linear.x = (1.0 / ramp_up_duration) * (time.time() - start)
        twist_pub.publish(t)
        time.sleep(0.1)
    start = time.time()
    # ramp down
    while time.time() <= start + ramp_down_duration:
        t.linear.x = 1.0 - (1.0 / ramp_down_duration) * (time.time() - start)
        twist_pub.publish(t)
        time.sleep(0.05)

    abs_brake(twist_pub, direction=-1)

def inch_forward_exponential(twist_pub, ramp_up_duration=0.5, ramp_down_duration=0.25, max_speed=1.0):
    """Short forward "inch" motion with exponential acceleration/deceleration.

    Generates a small forward movement using exponential curves:
    - Ramp up: starts slow, then accelerates faster (ease-in)
    - Ramp down: starts slow deceleration, then decelerates faster (ease-in)

    Uses the formula: speed = max_speed * (progress^2) for ramp up
                      speed = max_speed * (1 - progress)^2 for ramp down

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        ramp_up_duration (float): seconds to accelerate.
        ramp_down_duration (float): seconds to decelerate.
        max_speed (float): maximum linear velocity [m/s].

    Returns:
        None
    """
    t = Twist()
    start = time.time()
    
    # Ramp up: slow start, fast finish (quadratic ease-in)
    while time.time() <= start + ramp_up_duration:
        elapsed = time.time() - start
        progress = elapsed / ramp_up_duration  # 0 to 1
        # Quadratic ease-in: progress^2
        t.linear.x = max_speed * (progress ** 2)
        twist_pub.publish(t)
        time.sleep(0.05)
    
    start = time.time()
    # Ramp down: slow start, fast finish deceleration (quadratic ease-in for decel)
    while time.time() <= start + ramp_down_duration:
        elapsed = time.time() - start
        progress = elapsed / ramp_down_duration  # 0 to 1
        # Start at max_speed, end at 0, with slow-then-fast deceleration
        # remaining = (1 - progress)^2 means we stay high longer, then drop fast
        t.linear.x = max_speed * ((1 - progress) ** 2)
        twist_pub.publish(t)
        time.sleep(0.05)

    abs_brake(twist_pub, direction=-1)

def inch_backward(twist_pub, ramp_up_duration=0.5, ramp_down_duration=0.25):
    """Short backward "inch" motion: accelerate briefly, then decelerate.

    Like :func:`inch_forward` but mirrored in the negative x direction. Ends with
    a small forward braking pulse to settle.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        ramp_up_duration (float): seconds to accelerate.
        ramp_down_duration (float): seconds to decelerate.

    Returns:
        None
    """
    t = Twist()
    start = time.time()
    
    while time.time() <= start + ramp_up_duration:
        t.linear.x = -(1.0 / ramp_up_duration) * (time.time() - start)
        twist_pub.publish(t)
        time.sleep(0.1)
    
    start = time.time()
    while time.time() <= start + ramp_down_duration:
        t.linear.x = -1.0 * (1.0 - (1.0 / ramp_down_duration) * (time.time() - start))
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

def pirouette(twist_pub, side="left", spin_duration=5.0, track=0.60, cmd_dt=0.05):
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

    # Stop all motion
    t.linear.x = 0.0
    t.angular.z = 0.0
    twist_pub.publish(t)

    # small brake pulses to settle linear motion
    abs_brake(twist_pub,brake_times=10,direction=-1)


def slalom(
    twist_pub,
    direction="forward",
    duration=6.0,
    linear_speed=0.4,
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
        # Heading (integral) = A/(2*pi*f) * sin(2*pi*f*t)
        # This starts at heading=0, oscillates symmetrically, returns to 0
        t.angular.z = oscillation_amp * math.cos(2 * math.pi * frequency * elapsed)
        
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    # Stop cleanly
    t.linear.x = 0.0
    t.angular.z = 0.0
    twist_pub.publish(t)
    
    # Quick brake to kill momentum
    abs_brake(twist_pub, direction=-lin_sign)

def teacup_spin(
    twist_pub,
    side="left",
    duration=8.0,
    radius=1.0,
    cmd_dt=0.05
):
    """Spin 360 degrees while displacing to the side (Disney teacup style).

    The robot performs a full 360 rotation while varying its linear velocity.
    This creates a spiral-like motion where the robot ends up laterally displaced
    to the specified side, but facing the same direction.

    Args:
        twist_pub: ROS 2 publisher.
        side (str): "left" or "right" (direction of rotation and displacement).
        duration (float): Total time for the 360 spin.
        radius (float): Approximate lateral displacement [m]. Larger = bigger arc.
        cmd_dt (float): Command period.
    """
    t = Twist()

    # Angular velocity for 360 deg in duration
    w_mag = 2.0 * math.pi / duration
    if side.lower() == "left":
        w = w_mag
    elif side.lower() == "right":
        w = -w_mag
    else:
        raise ValueError("side must be 'left' or 'right'")

    # Linear velocity varies from +v_limit to -v_limit over the spin
    # The net lateral displacement is approximately: integral of v*sin(theta) dtheta
    # For a full 360 spin with linear ramp from +v to -v, displacement ≈ 2*v_limit/w
    # So v_limit = radius * w / 2
    v_limit = radius * w_mag / 2.0

    v_start = v_limit
    v_end = -v_limit

    start_time = time.time()
    end_time = start_time + duration

    while time.time() < end_time:
        now = time.time()
        elapsed = now - start_time
        progress = elapsed / duration
        
        # Linear ramp of linear velocity
        current_v = v_start + (v_end - v_start) * progress
        
        t.linear.x = current_v
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)
    
    abs_brake(twist_pub, direction=1)


def spin_on_axis(
    twist_pub,
    rotations=1.0,
    spin_duration=5.0,
    clockwise=False,
    cmd_dt=0.05
):
    """Spin the robot in-place about its center axis.

    This primitive commands zero linear velocity and a constant angular
    velocity so the robot rotates around its center. By default it completes
    ``rotations`` full turns in ``spin_duration`` seconds.

    Args:
        twist_pub: ROS 2 publisher for geometry_msgs.msg.Twist
        rotations (float): number of full 360° rotations to perform
        spin_duration (float): total time to perform the rotations [s]
        clockwise (bool): if True, rotate clockwise (negative angular.z)
        cmd_dt (float): command period [s]
    """
    t = Twist()

    if spin_duration <= 0 or rotations == 0:
        return

    # angular speed magnitude (rad/s) to complete `rotations` in spin_duration
    w_mag = 2.0 * math.pi * float(rotations) / float(spin_duration)
    w = -w_mag if clockwise else w_mag

    end = time.time() + float(spin_duration)
    while time.time() < end:
        t.linear.x = 0.0
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    # Stop and brake
    t.angular.z = 0.0
    twist_pub.publish(t)
    brake_dir = -1 if w > 0 else 1
    abs_brake_angular(twist_pub, direction=brake_dir)

def spiral(
    twist_pub,
    duration=4.0,
    linear_v=0.5,
    start_w=0.1,
    end_w=1.0,
    turns=1.0,
    direction="left",
    cmd_dt=0.05
):
    """Smooth spiral motion by changing angular velocity over time.

    The robot moves at a constant linear speed while the angular velocity
    ramps linearly. The angular velocities are scaled so that the robot
    completes exactly `turns` rotations in `duration`.

    Args:
        twist_pub: ROS 2 publisher for Twist messages.
        duration (float): Total time for the spiral.
        linear_v (float): Constant linear velocity [m/s].
        start_w (float): Relative initial angular velocity.
        end_w (float): Relative final angular velocity.
        turns (float): Number of full rotations to complete.
        direction (str): "left" or "right".
        cmd_dt (float): Command period [s].
    """
    t = Twist()
    start_time = time.time()
    end_time = start_time + duration

    # Calculate scaling to ensure exactly 'turns' rotations
    # Average w * duration = 2 * pi * turns
    required_avg_w = (2.0 * math.pi * abs(turns)) / duration
    provided_avg = (start_w + end_w) / 2.0
    
    if abs(provided_avg) > 1e-6:
        scale = required_avg_w / abs(provided_avg)
        actual_start_w = abs(start_w) * scale
        actual_end_w = abs(end_w) * scale
    else:
        # Fallback if provided average is zero
        actual_start_w = required_avg_w
        actual_end_w = required_avg_w

    if direction.lower() == "right":
        actual_start_w = -actual_start_w
        actual_end_w = -actual_end_w

    while time.time() < end_time:
        elapsed = time.time() - start_time
        progress = elapsed / duration
        
        # Linear interpolation of angular velocity
        current_w = actual_start_w + (actual_end_w - actual_start_w) * progress
        
        t.linear.x = linear_v
        t.angular.z = current_w
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    # Stop
    t.linear.x = 0.0
    t.angular.z = 0.0
    twist_pub.publish(t)
    
    brake_dir = -1 if linear_v > 0 else 1
    abs_brake(twist_pub, direction=brake_dir)

def teacup(
    twist_pub,
    duration=20.0,
    radius_orbit=0.5,
    orbit_turns=1.0,
    spin_turns=4.0,
    direction="left",
    cmd_dt=0.05
):
    """Perform a 'teacup' style move: alternate between driving an arc and spinning in place.
    
    This simulates the teacup ride by breaking the orbit into segments. In each segment,
    the robot drives part of the circle, then stops to perform a full 360 degree spin.
    
    Args:
        twist_pub: ROS 2 publisher.
        duration (float): Total time.
        radius_orbit (float): Radius of the main circle.
        orbit_turns (float): Number of main circle orbits.
        spin_turns (float): Number of spins to perform (should be integer).
        direction (str): "left" or "right".
        cmd_dt (float): Command period.
    """
    t = Twist()
    
    # Ensure integer number of spins for the loop logic
    n_spins = max(1, int(round(spin_turns)))
    
    # Time allocation: 50% driving, 50% spinning
    drive_ratio = 0.5
    spin_ratio = 0.5
    
    time_per_segment = duration / n_spins
    time_drive = time_per_segment * drive_ratio
    time_spin = time_per_segment * spin_ratio
    
    # Arc parameters
    # Total orbit angle
    total_orbit_angle = 2.0 * math.pi * orbit_turns
    angle_per_arc = total_orbit_angle / n_spins
    
    dir_sign = 1.0 if direction.lower() == "left" else -1.0
    
    w_arc = (angle_per_arc / time_drive) * dir_sign
    v_arc = abs(w_arc * radius_orbit)
    
    # Spin parameters (1 full rotation per segment)
    w_spin = (2.0 * math.pi / time_spin) * dir_sign
    
    for _ in range(n_spins):
        # 1. Drive Arc
        end_drive = time.time() + time_drive
        while time.time() < end_drive:
            t.linear.x = v_arc
            t.angular.z = w_arc
            twist_pub.publish(t)
            time.sleep(cmd_dt)
            
        # 2. Spin in place
        end_spin = time.time() + time_spin
        while time.time() < end_spin:
            t.linear.x = 0.0
            t.angular.z = w_spin
            twist_pub.publish(t)
            time.sleep(cmd_dt)
            
    # Stop
    t.linear.x = 0.0
    t.angular.z = 0.0
    twist_pub.publish(t)
    abs_brake(twist_pub, direction=1)

def figure_eight(
    twist_pub,
    radius=0.5,
    duration=20.0,
    turns=1.0,
    cmd_dt=0.05
):
    """Drive a figure-eight pattern (two tangent circles).
    
    The robot drives one full circle to the left, then one full circle to the right.
    This creates a smooth figure-eight shape tangent to the starting direction.
    
    Args:
        twist_pub: ROS 2 publisher.
        radius (float): Radius of each circle [m].
        duration (float): Total duration for one full figure-eight (both circles) [s].
        turns (float): Number of times to repeat the full figure-eight.
        cmd_dt (float): Command period.
    """
    t = Twist()
    
    # Each figure eight is 2 circles.
    # Total time per circle = duration / 2
    # Speed v = 2*pi*R / (duration/2) = 4*pi*R / duration
    # Angular w = v / R = 4*pi / duration
    
    # We want to execute 'turns' full figure eights.
    # Total loops = turns * 2 (lefts and rights)
    
    full_cycles = int(turns)
    # If turns is 1.5, we do Full 8 + Half 8 (one circle).
    extra_half = (turns - full_cycles) >= 0.5
    
    # Duration for one circle
    circle_duration = duration / 2.0
    
    w_mag = (2.0 * math.pi) / circle_duration
    v_mag = w_mag * radius
    
    def drive_circle(direction):
        w = w_mag if direction == "left" else -w_mag
        end = time.time() + circle_duration
        while time.time() < end:
            t.linear.x = v_mag
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    for _ in range(full_cycles):
        drive_circle("left")
        drive_circle("right")
        
    if extra_half:
        drive_circle("left")
        
    # Stop
    t.linear.x = 0.0
    t.angular.z = 0.0
    twist_pub.publish(t)
    abs_brake(twist_pub, direction=-1)


def flower(
    twist_pub,
    radius=0.8,
    petals=2,
    duration=20.0,
    turns=1.0,
    cmd_dt=0.05
):
    """Drive a flower pattern (Rose Curve).
    
    Uses the polar equation r = a * sin(k * theta) to generate a smooth,
    petal-like path. The robot starts and ends at the center (if turns is integer).
    
    Args:
        twist_pub: ROS 2 publisher.
        radius (float): Approximate radius of the petals [m].
        petals (int): Parameter k for the rose curve.
                      k=2 produces 4 petals. k=3 produces 3 petals.
        duration (float): Duration for one full cycle (2*pi radians).
        turns (float): Number of full cycles to perform.
        cmd_dt (float): Command period.
    """
    t_msg = Twist()
    
    # k parameter
    k = float(petals)
    
    # Omega for the parameter theta (theta = omega * t)
    # We want theta to go from 0 to 2*pi*turns in 'duration'
    omega = (2.0 * math.pi * turns) / duration
    
    start_time = time.time()
    end_time = start_time + duration
    
    while time.time() < end_time:
        now = time.time() - start_time
        theta = omega * now
        dx_dtheta = radius * (k * math.cos(k*theta) * math.cos(theta) - math.sin(k*theta) * math.sin(theta))
        dy_dtheta = radius * (k * math.cos(k*theta) * math.sin(theta) + math.sin(k*theta) * math.cos(theta))
        
        vx = dx_dtheta * omega
        vy = dy_dtheta * omega
        
        # Linear velocity v
        v = math.sqrt(vx**2 + vy**2)
        d2x_dtheta2 = radius * ( -(k**2 + 1)*math.sin(k*theta)*math.cos(theta) - 2*k*math.cos(k*theta)*math.sin(theta))
        d2y_dtheta2 = radius * ( -(k**2 + 1)*math.sin(k*theta)*math.sin(theta) + 2*k*math.cos(k*theta)*math.cos(theta))
        
        ax = d2x_dtheta2 * (omega**2)
        ay = d2y_dtheta2 * (omega**2)
        
        if v > 1e-4:
            w_robot = (vx * ay - vy * ax) / (v**2)
        else:
            w_robot = 0.0

        t_msg.linear.x = v
        t_msg.angular.z = w_robot
        twist_pub.publish(t_msg)
        time.sleep(cmd_dt)

    # Stop
    t_msg.linear.x = 0.0
    t_msg.angular.z = 0.0
    twist_pub.publish(t_msg)
    abs_brake(twist_pub, direction=-1)

def wag_walking(
    twist_pub,
    duration=5.0,
    linear_speed=0.3,
    wag_frequency=0.5,
    wag_magnitude=0.8,
    cmd_dt=0.05
):
    """Move forward while intermittently glancing left and right.
    
    The robot moves forward at a constant speed. Periodically, it quickly
    rotates left-then-center, pauses, then right-then-center.
    
    Args:
        twist_pub: ROS 2 publisher.
        duration (float): Total time.
        linear_speed (float): Forward speed.
        wag_frequency (float): Frequency of the full left-right cycle [Hz].
        wag_magnitude (float): Angular velocity during the wag [rad/s].
        cmd_dt (float): Command period.
    """
    t = Twist()
    start_time = time.time()
    end_time = start_time + duration
    
    period = 1.0 / wag_frequency

    target_wag_duration = 0.6
    max_wag_duration = (period / 2.0) * 0.9 # Leave some gap
    wag_duration = min(target_wag_duration, max_wag_duration)
    
    while time.time() < end_time:
        now = time.time() - start_time
        cycle_time = now % period
        
        w = 0.0
        
        # First half of cycle: Wag Left
        if cycle_time < period / 2.0:
            if cycle_time < wag_duration:
                # Out (Left)
                if cycle_time < wag_duration / 2.0:
                    w = wag_magnitude
                # Back (Right)
                else:
                    w = -wag_magnitude
        
        # Second half of cycle: Wag Right
        else:
            rel_time = cycle_time - (period / 2.0)
            if rel_time < wag_duration:
                # Out (Right)
                if rel_time < wag_duration / 2.0:
                    w = -wag_magnitude
                # Back (Left)
                else:
                    w = wag_magnitude
                    
        t.linear.x = linear_speed
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)
        
    # Stop
    t.linear.x = 0.0
    t.angular.z = 0.0
    twist_pub.publish(t)
    abs_brake(twist_pub, direction=-1)

def peek_left_right(
    twist_pub,
    turn_duration=0.6,
    pause_duration=1.0,
    cmd_dt=0.05
):
    """Look left, then right, then center.
    
    Args:
        twist_pub: ROS 2 publisher.
        turn_duration (float): Time for each turn segment.
        pause_duration (float): Time to pause at each look.
        cmd_dt (float): Command period.
    """
    t = Twist()
    
    # 45 degrees
    target_angle = math.pi / 4.0
    w = target_angle / turn_duration
    
    # Turn Left
    end = time.time() + turn_duration
    while time.time() < end:
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)
    
    abs_brake_angular(twist_pub, direction=-1)
    time.sleep(pause_duration)
    
    # Turn Right (2x angle)
    end = time.time() + (turn_duration * 2.0)
    while time.time() < end:
        t.angular.z = -w
        twist_pub.publish(t)
        time.sleep(cmd_dt)
        
    abs_brake_angular(twist_pub, direction=1)
    time.sleep(pause_duration)
    
    # Turn Center (Left)
    end = time.time() + turn_duration
    while time.time() < end:
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)
        
    abs_brake_angular(twist_pub, direction=-1)

def bow_sequence(
    twist_pub,
    cmd_dt=0.05
):
    """Perform a bow sequence: Center, Left, Right.
    
    Sequence:
    1. Center: Step forward, wiggle, step back.
    2. Turn Left 45 deg.
    3. Left: Step forward, wiggle, step back.
    4. Turn Right 90 deg (to 45 deg Right of original).
    5. Right: Step forward, wiggle, step back.
    6. Return to Center (Turn Left 45 deg).
    
    Args:
        twist_pub: ROS 2 publisher.
        cmd_dt (float): Command period.
    """
    t = Twist()
    
    def perform_bow():
        # Step forward
        end = time.time() + 1.0
        while time.time() < end:
            t.linear.x = 0.3
            t.angular.z = 0.0
            twist_pub.publish(t)
            time.sleep(cmd_dt)
        abs_brake(twist_pub, direction=-1)
        
        # Wiggle
        for _ in range(4):
            t.linear.x = 0.0
            t.angular.z = 1.0
            twist_pub.publish(t)
            time.sleep(0.15)
            t.angular.z = -1.0
            twist_pub.publish(t)
            time.sleep(0.15)
        t.angular.z = 0.0
        twist_pub.publish(t)
        
        # Step back
        end = time.time() + 1.0
        while time.time() < end:
            t.linear.x = -0.3
            t.angular.z = 0.0
            twist_pub.publish(t)
            time.sleep(cmd_dt)
        abs_brake(twist_pub, direction=1)
        time.sleep(0.5)

    def turn(angle_rad, duration=1.0):
        w = angle_rad / duration
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = 0.0
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)
        t.angular.z = 0.0
        abs_brake_angular(twist_pub, direction=-1 if w > 0 else 1)
        time.sleep(0.5)

    # 1. Center Bow
    perform_bow()
    # 2. Turn Left 45 deg
    turn(math.pi / 4.0)
    # 3. Left Bow
    perform_bow()
    # 4. Turn Right 90 deg (to 45 deg Right of original)
    turn(-math.pi / 2.0)
    # 5. Right Bow
    perform_bow()
    # 6. Return to Center (Turn Left 45 deg)
    turn(math.pi / 4.0)
