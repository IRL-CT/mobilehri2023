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

Dancer Vocabulary
- All primitives accept optional kwargs: energy, weight, noise_level.
- energy (0-1)      : maps to speed scale; 0=restrained, 0.5=nominal, 1=explosive
- weight (str)      : 'light'|'neutral'|'heavy' maps to ramp timing and profile
- noise_level (0-1) : adds Gaussian variation for organic, non-mechanical motion
"""
import time
import math
import random
from geometry_msgs.msg import Twist


# ── Vocabulary helpers ────────────────────────────────────────────────────────

def energy_to_speed(energy: float, base_speed: float = 0.4) -> float:
    """Map dancer energy [0, 1] to a speed value scaled from base_speed.

    energy=0   → 0.3 × base_speed  (restrained, slow)
    energy=0.5 → 1.0 × base_speed  (nominal)
    energy=1.0 → 2.0 × base_speed  (explosive, fast)
    """
    energy = max(0.0, min(1.0, energy))
    return base_speed * (0.3 + 1.7 * (energy ** 0.7))


def weight_to_accel(weight: str) -> dict:
    """Map dancer weight quality to ramp timing parameters.

    Returns a dict with 'ramp_up_duration', 'ramp_down_duration', 'profile'.
    'light'   → quick start and stop, exponential profile (floaty, airy)
    'neutral' → balanced timing, linear profile
    'heavy'   → slow build, slow settle, linear profile (grounded, weighted)
    """
    profiles = {
        'light':   {'ramp_up_duration': 0.2,  'ramp_down_duration': 0.15, 'profile': 'exponential'},
        'neutral': {'ramp_up_duration': 0.5,  'ramp_down_duration': 0.25, 'profile': 'linear'},
        'heavy':   {'ramp_up_duration': 0.9,  'ramp_down_duration': 0.70, 'profile': 'linear'},
    }
    return profiles.get(weight.lower(), profiles['neutral'])


def flow_to_gap(flow: str) -> float:
    """Map Laban flow quality to a pause duration between movements [s].

    'bound'   → 0.0  (staccato; moves run directly into each other)
    'neutral' → 0.3  (natural breathing room)
    'free'    → 0.8  (generous pause for breath, suspension, anticipation)
    """
    return {'bound': 0.0, 'neutral': 0.3, 'free': 0.8}.get(flow.lower(), 0.3)


def add_noise(value: float, noise_level: float,
              min_val: float = None, max_val: float = None) -> float:
    """Perturb a parameter value with Gaussian noise for organic variation.

    noise_level=0.1 → ±≈10% variation (σ = 10% of |value|)
    """
    if noise_level <= 0.0:
        return value
    result = value + random.gauss(0.0, abs(value) * noise_level)
    if min_val is not None:
        result = max(min_val, result)
    if max_val is not None:
        result = min(max_val, result)
    return result


def _apply_vocab(kwargs: dict, base_speed: float, ramp_up: float, ramp_down: float):
    """Resolve dancer vocabulary kwargs into (speed_scale, ramp_up, ramp_down)."""
    speed_scale = 1.0

    if 'energy' in kwargs:
        speed_scale = energy_to_speed(kwargs['energy'], base_speed) / base_speed

    if 'weight' in kwargs:
        accel = weight_to_accel(kwargs['weight'])
        ramp_up = accel['ramp_up_duration']
        ramp_down = accel['ramp_down_duration']

    if 'noise_level' in kwargs:
        nl = kwargs['noise_level']
        speed_scale = add_noise(speed_scale, nl, min_val=0.1)
        ramp_up = add_noise(ramp_up, nl, min_val=0.05)
        ramp_down = add_noise(ramp_down, nl, min_val=0.05)

    return speed_scale, ramp_up, ramp_down


# ── Internal braking helpers ──────────────────────────────────────────────────

def _brake(twist_pub, direction, times=5, dt=0.05):
    """Pulse a small linear command to damp residual motion."""
    t = Twist()
    for _ in range(times):
        t.linear.x = direction * 0.1
        twist_pub.publish(t)
        time.sleep(dt)


def _brake_angular(twist_pub, direction, times=5, dt=0.05, intensity=0.2):
    """Pulse a small angular command to damp residual rotation."""
    t = Twist()
    for _ in range(times):
        t.angular.z = direction * intensity
        twist_pub.publish(t)
        time.sleep(dt)
    t.angular.z = 0.0
    twist_pub.publish(t)


def _stop(twist_pub):
    """Publish a zero Twist to halt the robot."""
    twist_pub.publish(Twist())


# ── Primitives ────────────────────────────────────────────────────────────────

def step(
    twist_pub,
    direction="forward",
    max_speed=1.0,
    ramp_up_duration=0.5,
    ramp_down_duration=0.25,
    profile="linear",
    cmd_dt=0.05,
    **kwargs
):
    """Short translational step: ramp up, hold briefly, ramp down.

    Replaces inch_forward, inch_forward_exponential, and inch_backward.
    The profile controls the acceleration curve shape:
      'linear'      : constant acceleration (uniform ramp)
      'exponential' : quadratic ease-in / ease-out (floaty, weight='light' feel)

    Args:
        direction (str): 'forward' or 'backward'.
        max_speed (float): peak linear velocity [m/s].
        ramp_up_duration (float): seconds to reach max_speed.
        ramp_down_duration (float): seconds to decelerate to zero.
        profile (str): 'linear' or 'exponential'.
    """
    speed_scale, ramp_up_duration, ramp_down_duration = _apply_vocab(
        kwargs, base_speed=max_speed, ramp_up=ramp_up_duration, ramp_down=ramp_down_duration)
    peak = max_speed * speed_scale
    sign = -1.0 if direction.lower() == "backward" else 1.0

    t = Twist()
    start = time.time()

    while time.time() <= start + ramp_up_duration:
        progress = (time.time() - start) / ramp_up_duration
        if profile == "exponential":
            t.linear.x = sign * peak * (progress ** 2)
        else:
            t.linear.x = sign * peak * progress
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    start = time.time()
    while time.time() <= start + ramp_down_duration:
        progress = (time.time() - start) / ramp_down_duration
        if profile == "exponential":
            t.linear.x = sign * peak * ((1 - progress) ** 2)
        else:
            t.linear.x = sign * peak * (1.0 - progress)
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _brake(twist_pub, direction=-sign)


def glide(
    twist_pub,
    direction="forward",
    duration=2.0,
    speed=0.3,
    cmd_dt=0.05,
    **kwargs
):
    """Constant-speed travel with no ramp — sustained, flowing quality.

    Unlike step(), there is no acceleration curve. The robot moves at a
    steady speed, then stops cleanly. Good for 'sustained' Laban effort quality.

    Args:
        direction (str): 'forward' or 'backward'.
        duration (float): total travel time [s].
        speed (float): constant linear velocity [m/s].
    """
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=speed, ramp_up=duration, ramp_down=duration)
    sign = -1.0 if direction.lower() == "backward" else 1.0
    v = speed * speed_scale * sign

    t = Twist()
    end = time.time() + duration
    while time.time() < end:
        t.linear.x = v
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake(twist_pub, direction=-sign)


def drive_arc(
    twist_pub,
    radius=0.5,
    angle=math.pi,
    direction="left",
    cmd_dt=0.05,
    **kwargs
):
    """Drive a constant-radius circular arc for a given angle.

    This is the fundamental differential drive primitive. All complex paths
    (figure-eight, teacup, spiral entry) are composed from arcs.

    v = w * radius, so angular velocity w = v / radius.
    Duration is computed from the desired angle: duration = angle / w.

    Args:
        radius (float): turning radius [m]. Larger = gentler curve.
        angle (float): arc angle to subtend [rad]. math.pi = half circle.
        direction (str): 'left' (CCW) or 'right' (CW).
    """
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=0.4, ramp_up=1.0, ramp_down=1.0)
    base_v = 0.4 * speed_scale
    w = base_v / radius
    duration = abs(angle) / w
    w_signed = w if direction.lower() == "left" else -w

    t = Twist()
    end = time.time() + duration
    while time.time() < end:
        t.linear.x = base_v
        t.angular.z = w_signed
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake(twist_pub, direction=-1)


def glance(
    twist_pub,
    turn_duration=0.6,
    pause_duration=1.0,
    cmd_dt=0.05,
    **kwargs
):
    """Look left, hold, look right, hold, return to center.

    A social gesture: the robot scans left then right, like greeting an audience.
    Replaces both greeting() and peek_left_right().

    Args:
        turn_duration (float): time to rotate 45 degrees [s].
        pause_duration (float): dwell time at each extreme [s].
    """
    speed_scale, turn_duration, _ = _apply_vocab(
        kwargs, base_speed=0.4, ramp_up=turn_duration, ramp_down=turn_duration)
    w = (math.pi / 4.0 / turn_duration) * speed_scale  # rad/s for 45 deg

    t = Twist()

    def _turn(angular_speed, duration):
        end = time.time() + duration
        while time.time() < end:
            t.angular.z = angular_speed
            twist_pub.publish(t)
            time.sleep(cmd_dt)
        _brake_angular(twist_pub, direction=-1 if angular_speed > 0 else 1)

    _turn(w, turn_duration)           # Left 45°
    time.sleep(pause_duration)
    _turn(-w, 2.0 * turn_duration)   # Right 90° (through center)
    time.sleep(pause_duration)
    _turn(w, turn_duration)           # Back to center


def shimmy(
    twist_pub,
    duration=3.0,
    frequency=3.0,
    magnitude=0.8,
    cmd_dt=0.05,
    **kwargs
):
    """Rapid angular oscillation in place — the robot shakes or wiggles.

    No net displacement. Expresses excitement, nervousness, or playfulness.
    The robot rocks left and right at the given frequency without traveling.

    Args:
        duration (float): total shimmy time [s].
        frequency (float): oscillation frequency [Hz]. 1-2 = slow shimmy, 4-6 = fast shake.
        magnitude (float): peak angular velocity [rad/s].
    """
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=magnitude, ramp_up=duration, ramp_down=duration)
    magnitude = magnitude * speed_scale

    t = Twist()
    start = time.time()
    end = start + duration
    while time.time() < end:
        elapsed = time.time() - start
        t.linear.x = 0.0
        t.angular.z = magnitude * math.sin(2 * math.pi * frequency * elapsed)
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake_angular(twist_pub, direction=1)


def pulse(
    twist_pub,
    n=4,
    forward_duration=0.2,
    backward_duration=0.15,
    speed=0.3,
    gap=0.05,
    cmd_dt=0.05,
    **kwargs
):
    """Rapid forward-backward beats in place — a heartbeat or bounce motion.

    The robot surges forward briefly then returns, repeating n times.
    Produces a percussive, rhythmic quality without net displacement.

    Args:
        n (int): number of pulse beats.
        forward_duration (float): duration of the forward stroke [s].
        backward_duration (float): duration of the return stroke [s].
        speed (float): linear velocity magnitude [m/s].
        gap (float): pause between beats [s].
    """
    speed_scale, forward_duration, backward_duration = _apply_vocab(
        kwargs, base_speed=speed, ramp_up=forward_duration, ramp_down=backward_duration)
    v = speed * speed_scale

    t = Twist()
    for i in range(n):
        # Forward stroke
        end = time.time() + forward_duration
        while time.time() < end:
            t.linear.x = v
            twist_pub.publish(t)
            time.sleep(cmd_dt)

        # Return stroke
        end = time.time() + backward_duration
        while time.time() < end:
            t.linear.x = -v
            twist_pub.publish(t)
            time.sleep(cmd_dt)

        if gap > 0 and i < n - 1:
            _stop(twist_pub)
            time.sleep(gap)

    _stop(twist_pub)


def vibrate(
    twist_pub,
    duration=2.0,
    intensity=0.08,
    cmd_dt=0.03,
    **kwargs
):
    """Tiny high-frequency random perturbations — trembling or nervous energy.

    Both linear.x and angular.z are randomly perturbed each tick, creating
    an organic shivering quality. Useful for tension before a big move,
    or to express a fearful / excited state.

    Args:
        duration (float): total vibration time [s].
        intensity (float): maximum perturbation magnitude [m/s or rad/s].
    """
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=intensity, ramp_up=duration, ramp_down=duration)
    intensity = intensity * speed_scale

    t = Twist()
    end = time.time() + duration
    while time.time() < end:
        t.linear.x = random.uniform(-intensity, intensity)
        t.angular.z = random.uniform(-intensity * 4, intensity * 4)
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)


def tap_on_side(
    twist_pub,
    side="left",
    tap_times=1,
    tap_duration=0.25,
    v_mag=0.40,
    track=0.6,
    cmd_dt=0.05,
    **kwargs
):
    """Tap on one wheel while the other stays fixed, producing a pivot motion.

    For a differential drive with track width `track`, approximates the base
    linear and angular velocities from the active wheel's linear speed (v_mag):
      side='left' : pivots about the right wheel  →  v = v_L/2,  w = -v_L/track
      side='right': pivots about the left wheel   →  v = v_R/2,  w = +v_R/track

    Args:
        side (str): 'left' or 'right' — which wheel performs the tap.
        tap_times (int): number of forward/backward tap pairs.
        tap_duration (float): seconds for each stroke.
        v_mag (float): linear speed [m/s] applied to the active wheel.
        track (float): track width [m] between wheels.
    """
    speed_scale, tap_duration, _ = _apply_vocab(
        kwargs, base_speed=v_mag, ramp_up=tap_duration, ramp_down=tap_duration)
    v = v_mag * speed_scale

    t = Twist()

    def _stroke(v_side, duration):
        if side.lower() == "left":
            lx, az = 0.5 * v_side, -v_side / track
        elif side.lower() == "right":
            lx, az = 0.5 * v_side, v_side / track
        else:
            raise ValueError("side must be 'left' or 'right'")
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = lx
            t.angular.z = az
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    for _ in range(tap_times):
        _stroke(v, tap_duration)
        _brake(twist_pub, direction=-1)
        _stroke(-v, tap_duration)
        _brake(twist_pub, direction=1)


def zigzag(
    twist_pub,
    direction="forward",
    turns=4,
    tap_duration=0.5,
    v_mag=0.40,
    track=0.6,
    cmd_dt=0.05,
    **kwargs
):
    """Advance forward or backward while alternating left/right pivot arcs.

    Args:
        direction (str): 'forward' or 'backward'.
        turns (int): number of left/right arc pairs.
        tap_duration (float): seconds per arc.
        v_mag (float): base speed [m/s].
        track (float): track width [m].
    """
    speed_scale, tap_duration, _ = _apply_vocab(
        kwargs, base_speed=v_mag, ramp_up=tap_duration, ramp_down=tap_duration)
    vm = v_mag * speed_scale

    if direction.lower() == "forward":
        dir_sign, brake_dir = 1, -1
    elif direction.lower() == "backward":
        dir_sign, brake_dir = -1, 1
    else:
        raise ValueError("direction must be 'forward' or 'backward'")

    t = Twist()

    def _arc(w_sign, duration):
        end = time.time() + duration
        while time.time() < end:
            t.linear.x = 0.5 * vm * dir_sign
            t.angular.z = w_sign * vm / track
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    if turns > 0:
        _arc(-1, tap_duration / 2.0)
        _brake(twist_pub, direction=brake_dir)
        _arc(1, tap_duration)
        _brake(twist_pub, direction=brake_dir)
        for _ in range(turns - 1):
            _arc(-1, tap_duration)
            _brake(twist_pub, direction=brake_dir)
            _arc(1, tap_duration)
            _brake(twist_pub, direction=brake_dir)
        _arc(-1, tap_duration / 2.0)
        _brake(twist_pub, direction=brake_dir)


def pirouette(
    twist_pub,
    side="left",
    spin_duration=5.0,
    track=0.60,
    cmd_dt=0.05,
    **kwargs
):
    """Pivot the robot around one wheel for a full 360° turn.

    Args:
        side (str): 'left' or 'right' — which wheel stays fixed.
        spin_duration (float): seconds to complete 360°.
        track (float): distance between wheels [m].
    """
    speed_scale, spin_duration, _ = _apply_vocab(
        kwargs, base_speed=0.4, ramp_up=spin_duration, ramp_down=spin_duration)
    spin_duration = spin_duration / speed_scale

    w_sign = 1.0 if side.lower() == "left" else -1.0 if side.lower() == "right" else (_ for _ in ()).throw(ValueError("side must be 'left' or 'right'"))
    w = 2.0 * math.pi / spin_duration
    v = w * (track / 2.0)

    t = Twist()
    end = time.time() + spin_duration
    while time.time() < end:
        t.linear.x = v
        t.angular.z = w_sign * w
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake(twist_pub, brake_times=10, direction=-1)


def slalom(
    twist_pub,
    direction="forward",
    duration=6.0,
    linear_speed=0.4,
    oscillation_amp=1.2,
    frequency=0.5,
    cmd_dt=0.05,
    **kwargs
):
    """Move continuously while weaving side-to-side in a sine-wave pattern.

    Duration is snapped to a full number of oscillation periods so the robot
    returns to its original heading.

    Args:
        direction (str): 'forward' or 'backward'.
        duration (float): approximate total time [s].
        linear_speed (float): constant linear speed [m/s].
        oscillation_amp (float): peak angular velocity [rad/s].
        frequency (float): weave frequency [Hz].
    """
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=linear_speed, ramp_up=duration, ramp_down=duration)
    linear_speed *= speed_scale
    oscillation_amp *= speed_scale
    lin_sign = 1.0 if direction.lower() == "forward" else -1.0

    period = 1.0 / frequency
    actual_duration = max(1, round(duration / period)) * period

    t = Twist()
    start = time.time()
    end = start + actual_duration
    while time.time() < end:
        elapsed = time.time() - start
        t.linear.x = linear_speed * lin_sign
        t.angular.z = oscillation_amp * math.cos(2 * math.pi * frequency * elapsed)
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake(twist_pub, direction=-lin_sign)


def spin_on_axis(
    twist_pub,
    rotations=1.0,
    spin_duration=8.0,
    clockwise=False,
    cmd_dt=0.05,
    **kwargs
):
    """Spin in place about the robot's center axis.

    Args:
        rotations (float): number of full 360° rotations.
        spin_duration (float): total time [s].
        clockwise (bool): if True, rotate clockwise (negative angular.z).
    """
    speed_scale, spin_duration, _ = _apply_vocab(
        kwargs, base_speed=0.4, ramp_up=spin_duration, ramp_down=spin_duration)
    spin_duration = spin_duration / speed_scale

    if spin_duration <= 0 or rotations == 0:
        return

    w_mag = 2.0 * math.pi * rotations / spin_duration
    # Clamp angular velocity to reduce wheel slip in simulation
    max_w = 1.5  # rad/s
    if w_mag > max_w:
        spin_duration = 2.0 * math.pi * rotations / max_w
        w_mag = max_w
    w = -w_mag if clockwise else w_mag

    t = Twist()
    end = time.time() + spin_duration
    while time.time() < end:
        t.linear.x = 0.0
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    t.angular.z = 0.0
    twist_pub.publish(t)
    _brake_angular(twist_pub, direction=-1 if w > 0 else 1)


def teacup_spin(
    twist_pub,
    side="left",
    duration=8.0,
    radius=1.0,
    cmd_dt=0.05,
    **kwargs
):
    """Spin 360° while laterally displacing (Disney teacup style).

    The robot completes a full rotation while linearly ramping its forward
    velocity from +v_limit to -v_limit, creating a net lateral displacement.

    Args:
        side (str): 'left' or 'right' (rotation and displacement direction).
        duration (float): total time [s].
        radius (float): approximate lateral displacement [m].
    """
    speed_scale, duration, _ = _apply_vocab(kwargs, base_speed=0.4, ramp_up=duration, ramp_down=duration)
    duration = duration / speed_scale

    w_mag = 2.0 * math.pi / duration
    w = w_mag if side.lower() == "left" else -w_mag if side.lower() == "right" else (_ for _ in ()).throw(ValueError("side must be 'left' or 'right'"))
    v_limit = radius * w_mag / 2.0

    t = Twist()
    start = time.time()
    end = start + duration
    while time.time() < end:
        progress = (time.time() - start) / duration
        t.linear.x = v_limit + (-2 * v_limit) * progress   # ramps +v_limit → -v_limit
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _brake(twist_pub, direction=1)


def spiral(
    twist_pub,
    duration=4.0,
    linear_v=0.5,
    start_w=0.1,
    end_w=1.0,
    turns=1.0,
    direction="left",
    cmd_dt=0.05,
    **kwargs
):
    """Smooth spiral: constant linear speed, angular velocity ramps over time.

    Angular velocities are scaled so the robot completes exactly `turns` rotations.

    Args:
        linear_v (float): constant linear velocity [m/s].
        start_w (float): relative initial angular velocity.
        end_w (float): relative final angular velocity.
        turns (float): number of full rotations to complete.
        direction (str): 'left' or 'right'.
    """
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=linear_v, ramp_up=duration, ramp_down=duration)
    linear_v *= speed_scale

    required_avg_w = (2.0 * math.pi * abs(turns)) / duration
    provided_avg = (start_w + end_w) / 2.0
    scale = required_avg_w / abs(provided_avg) if abs(provided_avg) > 1e-6 else 1.0
    aw0 = abs(start_w) * scale
    aw1 = abs(end_w) * scale
    if direction.lower() == "right":
        aw0, aw1 = -aw0, -aw1

    t = Twist()
    start = time.time()
    end = start + duration
    while time.time() < end:
        progress = (time.time() - start) / duration
        t.linear.x = linear_v
        t.angular.z = aw0 + (aw1 - aw0) * progress
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake(twist_pub, direction=-1 if linear_v > 0 else 1)


def teacup(
    twist_pub,
    duration=20.0,
    radius_orbit=0.5,
    orbit_turns=1.0,
    spin_turns=4.0,
    direction="left",
    cmd_dt=0.05,
    **kwargs
):
    """Alternate between driving a circle arc and spinning in place.

    Simulates the teacup ride: orbit segments interleaved with in-place spins.

    Args:
        radius_orbit (float): orbit radius [m].
        orbit_turns (float): total number of main-circle orbits.
        spin_turns (float): number of in-place spins (integer).
        direction (str): 'left' or 'right'.
    """
    speed_scale, duration, _ = _apply_vocab(kwargs, base_speed=0.4, ramp_up=duration, ramp_down=duration)
    duration = duration / speed_scale

    n = max(1, int(round(spin_turns)))
    seg = duration / n
    t_drive, t_spin = seg * 0.5, seg * 0.5

    dir_sign = 1.0 if direction.lower() == "left" else -1.0
    angle_per_arc = (2.0 * math.pi * orbit_turns) / n
    w_arc = (angle_per_arc / t_drive) * dir_sign
    v_arc = abs(w_arc * radius_orbit)
    w_spin = (2.0 * math.pi / t_spin) * dir_sign

    t = Twist()
    for _ in range(n):
        end = time.time() + t_drive
        while time.time() < end:
            t.linear.x = v_arc
            t.angular.z = w_arc
            twist_pub.publish(t)
            time.sleep(cmd_dt)

        end = time.time() + t_spin
        while time.time() < end:
            t.linear.x = 0.0
            t.angular.z = w_spin
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake(twist_pub, direction=1)


def figure_eight(
    twist_pub,
    radius=0.5,
    duration=20.0,
    turns=1.0,
    cmd_dt=0.05,
    **kwargs
):
    """Drive a figure-eight pattern (two tangent circles, left then right).

    Args:
        radius (float): radius of each circle [m].
        duration (float): total duration for one complete figure-eight [s].
        turns (float): number of full figure-eights to perform.
    """
    speed_scale, duration, _ = _apply_vocab(kwargs, base_speed=0.4, ramp_up=duration, ramp_down=duration)
    duration = duration / speed_scale

    circle_dur = duration / 2.0
    w_mag = (2.0 * math.pi) / circle_dur
    v_mag = w_mag * radius

    t = Twist()

    def _circle(w):
        end = time.time() + circle_dur
        while time.time() < end:
            t.linear.x = v_mag
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)

    full = int(turns)
    for _ in range(full):
        _circle(w_mag)
        _circle(-w_mag)
    if (turns - full) >= 0.5:
        _circle(w_mag)

    _stop(twist_pub)
    _brake(twist_pub, direction=-1)


def flower(
    twist_pub,
    radius=0.8,
    petals=2,
    duration=20.0,
    turns=1.0,
    cmd_dt=0.05,
    **kwargs
):
    """Drive a rose-curve flower pattern.

    Uses the polar equation r = a·sin(k·θ). k=2 → 4 petals, k=3 → 3 petals.

    Args:
        radius (float): approximate petal radius [m].
        petals (int): rose-curve k parameter.
        duration (float): duration for one full 2π cycle [s].
        turns (float): number of full cycles.
    """
    speed_scale, duration, _ = _apply_vocab(kwargs, base_speed=0.4, ramp_up=duration, ramp_down=duration)
    duration = duration / speed_scale

    k = float(petals)
    omega = (2.0 * math.pi * turns) / duration

    t = Twist()
    start = time.time()
    end = start + duration
    while time.time() < end:
        theta = omega * (time.time() - start)
        dx = radius * (k * math.cos(k * theta) * math.cos(theta) - math.sin(k * theta) * math.sin(theta))
        dy = radius * (k * math.cos(k * theta) * math.sin(theta) + math.sin(k * theta) * math.cos(theta))
        vx, vy = dx * omega, dy * omega
        v = math.sqrt(vx ** 2 + vy ** 2)
        d2x = radius * (-(k**2 + 1) * math.sin(k*theta) * math.cos(theta) - 2*k * math.cos(k*theta) * math.sin(theta))
        d2y = radius * (-(k**2 + 1) * math.sin(k*theta) * math.sin(theta) + 2*k * math.cos(k*theta) * math.cos(theta))
        ax, ay = d2x * omega**2, d2y * omega**2
        w = (vx * ay - vy * ax) / (v**2) if v > 1e-4 else 0.0
        t.linear.x = v
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake(twist_pub, direction=-1)


def wag_walking(
    twist_pub,
    duration=5.0,
    linear_speed=0.3,
    wag_frequency=0.5,
    wag_magnitude=0.8,
    cmd_dt=0.05,
    **kwargs
):
    """Move forward while periodically glancing left and right.

    Args:
        linear_speed (float): forward speed [m/s].
        wag_frequency (float): full left-right cycle frequency [Hz].
        wag_magnitude (float): angular velocity during the wag [rad/s].
    """
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=linear_speed, ramp_up=duration, ramp_down=duration)
    linear_speed *= speed_scale
    wag_magnitude *= speed_scale

    period = 1.0 / wag_frequency
    wag_dur = min(0.6, period * 0.45)

    t = Twist()
    start = time.time()
    end = start + duration
    while time.time() < end:
        ct = (time.time() - start) % period
        if ct < period / 2.0:
            half_ct = ct
            w = wag_magnitude if half_ct < wag_dur / 2.0 else (-wag_magnitude if half_ct < wag_dur else 0.0)
        else:
            half_ct = ct - period / 2.0
            w = -wag_magnitude if half_ct < wag_dur / 2.0 else (wag_magnitude if half_ct < wag_dur else 0.0)
        t.linear.x = linear_speed
        t.angular.z = w
        twist_pub.publish(t)
        time.sleep(cmd_dt)

    _stop(twist_pub)
    _brake(twist_pub, direction=-1)


# ── Compound move (Layer 2 territory — kept here for server compatibility) ────

def bow_sequence(twist_pub, cmd_dt=0.05, **kwargs):
    """Bow to center, left, and right audiences.

    Note: this is a compound routine (step + shimmy + turn × 3). It is kept
    here for backward compatibility but conceptually belongs in Layer 2.
    """
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=0.3, ramp_up=1.0, ramp_down=1.0)

    t = Twist()

    def _bow():
        end = time.time() + 1.0
        while time.time() < end:
            t.linear.x = 0.3 * speed_scale
            t.angular.z = 0.0
            twist_pub.publish(t)
            time.sleep(cmd_dt)
        _brake(twist_pub, direction=-1)
        for _ in range(4):
            t.linear.x = 0.0
            t.angular.z = 1.0 * speed_scale
            twist_pub.publish(t)
            time.sleep(0.15)
            t.angular.z = -1.0 * speed_scale
            twist_pub.publish(t)
            time.sleep(0.15)
        t.angular.z = 0.0
        twist_pub.publish(t)
        end = time.time() + 1.0
        while time.time() < end:
            t.linear.x = -0.3 * speed_scale
            t.angular.z = 0.0
            twist_pub.publish(t)
            time.sleep(cmd_dt)
        _brake(twist_pub, direction=1)
        time.sleep(0.5)

    def _turn(angle_rad, dur=1.0):
        w = angle_rad / dur
        end = time.time() + dur
        while time.time() < end:
            t.linear.x = 0.0
            t.angular.z = w
            twist_pub.publish(t)
            time.sleep(cmd_dt)
        t.angular.z = 0.0
        _brake_angular(twist_pub, direction=-1 if w > 0 else 1)
        time.sleep(0.5)

    _bow()
    _turn(math.pi / 4.0)
    _bow()
    _turn(-math.pi / 2.0)
    _bow()
    _turn(math.pi / 4.0)


# ── Backward-compatibility aliases ────────────────────────────────────────────
# Old names used in dance_server.py. These will be removed in a future version.

def greeting(twist_pub, turn_duration=1.0, pause_duration=1.0, cmd_dt=0.05, **kwargs):
    """Alias for glance(). Use glance() in new code."""
    glance(twist_pub, turn_duration=turn_duration, pause_duration=pause_duration, cmd_dt=cmd_dt, **kwargs)

def peek_left_right(twist_pub, turn_duration=0.6, pause_duration=1.0, cmd_dt=0.05, **kwargs):
    """Alias for glance(). Use glance() in new code."""
    glance(twist_pub, turn_duration=turn_duration, pause_duration=pause_duration, cmd_dt=cmd_dt, **kwargs)

def inch_forward(twist_pub, ramp_up_duration=0.5, ramp_down_duration=0.25, **kwargs):
    """Alias for step(). Use step() in new code."""
    step(twist_pub, direction="forward", ramp_up_duration=ramp_up_duration,
         ramp_down_duration=ramp_down_duration, **kwargs)

def inch_forward_exponential(twist_pub, ramp_up_duration=0.5, ramp_down_duration=0.25, max_speed=1.0, **kwargs):
    """Alias for step(profile='exponential'). Use step() in new code."""
    step(twist_pub, direction="forward", max_speed=max_speed,
         ramp_up_duration=ramp_up_duration, ramp_down_duration=ramp_down_duration,
         profile="exponential", **kwargs)

def inch_backward(twist_pub, ramp_up_duration=0.5, ramp_down_duration=0.25, **kwargs):
    """Alias for step(direction='backward'). Use step() in new code."""
    step(twist_pub, direction="backward", ramp_up_duration=ramp_up_duration,
         ramp_down_duration=ramp_down_duration, **kwargs)

# Keep old brake names in case anything external calls them
abs_brake = _brake
abs_brake_angular = _brake_angular
