"""
Differential Drive Platform — Robot Choreography System

Concrete RobotPlatform implementation for two-wheeled differential drive robots.
Publishes geometry_msgs/Twist commands via a ROS 2 publisher.

This module wraps all the motion primitives from dance_moves.py and exposes
them through the platform abstraction. The choreography layer never touches
Twist directly — it sends move names + MoveContext through the platform interface.

Dancer Design Thinking — Platform-Specific Implementations
-----------------------------------------------------------
§1 Pre-roll:        100ms micro-vibration or subtle counter-steer
§1 Texture presets: honey=high damping/slow ramp, staccato=instant torque,
                    ice=minimal damping, cloud=exponential ramps
§3 Active brake:    brief counter-torque pulse then zero
§3 Spotting:        virtual heading anchor via odometry (placeholder)
§3 Lean into error: residual velocity → suggested flow move
"""

from __future__ import annotations

import math
import time
from typing import Optional

from dance_manager.platform import RobotPlatform, MoveContext, Texture
from dance_manager.dance_moves import (
    step, glide, drive_arc, glance, shimmy, pulse, vibrate,
    tap_on_side, zigzag, pirouette, slalom, spin_on_axis,
    teacup_spin, spiral, teacup, figure_eight, flower,
    wag_walking, bow_sequence,
    _brake, _brake_angular, _stop,
)


# ── Texture → motor parameter translation ────────────────────────────────────

TEXTURE_PROFILES = {
    Texture.NEUTRAL: {
        "ramp_up_scale": 1.0,
        "ramp_down_scale": 1.0,
        "profile": "linear",
        "damping": 1.0,
    },
    Texture.HONEY: {
        "ramp_up_scale": 2.0,      # slow onset
        "ramp_down_scale": 2.0,    # slow settle
        "profile": "linear",
        "damping": 2.0,            # high damping
    },
    Texture.STACCATO: {
        "ramp_up_scale": 0.3,      # instant torque
        "ramp_down_scale": 0.2,    # abrupt stop
        "profile": "linear",
        "damping": 0.5,
    },
    Texture.ICE: {
        "ramp_up_scale": 0.5,      # quick start
        "ramp_down_scale": 3.0,    # long coast
        "profile": "linear",
        "damping": 0.2,            # minimal damping — gliding
    },
    Texture.CLOUD: {
        "ramp_up_scale": 1.5,
        "ramp_down_scale": 1.5,
        "profile": "exponential",  # floaty ease-in/out
        "damping": 0.8,
    },
    Texture.MAGNET: {
        "ramp_up_scale": 0.4,      # snappy attraction
        "ramp_down_scale": 0.4,    # snappy repulsion
        "profile": "exponential",
        "damping": 1.5,
    },
}


class DiffDrivePlatform(RobotPlatform):
    """Differential drive robot platform (e.g., hoverboard robot).

    Publishes Twist messages to a ROS 2 topic. All motion primitives
    from dance_moves.py are registered as named moves.

    Args:
        twist_pub: A rclpy Publisher[geometry_msgs.msg.Twist].
        track_width: Distance between wheels [m]. Default 0.6.
    """

    def __init__(self, twist_pub, track_width: float = 0.6):
        self._twist_pub = twist_pub
        self._track_width = track_width
        self._move_registry = self._build_move_registry()

    def _build_move_registry(self) -> dict:
        """Build the mapping from move names to callables.

        Each entry is a function that takes (twist_pub, **extra_kwargs).
        The execute_move method handles translating MoveContext into kwargs.
        """
        tp = self._twist_pub
        track = self._track_width

        return {
            # Social gestures
            "Greeting":             lambda **kw: glance(tp, turn_duration=1.0, pause_duration=1.0, **kw),
            "PeekLeftRight":        lambda **kw: glance(tp, turn_duration=0.6, pause_duration=1.0, **kw),
            "Bow":                  lambda **kw: bow_sequence(tp, **kw),
            # Linear steps
            "InchForward":          lambda **kw: step(tp, direction="forward",  ramp_up_duration=0.2, ramp_down_duration=0.2, **kw),
            "StepForward":          lambda **kw: step(tp, direction="forward",  ramp_up_duration=0.7, ramp_down_duration=0.3, profile="exponential", **kw),
            "RollForward":          lambda **kw: step(tp, direction="forward",  ramp_up_duration=1.5, ramp_down_duration=0.8, **kw),
            "InchBackward":         lambda **kw: step(tp, direction="backward", ramp_up_duration=0.2, ramp_down_duration=0.2, **kw),
            "StepBackward":         lambda **kw: step(tp, direction="backward", ramp_up_duration=0.7, ramp_down_duration=0.3, profile="exponential", **kw),
            "GlideForward":         lambda **kw: glide(tp, direction="forward",  duration=2.0, speed=0.3, **kw),
            "GlideBackward":        lambda **kw: glide(tp, direction="backward", duration=2.0, speed=0.3, **kw),
            # Expressive in-place
            "Shimmy":               lambda **kw: shimmy(tp, duration=3.0, frequency=3.0, **kw),
            "ShimmyFast":           lambda **kw: shimmy(tp, duration=2.0, frequency=5.0, magnitude=1.2, **kw),
            "Pulse":                lambda **kw: pulse(tp, n=4, **kw),
            "Vibrate":              lambda **kw: vibrate(tp, duration=2.0, **kw),
            # Pivots and taps
            "TapOnLeft":            lambda **kw: tap_on_side(tp, side="left", track=track, **kw),
            "TapOnRight":           lambda **kw: tap_on_side(tp, side="right", track=track, **kw),
            "PirouetteLeft":        lambda **kw: pirouette(tp, side="left", track=track, **kw),
            "PirouetteRight":       lambda **kw: pirouette(tp, side="right", track=track, **kw),
            # Axis spins
            "SpinClockwise":        lambda **kw: spin_on_axis(tp, clockwise=True, **kw),
            "SpinCounterClockwise": lambda **kw: spin_on_axis(tp, clockwise=False, **kw),
            "Spin180CW":            lambda **kw: spin_on_axis(tp, clockwise=True,  rotations=0.5,  spin_duration=2.5, **kw),
            "Spin180CCW":           lambda **kw: spin_on_axis(tp, clockwise=False, rotations=0.5,  spin_duration=2.5, **kw),
            "Spin90CW":             lambda **kw: spin_on_axis(tp, clockwise=True,  rotations=0.25, spin_duration=2.0, **kw),
            "Spin90CCW":            lambda **kw: spin_on_axis(tp, clockwise=False, rotations=0.25, spin_duration=2.0, **kw),
            "Spin15CW":             lambda **kw: spin_on_axis(tp, clockwise=True,  rotations=0.04, spin_duration=1.0, **kw),
            "Spin15CCW":            lambda **kw: spin_on_axis(tp, clockwise=False, rotations=0.04, spin_duration=1.0, **kw),
            # Weaving paths
            "ZigZaggingForward":    lambda **kw: zigzag(tp, direction="forward", track=track, **kw),
            "ZigZaggingBackward":   lambda **kw: zigzag(tp, direction="backward", track=track, **kw),
            "SlalomForward":        lambda **kw: slalom(tp, direction="forward", **kw),
            "SlalomBackward":       lambda **kw: slalom(tp, direction="backward", **kw),
            "WagWalk":              lambda **kw: wag_walking(tp, **kw),
            # Arc / circle patterns
            "ArcLeft":              lambda **kw: drive_arc(tp, radius=0.5, angle=math.pi, direction="left", **kw),
            "ArcRight":             lambda **kw: drive_arc(tp, radius=0.5, angle=math.pi, direction="right", **kw),
            "TeacupSpinLeft":       lambda **kw: teacup_spin(tp, side="left", **kw),
            "TeacupSpinRight":      lambda **kw: teacup_spin(tp, side="right", **kw),
            "TeacupCircleLeft":     lambda **kw: teacup(tp, direction="left", **kw),
            "TeacupCircleRight":    lambda **kw: teacup(tp, direction="right", **kw),
            # Complex paths
            "SpiralLeft":           lambda **kw: spiral(tp, direction="left", **kw),
            "SpiralRight":          lambda **kw: spiral(tp, direction="right", **kw),
            "FigureEight":          lambda **kw: figure_eight(tp, **kw),
            "FlowerDance":          lambda **kw: flower(tp, **kw),
        }

    # ── RobotPlatform interface ──────────────────────────────────────────────

    def get_available_moves(self) -> list[str]:
        return list(self._move_registry.keys())

    def execute_move(self, move_name: str, context: MoveContext) -> bool:
        if move_name not in self._move_registry:
            return False

        # Translate MoveContext into dancer vocabulary kwargs
        kwargs = self._context_to_kwargs(context)

        # §1: Pre-roll — preparatory micro-movement
        if context.enable_pre_roll:
            self.pre_roll(move_name, context)

        # Execute the move
        self._move_registry[move_name](**kwargs)

        # §3: Active brake — freeze as tension
        if context.enable_active_brake:
            self.active_brake(context)

        return True

    def pre_roll(self, move_name: str, context: MoveContext) -> None:
        """100ms micro-vibration or subtle counter-steer before the move.

        For diff-drive, this is a brief vibration that signals "I'm about
        to move" — like a dancer's preparatory breath.
        """
        if not context.enable_pre_roll:
            return

        from geometry_msgs.msg import Twist
        t = Twist()
        intensity = 0.05 + 0.05 * context.energy  # scale with energy

        # Brief vibration: 2 quick oscillations over ~100ms
        for _ in range(2):
            t.angular.z = intensity
            self._twist_pub.publish(t)
            time.sleep(0.025)
            t.angular.z = -intensity
            self._twist_pub.publish(t)
            time.sleep(0.025)

        t.angular.z = 0.0
        self._twist_pub.publish(t)

    def active_brake(self, context: MoveContext) -> None:
        """Brief counter-torque pulse then zero — freeze as active tension.

        Instead of just publishing zero velocity, we pulse a brief
        counter-movement to create a crisp, intentional stop.
        """
        if not context.enable_active_brake:
            return

        from geometry_msgs.msg import Twist
        t = Twist()

        # Brief counter-pulse (the "tension" of stopping)
        brake_intensity = 0.05 + 0.05 * context.energy
        t.linear.x = -brake_intensity
        self._twist_pub.publish(t)
        time.sleep(0.03)

        # Then firm zero
        t.linear.x = 0.0
        t.angular.z = 0.0
        self._twist_pub.publish(t)

    def stop(self) -> None:
        _stop(self._twist_pub)

    def apply_texture(self, texture: Texture) -> dict:
        return TEXTURE_PROFILES.get(texture, TEXTURE_PROFILES[Texture.NEUTRAL])

    def spotting_anchor(self, target_heading: Optional[float] = None) -> None:
        # Placeholder — requires odometry/IMU integration
        pass

    def lean_into_error(self, residual_velocity: float) -> Optional[str]:
        """Suggest a flow move based on residual drift direction."""
        if abs(residual_velocity) < 0.05:
            return None
        if residual_velocity > 0:
            return "GlideForward"   # flowing with forward drift
        return "GlideBackward"      # flowing with backward drift

    def get_move_categories(self) -> dict[str, list[str]]:
        return {
            "social":     ["Greeting", "PeekLeftRight", "Bow"],
            "forward":    ["InchForward", "StepForward", "RollForward",
                           "GlideForward", "WagWalk", "SlalomForward",
                           "ZigZaggingForward"],
            "backward":   ["InchBackward", "StepBackward", "GlideBackward",
                           "SlalomBackward", "ZigZaggingBackward"],
            "spin":       ["SpinClockwise", "SpinCounterClockwise",
                           "Spin180CW", "Spin180CCW",
                           "Spin90CW", "Spin90CCW",
                           "Spin15CW", "Spin15CCW"],
            "pivot":      ["PirouetteLeft", "PirouetteRight",
                           "TeacupSpinLeft", "TeacupSpinRight"],
            "arc":        ["ArcLeft", "ArcRight",
                           "TeacupCircleLeft", "TeacupCircleRight"],
            "complex":    ["SpiralLeft", "SpiralRight",
                           "FigureEight", "FlowerDance"],
            "percussive": ["TapOnLeft", "TapOnRight"],
            "expressive": ["Shimmy", "ShimmyFast", "Pulse", "Vibrate"],
        }

    def get_platform_description(self) -> str:
        return (
            "A differential-drive dance robot with two wheels. "
            "It can move forward/backward, spin in place, pivot around one wheel, "
            "and drive arcs, spirals, figure-eights, and flower curves. "
            f"Track width: {self._track_width}m."
        )

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _context_to_kwargs(self, context: MoveContext) -> dict:
        """Translate MoveContext into kwargs that dance_moves functions accept."""
        kwargs = {}

        if context.energy != 0.5:
            kwargs["energy"] = context.energy
        if context.weight != "neutral":
            kwargs["weight"] = context.weight
        if context.noise_level > 0.0:
            kwargs["noise_level"] = context.noise_level

        # Apply texture as ramp scaling
        # The texture profiles modify how the underlying move functions behave
        # by adjusting their timing parameters through the weight/energy system
        tex = TEXTURE_PROFILES.get(context.texture, TEXTURE_PROFILES[Texture.NEUTRAL])
        if context.texture == Texture.CLOUD:
            kwargs.setdefault("weight", "light")
        elif context.texture == Texture.HONEY:
            kwargs.setdefault("weight", "heavy")

        return kwargs
