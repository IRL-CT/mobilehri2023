"""
Layer 2 — Movement Modifiers

Higher-order function wrappers around Layer 1 motion primitives that add
expressive variation. Each modifier takes a callable move function and
returns a new callable with the same twist_pub-first signature.

Modifier types
--------------
repeat      : execute a move N times with an optional gap between
mirror      : flip the 'side' parameter (left ↔ right) and run both
decay       : repeat N times, energy shrinking by `factor` each time
crescendo   : repeat N times, energy growing by `factor` each time
with_noise  : inject organic randomness into numeric parameters
tension     : hold still for a beat of anticipation before the move
release     : execute with boosted energy (explosive after tension)
alternate   : interleave two moves N times each

Sequence-level spec helpers (seq_*)
------------------------------------
These return plain dicts describing a modifier for use in choreographer.py.
The dict is consumed by run_sequence() in choreographer.py and expanded
into individual action-server goal calls without touching twist_pub directly.

Usage — direct twist_pub wrappers
----------------------------------
    from dance_manager.dance_moves import tap_on_side, slalom
    from dance_manager.movement_modifiers import repeat, decay, with_noise

    # Repeat tap 4 times with a 0.2s gap
    four_taps = repeat(tap_on_side, 4, gap=0.2)
    four_taps(twist_pub, side="right")

    # Slalom with organic variation
    noisy_slalom = with_noise(slalom, noise_level=0.12)
    noisy_slalom(twist_pub, direction="forward")

Usage — sequence-level specs (for AIChoreographer)
---------------------------------------------------
    from dance_manager.movement_modifiers import seq_repeat, seq_crescendo

    motif = {
        "move": "Tap",
        "modifier": seq_repeat(n=4, gap=0.2),
    }
"""

import time
import random
from typing import Callable

from dance_manager.dance_moves import (
    energy_to_speed,
    weight_to_accel,
    flow_to_gap,
    add_noise,
)

__all__ = [
    # twist_pub wrappers
    "repeat",
    "mirror",
    "decay",
    "crescendo",
    "with_noise",
    "tension",
    "release",
    "alternate",
    # vocabulary re-exports
    "energy_to_speed",
    "weight_to_accel",
    "flow_to_gap",
    "add_noise",
    # sequence-level specs
    "seq_repeat",
    "seq_mirror",
    "seq_decay",
    "seq_crescendo",
    "seq_tension",
    "seq_alternate",
    "seq_asymmetric_pause",
]


# ── twist_pub-level wrappers ──────────────────────────────────────────────────

def repeat(move_fn: Callable, n: int, gap: float = 0.0) -> Callable:
    """Return a callable that executes move_fn n times with an optional gap.

    Args:
        move_fn: Motion primitive with signature move_fn(twist_pub, **params).
        n (int): Number of repetitions.
        gap (float): Seconds to pause between repetitions.
    """
    def _inner(twist_pub, **params):
        for i in range(n):
            move_fn(twist_pub, **params)
            if gap > 0.0 and i < n - 1:
                time.sleep(gap)
    return _inner


def mirror(move_fn: Callable, side_param: str = "side") -> Callable:
    """Return a callable that runs move_fn twice: left side then right side.

    The first call passes side_param='left', the second passes 'right'.
    Any side value already present in kwargs is overridden.

    Args:
        move_fn: Motion primitive that accepts a `side` kwarg.
        side_param (str): Name of the side keyword argument.
    """
    def _inner(twist_pub, **params):
        p_left = dict(params)
        p_left[side_param] = "left"
        move_fn(twist_pub, **p_left)

        p_right = dict(params)
        p_right[side_param] = "right"
        move_fn(twist_pub, **p_right)
    return _inner


def decay(move_fn: Callable, n: int, factor: float = 0.7, gap: float = 0.0) -> Callable:
    """Repeat move_fn n times with energy shrinking by factor each iteration.

    Models a movement quality that fades — like a bouncing ball losing height,
    or a gesture that winds down after a peak.

    Starting energy defaults to 0.8 if not provided in kwargs.
    Each repetition multiplies the current energy by `factor` (0 < factor < 1).

    Args:
        move_fn: Motion primitive.
        n (int): Number of repetitions.
        factor (float): Energy multiplier per step. Values <1 produce decay.
        gap (float): Seconds between repetitions.
    """
    def _inner(twist_pub, **params):
        energy = params.get("energy", 0.8)
        for i in range(n):
            p = dict(params)
            p["energy"] = max(0.05, energy)
            move_fn(twist_pub, **p)
            energy *= factor
            if gap > 0.0 and i < n - 1:
                time.sleep(gap)
    return _inner


def crescendo(move_fn: Callable, n: int, factor: float = 1.3, gap: float = 0.0) -> Callable:
    """Repeat move_fn n times with energy growing by factor each iteration.

    Models a movement quality that builds — like an accelerating rhythm,
    or a gesture that intensifies toward a climax.

    Starting energy is set so that the last repetition lands near 0.9.
    Each repetition multiplies the current energy by `factor` (>1 grows).

    Args:
        move_fn: Motion primitive.
        n (int): Number of repetitions.
        factor (float): Energy multiplier per step. Values >1 produce growth.
        gap (float): Seconds between repetitions.
    """
    def _inner(twist_pub, **params):
        # Back-calculate start energy so final energy ≈ 0.9
        final_target = 0.9
        start_energy = final_target / (factor ** (n - 1)) if n > 1 else final_target
        start_energy = max(0.05, min(start_energy, 1.0))
        energy = params.get("energy", start_energy)
        for i in range(n):
            p = dict(params)
            p["energy"] = min(1.0, energy)
            move_fn(twist_pub, **p)
            energy *= factor
            if gap > 0.0 and i < n - 1:
                time.sleep(gap)
    return _inner


def with_noise(move_fn: Callable, noise_level: float = 0.1) -> Callable:
    """Inject organic randomness into a move's parameters each call.

    Passes noise_level into the move's kwargs so the Layer 1 _apply_vocab
    helper perturbs numeric parameters (speed, ramp times) by ±noise_level.

    noise_level=0.0  → deterministic, machine-precise
    noise_level=0.1  → ±10% variation (feels natural)
    noise_level=0.25 → ±25% variation (feels improvisational)

    Args:
        move_fn: Motion primitive.
        noise_level (float): Gaussian noise fraction applied to numeric params.
    """
    def _inner(twist_pub, **params):
        p = dict(params)
        p["noise_level"] = noise_level
        move_fn(twist_pub, **p)
    return _inner


def tension(move_fn: Callable, hold_duration: float = 1.0) -> Callable:
    """Hold the robot still for hold_duration seconds before executing the move.

    In dance, tension is suspended energy — the moment before a leap or strike.
    The held stillness makes the subsequent motion feel more intentional and
    dramatically weighted.

    Args:
        move_fn: Motion primitive.
        hold_duration (float): Seconds to hold still (anticipation beat).
    """
    from geometry_msgs.msg import Twist

    def _inner(twist_pub, **params):
        stop = Twist()
        end = time.time() + hold_duration
        while time.time() < end:
            twist_pub.publish(stop)
            time.sleep(0.05)
        move_fn(twist_pub, **params)
    return _inner


def release(move_fn: Callable, speed_boost: float = 1.5) -> Callable:
    """Execute move_fn with boosted energy — a sudden release of held tension.

    After tension (stillness or restraint), release is the explosive moment
    when the movement breaks free. This modifier amps up the energy kwarg
    so the primitive runs faster/bigger.

    Args:
        move_fn: Motion primitive.
        speed_boost (float): Multiplier applied to the energy kwarg.
    """
    def _inner(twist_pub, **params):
        p = dict(params)
        p["energy"] = min(1.0, p.get("energy", 0.5) * speed_boost)
        move_fn(twist_pub, **p)
    return _inner


def alternate(move_fn_a: Callable, move_fn_b: Callable,
              n: int, gap: float = 0.0) -> Callable:
    """Interleave two moves A and B, repeating n complete pairs.

    Produces: A, B, A, B, ... (n times each).

    Useful for call-and-response patterns, like alternating left and right
    taps, or switching between a step forward and a step back.

    Args:
        move_fn_a: First motion primitive.
        move_fn_b: Second motion primitive.
        n (int): Number of A-B pairs.
        gap (float): Seconds between each move.
    """
    def _inner(twist_pub, **params):
        for i in range(n):
            move_fn_a(twist_pub, **params)
            if gap > 0.0:
                time.sleep(gap)
            move_fn_b(twist_pub, **params)
            if gap > 0.0 and i < n - 1:
                time.sleep(gap)
    return _inner


# ── Sequence-level spec helpers ───────────────────────────────────────────────
# These produce plain dicts consumed by choreographer.run_sequence().
# They do NOT call twist_pub. They describe how to expand a single Motif
# into multiple action-server goals.

def seq_repeat(n: int, gap: float = 0.0) -> dict:
    """Sequence spec: repeat a move n times with optional gap between [s]."""
    return {"type": "repeat", "n": n, "gap": gap}


def seq_mirror(gap: float = 0.0) -> dict:
    """Sequence spec: run the move twice — once with side='left', once side='right'.

    The run_sequence() runner injects {"side": "left"} then {"side": "right"}
    into the motif's params dict. Works with any move that accepts a side param
    (e.g. Tap, Pirouette, TeacupSpin).
    """
    return {"type": "mirror", "gap": gap}


def seq_decay(n: int, factor: float = 0.7) -> dict:
    """Sequence spec: repeat n times with increasing gap (energy fading)."""
    return {"type": "decay", "n": n, "factor": factor}


def seq_crescendo(n: int, factor: float = 1.3) -> dict:
    """Sequence spec: repeat n times with decreasing gap (urgency building)."""
    return {"type": "crescendo", "n": n, "factor": factor}


def seq_tension(hold_duration: float = 1.0) -> dict:
    """Sequence spec: add hold_duration seconds of stillness before the move."""
    return {"type": "tension", "hold_duration": hold_duration}


def seq_alternate(other_move: str, n: int, gap: float = 0.0) -> dict:
    """Sequence spec: interleave this move with other_move, n pairs total."""
    return {"type": "alternate", "other_move": other_move, "n": n, "gap": gap}


def seq_asymmetric_pause(short: float = 0.2, long: float = 0.8, n: int = 4) -> dict:
    """Sequence spec: repeat a move with alternating short/long pauses.

    Inspired by dancer "power of the pause" — the most intense part of a dance
    is often the stillness. Asymmetric pauses let the audience see the robot
    "deciding" what to do next, creating tension and anticipation.

    Args:
        short: Duration of the short pause [s].
        long: Duration of the long pause [s].
        n: Number of repetitions.
    """
    return {"type": "asymmetric_pause", "short": short, "long": long, "n": n}
