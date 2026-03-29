"""
Platform Abstraction Layer — Robot Choreography System

Defines the abstract interface that every robot platform must implement.
The choreography layer (choreographer.py) and action server (dance_server.py)
interact ONLY through this interface, never with platform-specific code directly.

Architecture
------------
    Choreography Layer  (platform-agnostic: Sequence, Phrase, Motif)
            │
            ▼
    RobotPlatform ABC   ← this file
            │
            ▼
    Platform Impl       (e.g. DiffDrivePlatform — publishes Twist)

Dancer Design Thinking Mapping
-------------------------------
§1 Foundational Sense of Movement:
    pre_roll()       → preparatory micro-movement before each move
    apply_texture()  → translate abstract texture intent to motor params
    subdivide_time() → non-linear velocity profiles

§2 Deconstructing Complex Movements:
    Layering via compose_channels() — path + expression + dynamics
    Marking via energy scaling — energy=0.1 for low-power rehearsal

§3 Maintaining Control Through Instability:
    active_brake()      → freeze = tension, not zero power
    spotting_anchor()   → virtual anchor during spins
    lean_into_error()   → let drift become variation

§4 Choreographing Through Structure:
    Handled in choreographer.py (motif memory, repetition+deviation, etc.)
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


# ── Texture Presets ──────────────────────────────────────────────────────────

class Texture(Enum):
    """Abstract movement texture presets inspired by dancer imagery.

    Each platform translates these into its own motor characteristics:
    - Differential drive: damping curves, torque profiles, ramp shapes
    - Quadruped: joint impedance, gait style
    - Arm: stiffness, smoothness
    """
    NEUTRAL = "neutral"      # default — balanced acceleration
    HONEY = "honey"          # viscous: high damping, slow onset, smooth
    STACCATO = "staccato"    # percussive: instant torque, abrupt stops
    ICE = "ice"              # frictionless: minimal damping, gliding
    CLOUD = "cloud"          # light: floaty, gentle, airy
    MAGNET = "magnet"        # attraction/repulsion: push-pull dynamics


# ── Move Context ─────────────────────────────────────────────────────────────

@dataclass
class MoveContext:
    """Expressive parameters passed from choreography to platform execution.

    These are abstract intent — the platform decides how to realize them
    in its own motor space. For example, energy=0.9 + texture=STACCATO
    means "explosive and sharp" — a diff-drive would use high torque with
    instant ramps, while a quadruped might use fast leg snaps.

    Attributes:
        energy: Intensity [0.0–1.0]. 0=restrained, 0.5=normal, 1=explosive.
        texture: Movement quality preset.
        weight: Laban weight quality — affects acceleration profile.
        noise_level: Organic variation [0.0–1.0]. 0=mechanical, 0.1=natural.
        enable_pre_roll: Whether to execute a preparatory micro-movement.
        enable_active_brake: Whether to use active braking after the move.
        params: Move-specific parameters (e.g. direction, side, angle).
    """
    energy: float = 0.5
    texture: Texture = Texture.NEUTRAL
    weight: str = "neutral"
    noise_level: float = 0.0
    enable_pre_roll: bool = True
    enable_active_brake: bool = True
    params: dict = field(default_factory=dict)


# ── Abstract Base Class ──────────────────────────────────────────────────────

class RobotPlatform(ABC):
    """Abstract base class for robot platform implementations.

    Every robot platform (differential drive, quadruped, arm, etc.) must
    subclass this and provide concrete implementations. The dance server
    and choreographer interact only through this interface.
    """

    @abstractmethod
    def get_available_moves(self) -> list[str]:
        """Return the list of move names this platform supports.

        These names are used by the choreographer and AI to compose sequences.
        They must match exactly what execute_move() accepts.
        """

    @abstractmethod
    def execute_move(self, move_name: str, context: MoveContext) -> bool:
        """Execute a named move with the given expressive context.

        This is the main entry point called by the dance server. The platform
        should:
        1. Look up the move by name
        2. Apply texture and energy from context to its motor parameters
        3. Execute the motion

        Args:
            move_name: A move name from get_available_moves().
            context: Expressive parameters (energy, texture, etc.).

        Returns:
            True if the move completed successfully, False if cancelled/failed.
        """

    @abstractmethod
    def pre_roll(self, move_name: str, context: MoveContext) -> None:
        """Execute a preparatory micro-movement before the main move.

        Inspired by dancer "physicalizing the breath" — a dancer never
        starts from absolute zero. There is always a prep: an inhale,
        a slight counter-shift, a micro-movement that signals intent.

        Each platform implements this differently:
        - Diff-drive: 100ms micro-vibration or subtle counter-steer
        - Quadruped: weight shift to the launching leg
        - Arm: slight retraction before extension

        Can be a no-op if enable_pre_roll is False in context.
        """

    @abstractmethod
    def active_brake(self, context: MoveContext) -> None:
        """Execute an active stop — freeze as tension, not absence of power.

        Inspired by dancer "counterbalance & active breaking" — to stop
        instantly, a dancer engages opposing muscles. A "freeze" is a
        highly active state of tension.

        Each platform implements this differently:
        - Diff-drive: brief counter-torque pulse then hold
        - Quadruped: stiffen all joints momentarily
        - Arm: increase joint impedance

        Can be a no-op if enable_active_brake is False in context.
        """

    @abstractmethod
    def stop(self) -> None:
        """Immediately stop all motion. Emergency/cleanup use."""

    def apply_texture(self, texture: Texture) -> dict:
        """Translate an abstract texture preset into platform-specific params.

        Override to provide platform-specific texture interpretation.
        Returns a dict of motor parameters that the platform's move
        functions can consume.

        Default implementation returns an empty dict (texture ignored).
        """
        return {}

    def spotting_anchor(self, target_heading: Optional[float] = None) -> None:
        """Set a virtual anchor point for sensory reset during spins.

        Inspired by dancer "spotting" — during rapid spins, the head
        stays fixed on one point until the last second, then snaps back.
        This prevents dizziness and maintains spatial orientation.

        Platforms with IMU/odometry can lock onto a heading and snap
        back to it. Default is a no-op.

        Args:
            target_heading: Target heading in radians. None to clear.
        """

    def lean_into_error(self, residual_velocity: float) -> Optional[str]:
        """When error/drift is detected, suggest a variation instead of correcting.

        Inspired by dancer "leaning into error" — if a dancer loses balance,
        they flow with the mistake, turning a stumble into a slide.

        Instead of forcing error correction, let residual momentum dictate
        the next move. Returns a suggested move name that flows from the
        error, or None for standard correction.

        Args:
            residual_velocity: Detected residual velocity after a move.

        Returns:
            Optional move name to flow into, or None.
        """
        return None

    def get_move_displacements(self) -> dict[str, dict]:
        """Return estimated displacement info for each move.

        Returns a dict mapping move names to displacement descriptors:
        {
            "MoveName": {
                "dx": float,      # forward displacement [m] (+ = forward)
                "dy": float,      # lateral displacement [m] (+ = left)
                "dtheta": float,  # heading change [rad] (+ = CCW)
                "radius": float,  # max distance from start during move [m]
                "returns": bool,  # True if move returns near its start position
            }
        }

        Values are approximate (open-loop). Used by AI choreographer for
        planning-level boundary awareness, not for precise navigation.
        """
        return {name: {"dx": 0.0, "dy": 0.0, "dtheta": 0.0, "radius": 0.0, "returns": True}
                for name in self.get_available_moves()}

    def get_move_categories(self) -> dict[str, list[str]]:
        """Return moves organized by category for AI choreographer prompts.

        Override to provide platform-specific categorization.
        Default returns a single 'all' category with all moves.
        """
        return {"all": self.get_available_moves()}

    def get_platform_description(self) -> str:
        """Return a human-readable description of this platform's capabilities.

        Used by AIChoreographer to generate platform-appropriate choreography.
        """
        return "Generic robot platform"
