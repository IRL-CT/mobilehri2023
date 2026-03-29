"""
Kinematic simulator for the Dance Studio web tool.

Provides offline simulation of robot dance sequences without requiring
a physical robot or ROS. Hardcodes displacement data from DiffDrivePlatform
and implements frame-by-frame trajectory generation for visualization.
"""

import math

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

pi = math.pi

# Hardcoded displacement table derived from DiffDrivePlatform.
# Each entry maps a move name to:
#   dx, dy   - net translation in the robot's local frame (metres)
#   dtheta   - net heading change (radians)
#   radius   - characteristic spatial extent of the move (metres)
#   returns  - True if the robot ends where it started (no net displacement)
DISPLACEMENTS = {
    "Glance":       {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 0.0,  "returns": True},
    "Bow":          {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 0.3,  "returns": True},
    "Step":         {"dx": 0.5, "dy": 0.0, "dtheta": 0.0,      "radius": 0.5,  "returns": False},
    "Glide":        {"dx": 0.6, "dy": 0.0, "dtheta": 0.0,      "radius": 0.6,  "returns": False},
    "Shimmy":       {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 0.0,  "returns": True},
    "Pulse":        {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 0.1,  "returns": True},
    "Vibrate":      {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 0.05, "returns": True},
    "Tap":          {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 0.1,  "returns": True},
    "Pirouette":    {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 0.6,  "returns": True},
    "Spin":         {"dx": 0.0, "dy": 0.0, "dtheta": 2 * pi,   "radius": 0.0,  "returns": True},
    "Zigzag":       {"dx": 0.8, "dy": 0.0, "dtheta": 0.0,      "radius": 1.0,  "returns": False},
    "Slalom":       {"dx": 2.4, "dy": 0.0, "dtheta": 0.0,      "radius": 2.5,  "returns": False},
    "WagWalk":      {"dx": 1.5, "dy": 0.0, "dtheta": 0.0,      "radius": 1.5,  "returns": False},
    "Arc":          {"dx": 0.0, "dy": 1.0, "dtheta": pi,        "radius": 1.0,  "returns": False},
    "TeacupSpin":   {"dx": 0.0, "dy": 1.0, "dtheta": 2 * pi,   "radius": 1.0,  "returns": False},
    "TeacupCircle": {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 1.0,  "returns": True},
    "Spiral":       {"dx": 0.5, "dy": 0.5, "dtheta": 2 * pi,   "radius": 1.5,  "returns": False},
    "FigureEight":  {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 1.0,  "returns": True},
    "Flower":       {"dx": 0.0, "dy": 0.0, "dtheta": 0.0,      "radius": 1.6,  "returns": True},
}

# Move categories for grouping in the UI and choreography logic.
CATEGORIES = {
    "social":     ["Glance", "Bow"],
    "linear":     ["Step", "Glide", "WagWalk"],
    "weaving":    ["Zigzag", "Slalom"],
    "spin":       ["Spin", "Pirouette"],
    "arc":        ["Arc", "TeacupSpin", "TeacupCircle"],
    "complex":    ["Spiral", "FigureEight", "Flower"],
    "percussive": ["Tap"],
    "expressive": ["Shimmy", "Pulse", "Vibrate"],
}

# Human-readable description of the simulated platform.
PLATFORM_DESCRIPTION = (
    "Simulated differential-drive mobile robot platform. "
    "Supports 19 dance primitives spanning social gestures, linear traversals, "
    "weaving patterns, spins, arcs, and complex composite trajectories. "
    "All displacements are given in the robot's local coordinate frame."
)


# ---------------------------------------------------------------------------
# StubPlatform — duck-types the RobotPlatform interface
# ---------------------------------------------------------------------------

class StubPlatform:
    """Lightweight stand-in for the real RobotPlatform.

    Provides the same query interface so that choreography code can run
    without a ROS connection or physical hardware.
    """

    def get_available_moves(self):
        """Return an ordered list of all move names."""
        return list(DISPLACEMENTS.keys())

    def get_move_displacements(self):
        """Return the full displacement dictionary."""
        return DISPLACEMENTS

    def get_move_categories(self):
        """Return move-category groupings."""
        return CATEGORIES

    def get_platform_description(self):
        """Return a human-readable platform description string."""
        return PLATFORM_DESCRIPTION


# ---------------------------------------------------------------------------
# Displacement adjustment
# ---------------------------------------------------------------------------

def adjust_displacement(move_name, params, base_disp):
    """Return a *copy* of *base_disp* adjusted for the given *params*.

    Handles direction reversal, angle overrides, and side selection for
    the moves that support parameterisation.

    Parameters
    ----------
    move_name : str
        Name of the move (must exist in DISPLACEMENTS).
    params : dict
        Move parameters from the choreography (e.g. direction, angle, side).
    base_disp : dict
        The base displacement dict to adjust (will not be mutated).

    Returns
    -------
    dict
        A new displacement dict with adjusted values.
    """
    disp = dict(base_disp)  # shallow copy

    direction = params.get("direction", "forward")
    angle = params.get("angle")  # degrees, or None
    side = params.get("side")

    # --- linear / weaving moves: backward negates dx ---
    if move_name in ("Step", "Glide", "Zigzag", "Slalom"):
        if direction == "backward":
            disp["dx"] = -disp["dx"]

    # --- Spin: angle override ---
    elif move_name == "Spin":
        if angle is not None:
            angle_rad = math.radians(float(angle))
            disp["dtheta"] = angle_rad
            if abs(float(angle)) != 360.0:
                disp["returns"] = False

    # --- Arc: direction + angle scaling ---
    elif move_name == "Arc":
        if direction == "right":
            disp["dy"] = -disp["dy"]
        # Angle defaults to 180 degrees; scale displacement proportionally.
        arc_angle = float(angle) if angle is not None else 180.0
        scale = arc_angle / 180.0
        disp["dy"] = disp["dy"] * scale
        disp["dtheta"] = disp["dtheta"] * scale

    # --- TeacupSpin: side selection ---
    elif move_name == "TeacupSpin":
        if side == "right":
            disp["dy"] = -disp["dy"]

    # --- Spiral: direction selection ---
    elif move_name == "Spiral":
        if direction == "right":
            disp["dy"] = -disp["dy"]

    return disp


# ---------------------------------------------------------------------------
# Motif expansion
# ---------------------------------------------------------------------------

def expand_motifs(motifs_list, tempo_bpm):
    """Expand a list of motif dicts into a flat sequence of move tuples.

    Each motif dict is expected to have at least:
        move_name, params (dict), energy (float), texture (str)
    and optionally a *modifier* dict describing repetition / variation.

    Parameters
    ----------
    motifs_list : list[dict]
        Motif descriptors from a phrase.
    tempo_bpm : float
        Tempo in beats per minute (used for gap calculations).

    Returns
    -------
    list[tuple]
        Each element is ``(move_name, params, gap_before, energy, texture)``.
    """
    beat = 60.0 / tempo_bpm
    expanded = []

    for motif in motifs_list:
        move_name = motif.get("move") or motif.get("move_name", "")
        params = dict(motif.get("params", {}))
        energy = float(motif.get("energy", 0.5))
        texture = motif.get("texture", "smooth")
        modifier = motif.get("modifier")
        gap = float(motif.get("gap_before", 0.0))

        if modifier is None:
            # No modifier — single copy.
            expanded.append((move_name, params, gap, energy, texture))
            continue

        mod_type = modifier.get("type", "")
        n = int(modifier.get("n", 2))
        factor = float(modifier.get("factor", 0.5))

        if mod_type == "repeat":
            # n identical copies separated by one beat.
            for i in range(n):
                g = gap if i == 0 else beat
                expanded.append((move_name, params, g, energy, texture))

        elif mod_type == "mirror":
            # Two copies: first with side=left, second with side=right.
            p_left = dict(params, side="left")
            p_right = dict(params, side="right")
            expanded.append((move_name, p_left, gap, energy, texture))
            expanded.append((move_name, p_right, beat, energy, texture))

        elif mod_type == "decay":
            # n copies with progressively increasing gap.
            current_gap = gap
            for i in range(n):
                g = current_gap if i == 0 else current_gap
                expanded.append((move_name, params, g, energy, texture))
                current_gap += beat * (1.0 - factor)

        elif mod_type == "crescendo":
            # n copies with progressively decreasing gap.
            current_gap = beat * n
            for i in range(n):
                g = gap if i == 0 else current_gap
                expanded.append((move_name, params, g, energy, texture))
                current_gap = max(0.0, current_gap - beat)

        elif mod_type == "tension":
            # Single copy with extra hold duration added to the gap.
            hold = float(modifier.get("hold_duration", beat * 2))
            expanded.append((move_name, params, gap + hold, energy, texture))

        elif mod_type == "alternate":
            # Interleave with another move.
            other_move = modifier.get("other_move", move_name)
            other_params = dict(modifier.get("other_params", {}))
            for i in range(n):
                if i % 2 == 0:
                    g = gap if i == 0 else beat
                    expanded.append((move_name, params, g, energy, texture))
                else:
                    expanded.append((other_move, other_params, beat, energy, texture))

        elif mod_type == "asymmetric_pause":
            # n copies with alternating short / long gaps.
            short_gap = beat * factor
            long_gap = beat * (2.0 - factor)
            for i in range(n):
                if i == 0:
                    g = gap
                elif i % 2 == 1:
                    g = short_gap
                else:
                    g = long_gap
                expanded.append((move_name, params, g, energy, texture))

        else:
            # Unknown modifier — fall back to single copy.
            expanded.append((move_name, params, gap, energy, texture))

    return expanded


# ---------------------------------------------------------------------------
# Duration estimation
# ---------------------------------------------------------------------------

def estimate_duration(energy):
    """Estimate the duration (seconds) of a move given its energy level.

    Higher energy yields shorter, snappier moves; lower energy yields
    slower, more drawn-out ones.

    Formula: clamp(3.0 / max(0.1, energy), 1.5, 8.0)
    """
    raw = 3.0 / max(0.1, energy)
    return max(1.5, min(8.0, raw))


# ---------------------------------------------------------------------------
# Helper: clamp
# ---------------------------------------------------------------------------

def _clamp(value, lo, hi):
    """Clamp *value* to the interval [lo, hi]."""
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# Main simulation entry point
# ---------------------------------------------------------------------------

def simulate(sequence_dict, energy_scale=1.0, tempo_bpm=None, fps=30):
    """Run a kinematic simulation of a choreography sequence.

    Parameters
    ----------
    sequence_dict : dict
        A sequence descriptor matching the AIChoreographer JSON schema.
        Expected keys: ``tempo_bpm``, ``phrases`` (list of phrase dicts).
        Each phrase has ``name``, ``motifs`` (list), and ``gap_after`` (float).
    energy_scale : float, optional
        Multiplier applied to every motif's energy (default 1.0).
    tempo_bpm : float or None, optional
        If given, overrides the tempo in *sequence_dict*.
    fps : int, optional
        Frames per second for the output trajectory (default 30).

    Returns
    -------
    dict
        ``{"frames": [...], "total_duration": float, "move_boundaries": [...]}``

        Each frame dict:
            t, x, y, theta, move_name, phrase_name, energy, texture

        Each move_boundary dict:
            t_start, t_end, move_name, phrase_name, energy, texture, params,
            annotation
    """
    # Resolve tempo.
    effective_tempo = tempo_bpm if tempo_bpm is not None else float(sequence_dict.get("tempo_bpm", 120))

    phrases = sequence_dict.get("phrases", [])

    # State variables.
    x, y, theta = 0.0, 0.0, 0.0
    t = 0.0

    frames = []
    move_boundaries = []

    dt = 1.0 / fps  # time step for stationary-gap frames

    for phrase in phrases:
        phrase_name = phrase.get("name", "")
        motifs = phrase.get("motifs", [])
        gap_after = float(phrase.get("gap_after", 0.0))

        # Expand motifs with their modifiers.
        expanded = expand_motifs(motifs, effective_tempo)

        for move_name, params, gap_before, energy, texture in expanded:
            effective_energy = _clamp(energy * energy_scale, 0.05, 1.0)
            duration = estimate_duration(effective_energy)

            # --- emit stationary frames for gap_before ---
            if gap_before > 0:
                n_gap = max(1, int(gap_before * fps))
                gap_dt = gap_before / n_gap
                for gi in range(n_gap):
                    t += gap_dt
                    frames.append({
                        "t": round(t, 6),
                        "x": round(x, 6),
                        "y": round(y, 6),
                        "theta": round(theta, 6),
                        "move_name": "",
                        "phrase_name": phrase_name,
                        "energy": 0.0,
                        "texture": "",
                    })

            # --- get adjusted displacement ---
            base_disp = DISPLACEMENTS.get(move_name)
            if base_disp is None:
                # Unknown move — skip gracefully.
                continue
            disp = adjust_displacement(move_name, params, base_disp)

            # --- record boundary start ---
            t_start = t

            # --- animate the move ---
            n_frames = max(1, int(duration * fps))
            annotation = params.get("annotation", "")

            for i in range(n_frames):
                progress = (i + 1) / n_frames

                if disp["returns"] and disp["radius"] > 0:
                    # Out-and-back trajectory.
                    if progress <= 0.5:
                        p = progress * 2.0  # 0 -> 1 during first half
                        local_dx = disp["radius"] * math.sin(p * pi)
                        local_dy = 0.0
                    else:
                        p = (progress - 0.5) * 2.0  # 0 -> 1 during second half
                        local_dx = disp["radius"] * math.sin((1.0 - p) * pi)
                        local_dy = 0.0

                    # Rotate local displacement by current heading.
                    frame_x = x + local_dx * math.cos(theta) - local_dy * math.sin(theta)
                    frame_y = y + local_dx * math.sin(theta) + local_dy * math.cos(theta)
                    frame_theta = theta + disp["dtheta"] * progress
                else:
                    # Linear interpolation toward final displacement.
                    dx_so_far = disp["dx"] * progress
                    dy_so_far = disp["dy"] * progress

                    # Rotate displacement by heading at start of move.
                    frame_x = x + dx_so_far * math.cos(theta) - dy_so_far * math.sin(theta)
                    frame_y = y + dx_so_far * math.sin(theta) + dy_so_far * math.cos(theta)
                    frame_theta = theta + disp["dtheta"] * progress

                frame_t = t + (i + 1) * duration / n_frames
                frames.append({
                    "t": round(frame_t, 6),
                    "x": round(frame_x, 6),
                    "y": round(frame_y, 6),
                    "theta": round(frame_theta, 6),
                    "move_name": move_name,
                    "phrase_name": phrase_name,
                    "energy": round(effective_energy, 4),
                    "texture": texture,
                })

            # --- update persistent state ---
            if not disp["returns"]:
                x += disp["dx"] * math.cos(theta) - disp["dy"] * math.sin(theta)
                y += disp["dx"] * math.sin(theta) + disp["dy"] * math.cos(theta)
            theta += disp["dtheta"]
            t += duration

            # --- record boundary end ---
            move_boundaries.append({
                "t_start": round(t_start, 6),
                "t_end": round(t, 6),
                "move_name": move_name,
                "phrase_name": phrase_name,
                "energy": round(effective_energy, 4),
                "texture": texture,
                "params": params,
                "annotation": annotation,
            })

        # --- phrase gap_after: emit stationary frames ---
        if gap_after > 0:
            n_gap = max(1, int(gap_after * fps))
            gap_dt = gap_after / n_gap
            for gi in range(n_gap):
                t += gap_dt
                frames.append({
                    "t": round(t, 6),
                    "x": round(x, 6),
                    "y": round(y, 6),
                    "theta": round(theta, 6),
                    "move_name": "",
                    "phrase_name": phrase_name,
                    "energy": 0.0,
                    "texture": "",
                })

    return {
        "frames": frames,
        "total_duration": round(t, 6),
        "move_boundaries": move_boundaries,
    }
