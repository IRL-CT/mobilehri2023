"""
Layer 3 — Choreographic Structure (Platform-Agnostic)

Provides high-level data structures (Motif, Phrase, Sequence) and an
AI-powered choreographer that generates full sequences from natural language
descriptions using the Google Gemini API.

This layer is entirely platform-agnostic. It expresses choreographic INTENT
(move names, energy, texture, timing) without knowing how any robot executes
the moves. The platform abstraction layer translates intent into motor commands.

Dancer Design Thinking — Choreography Layer Concepts
------------------------------------------------------
§1 Texture:          abstract movement quality (honey, staccato, ice, cloud, magnet)
§2 Marking:          energy=0.1 for low-power rehearsal / preview
§4 Motif Memory:     track signature moves, recall them to build personality
§4 Repetition+Dev:   A-A-A-B patterns via decay/growth modifiers
§4 Asymmetric Pause: the most intense part is often the stillness
§4 Reason/Intent:    annotation + phrase intent describe WHY, not just WHAT

Architecture
------------
Motif    : atomic unit — one named move + energy + texture + modifier
Phrase   : ordered list of Motifs with a name, intent, and trailing pause
Sequence : ordered list of Phrases with metadata (tempo, mood, title)

Workflow
--------
1. Programmatic: build Motif / Phrase / Sequence manually, call run_sequence().
2. AI-driven: instantiate AIChoreographer with a RobotPlatform,
   call .generate(description), receive a Sequence, then call run_sequence().

Usage — programmatic
---------------------
    from dance_manager.choreographer import Motif, Phrase, Sequence, run_sequence
    from dance_manager.movement_modifiers import seq_repeat, seq_crescendo

    sequence = Sequence(
        title="Simple Groove",
        tempo_bpm=120.0,
        mood="playful",
        phrases=[
            Phrase(name="intro", intent="greet the audience", motifs=[
                Motif("Glance", energy=0.6, texture="neutral"),
                Motif("WagWalk", energy=0.5, gap_before=0.3),
            ]),
            Phrase(name="climax", gap_after=1.0, motifs=[
                Motif("Tap", params={"side": "right"}, modifier=seq_crescendo(n=4), annotation="build up"),
                Motif("Spin", params={"angle": 360}, energy=0.9, texture="staccato"),
            ]),
            Phrase(name="outro", motifs=[
                Motif("Bow", energy=0.4, texture="honey"),
            ]),
        ],
    )
    run_sequence(action_client, sequence)

Usage — AI choreography (platform-aware)
-----------------------------------------
    import rclpy
    from dance_manager.dance_client import DanceActionClient
    from dance_manager.choreographer import AIChoreographer, run_sequence
    from dance_manager.platforms.differential_drive import DiffDrivePlatform

    rclpy.init()
    client = DanceActionClient()
    platform = DiffDrivePlatform(twist_pub)

    ai = AIChoreographer(api_key="YOUR_GEMINI_API_KEY", platform=platform)
    sequence = ai.generate(
        "A curious, exploratory dance: start cautious, then grow more playful "
        "and energetic, end with a graceful bow."
    )
    run_sequence(client, sequence)
"""

from __future__ import annotations

import json
import math
import random
import time
from dataclasses import dataclass, field
from typing import Optional


# ── Data Structures ───────────────────────────────────────────────────────────

@dataclass
class Motif:
    """The atomic unit of choreography: one named move with expressive intent.

    Attributes:
        move: A primitive move name recognised by the RobotPlatform (e.g. "Spin").
        params: Move-specific parameters (e.g. {"angle": 90}, {"direction": "forward"},
                {"side": "left"}). Passed to the platform for interpretation.
        energy: Expressive intensity [0.0-1.0]. Used to scale motion and gaps.
        texture: Movement quality preset name ("neutral", "honey", "staccato",
                 "ice", "cloud", "magnet"). Passed to the platform for
                 interpretation into motor parameters.
        gap_before: Explicit pause [s] inserted before sending this goal.
        modifier: Optional dict produced by a seq_* helper in movement_modifiers
                  (e.g. seq_repeat(4, gap=0.2)). Expanded by run_sequence.
        annotation: Human-readable note shown in logs. Has no effect on motion.
    """
    move: str
    params: dict = field(default_factory=dict)
    energy: float = 0.5
    texture: str = "neutral"
    gap_before: float = 0.0
    modifier: Optional[dict] = None
    annotation: str = ""


@dataclass
class Phrase:
    """An ordered list of Motifs forming a musical phrase or section.

    Attributes:
        motifs: Ordered sequence of Motif objects.
        name: Human-readable label (e.g. 'intro', 'chorus', 'bridge').
        intent: WHY this phrase exists — the reason behind the movements.
                Inspired by dancer thinking: "one causes another, or the echo effect."
        gap_after: Seconds of stillness appended after the final motif.
    """
    motifs: list = field(default_factory=list)
    name: str = ""
    intent: str = ""
    gap_after: float = 0.0


@dataclass
class Sequence:
    """A complete choreographic sequence with metadata.

    Attributes:
        phrases: Ordered list of Phrase objects.
        title: Human-readable title of the dance.
        tempo_bpm: Musical tempo used to scale gap timing globally.
        mood: Single expressive descriptor (e.g. 'playful', 'dramatic', 'dreamy').
        description: Free-text summary of the choreographic intention.
    """
    phrases: list = field(default_factory=list)
    title: str = ""
    tempo_bpm: float = 120.0
    mood: str = "neutral"
    description: str = ""


# ── Motif Memory ─────────────────────────────────────────────────────────────

class MotifMemory:
    """Track used motifs and recall signature moves to build personality.

    Inspired by dancer "repeat but variate" — repeat a motif from earlier
    in a different context. This creates a "memory" for the audience and
    builds a recognizable personality for the robot.

    Usage:
        memory = MotifMemory()
        # Record moves as they execute
        memory.record("Tap")
        memory.record("Spin")
        # Get a signature move to recall
        sig = memory.get_signature_move()  # → "Tap" (most used)
        # Check if it's time to recall
        if memory.should_recall(current_index=10, total_moves=20):
            # Insert the signature move
            ...
    """

    def __init__(self, recall_interval: int = 5):
        """
        Args:
            recall_interval: Suggest a recall every N moves.
        """
        self._history: list[str] = []
        self._counts: dict[str, int] = {}
        self._recall_interval = recall_interval
        self._last_recall_index = -recall_interval  # allow first recall early

    def record(self, move_name: str) -> None:
        """Record a move that was just executed."""
        self._history.append(move_name)
        self._counts[move_name] = self._counts.get(move_name, 0) + 1

    def get_signature_move(self) -> Optional[str]:
        """Return the most-used move, or None if no history."""
        if not self._counts:
            return None
        return max(self._counts, key=self._counts.get)

    def should_recall(self, current_index: int, total_moves: int) -> bool:
        """Check if it's a good time to insert a signature move recall.

        Returns True if enough moves have passed since the last recall
        and we're not at the very beginning or end of the sequence.
        """
        if current_index < 3:  # too early
            return False
        if current_index >= total_moves - 2:  # too late
            return False
        return (current_index - self._last_recall_index) >= self._recall_interval

    def mark_recalled(self, index: int) -> None:
        """Mark that a recall was inserted at the given index."""
        self._last_recall_index = index

    def get_variation(self, move_name: str, available_moves: list[str]) -> str:
        """Get a variation of a move (same category if possible).

        For the "repeat but variate" concept — same move but the platform
        may execute it with different energy/texture.
        """
        return move_name  # simplest variation: same move, different context


# ── Sequence Runner ───────────────────────────────────────────────────────────

def run_sequence(action_client, sequence: Sequence, stage_tracker=None) -> None:
    """Execute a Sequence by sending goals to the DanceActionServer.

    Iterates through every Phrase and Motif in order. For each Motif:
    - Expands any modifier spec into a list of (move_name, gap_before, energy, texture) tuples.
    - Sleeps gap_before seconds before sending each goal.
    - Calls action_client.send_goal_and_wait() with move name, energy, and texture.

    After each Phrase, sleeps phrase.gap_after seconds.

    Args:
        action_client: An instance of DanceActionClient (or any object with
                       a .send_goal_and_wait(move_name, energy, texture) method).
                       For backward compatibility, also accepts clients with
                       .send_goal_and_wait(move_name) signature.
        sequence: A populated Sequence dataclass.
    """
    beat = 60.0 / max(1.0, sequence.tempo_bpm)
    memory = MotifMemory()

    # Count total moves for motif memory recall timing
    total_moves = sum(len(p.motifs) for p in sequence.phrases)
    move_index = 0

    def _expand(motif: Motif) -> list:
        """Expand one Motif into a list of (move_name, params, gap_before, energy, texture) tuples."""
        mod = motif.modifier
        base_gap = motif.gap_before
        energy = motif.energy
        texture = motif.texture
        params = motif.params

        if mod is None:
            return [(motif.move, params, base_gap, energy, texture)]

        mtype = mod.get("type", "")

        if mtype == "repeat":
            n = mod.get("n", 1)
            gap = mod.get("gap", 0.0)
            return [(motif.move, params, base_gap if i == 0 else gap, energy, texture) for i in range(n)]

        if mtype == "mirror":
            gap = mod.get("gap", 0.0)
            left_params = {**params, "side": "left"}
            right_params = {**params, "side": "right"}
            return [(motif.move, left_params, base_gap, energy, texture),
                    (motif.move, right_params, gap, energy, texture)]

        if mtype == "decay":
            n = mod.get("n", 3)
            factor = mod.get("factor", 0.7)
            pairs = []
            gap = base_gap
            for i in range(n):
                pairs.append((motif.move, params, max(0.0, gap), energy, texture))
                gap += beat * (1.0 - factor)
            return pairs

        if mtype == "crescendo":
            n = mod.get("n", 3)
            factor = mod.get("factor", 1.3)
            pairs = []
            gap = base_gap + beat * (n - 1) * (factor - 1.0) * 0.5
            for i in range(n):
                pairs.append((motif.move, params, max(0.0, gap), energy, texture))
                gap -= beat * (factor - 1.0)
            return pairs

        if mtype == "tension":
            hold = mod.get("hold_duration", 1.0)
            return [(motif.move, params, base_gap + hold, energy, texture)]

        if mtype == "alternate":
            other = mod.get("other_move", motif.move)
            other_params = mod.get("other_params", {})
            n = mod.get("n", 2)
            gap = mod.get("gap", 0.0)
            pairs = []
            for i in range(n):
                pairs.append((motif.move, params, base_gap if i == 0 else gap, energy, texture))
                pairs.append((other, other_params, gap, energy, texture))
            return pairs

        if mtype == "asymmetric_pause":
            short = mod.get("short", 0.2)
            long_ = mod.get("long", 0.8)
            n = mod.get("n", 2)
            pairs = []
            for i in range(n):
                pause = short if i % 2 == 0 else long_
                pairs.append((motif.move, params, base_gap if i == 0 else pause, energy, texture))
            return pairs

        # Unknown modifier type — fall back to bare move
        return [(motif.move, params, base_gap, energy, texture)]

    def _send_goal(move_name, params=None, energy=0.5, texture="neutral"):
        """Send goal with params support."""
        action_client.send_goal_and_wait(move_name, energy, texture,
                                         params=params or {})

    for phrase in sequence.phrases:
        if phrase.name:
            intent_str = f" — {phrase.intent}" if phrase.intent else ""
            print(f"[Choreographer] Phrase: {phrase.name}{intent_str}")
        for motif in phrase.motifs:
            expanded = _expand(motif)
            for move_name, params, gap, energy, texture in expanded:
                if gap > 0.0:
                    time.sleep(gap)
                params_str = f" {params}" if params else ""
                if motif.annotation:
                    print(f"[Choreographer]   {move_name}{params_str} (e={energy:.1f} t={texture})  # {motif.annotation}")
                else:
                    print(f"[Choreographer]   {move_name}{params_str} (e={energy:.1f} t={texture})")

                _send_goal(move_name, params, energy, texture)
                memory.record(move_name)
                move_index += 1

                if stage_tracker is not None:
                    stage_tracker.log_position(label=move_name)

        if phrase.gap_after > 0.0:
            time.sleep(phrase.gap_after)


# ── AI Choreographer ──────────────────────────────────────────────────────────

class AIChoreographer:
    """Generate Sequence objects from natural language using the Google Gemini API.

    Platform-aware: queries the loaded RobotPlatform for available moves
    and capabilities, so it generates choreography appropriate for any robot.

    Args:
        api_key (str): Google Gemini API key.
        platform: Optional RobotPlatform instance. If provided, the AI uses
                  the platform's available moves and description. If None,
                  falls back to the default differential drive move set.
        model (str): Gemini model ID.
    """

    # Fallback move list when no platform is provided
    DEFAULT_MOVES = [
        "Glance", "Bow",
        "Step", "Glide",
        "Shimmy", "Pulse", "Vibrate",
        "Tap", "Pirouette",
        "Spin",
        "Zigzag", "Slalom", "WagWalk",
        "Arc", "TeacupSpin", "TeacupCircle",
        "Spiral", "FigureEight", "Flower",
    ]

    MODIFIER_SCHEMA = """\
modifier (optional, null for no modifier):
  { "type": "repeat",    "n": <int>,   "gap": <float> }
  { "type": "mirror",    "gap": <float> }
  { "type": "decay",     "n": <int>,   "factor": <float 0-1> }
  { "type": "crescendo", "n": <int>,   "factor": <float >1> }
  { "type": "tension",   "hold_duration": <float> }
  { "type": "alternate", "other_move": "<move_name>", "n": <int>, "gap": <float> }
  { "type": "asymmetric_pause", "short": <float>, "long": <float>, "n": <int> }\
"""

    TEXTURE_OPTIONS = "neutral, honey, staccato, ice, cloud, magnet"

    SYSTEM_PROMPT_TEMPLATE = """\
You are a choreographer for a dance robot. \
Generate dance sequences as strict JSON — no prose, no markdown fences.

━━ Robot platform ━━
{platform_description}

━━ Available moves ━━
{move_list}

━━ Expressive parameters ━━
energy      float 0–1   0=restrained/slow, 0.5=normal, 1=explosive/fast
texture     string      one of: {texture_options}
            neutral  = balanced acceleration
            honey    = viscous, high damping, slow onset, smooth
            staccato = percussive, instant torque, abrupt stops
            ice      = frictionless, minimal damping, gliding
            cloud    = floaty, gentle, airy, exponential curves
            magnet   = push-pull attraction/repulsion dynamics
gap_before  float [s]   pause before this move (0=bound/staccato, 0.8=free/suspended)

━━ Modifiers ━━
{modifier_schema}

━━ Output JSON schema ━━
{{
  "title": "<string>",
  "tempo_bpm": <number 60–160>,
  "mood": "<string>",
  "description": "<string>",
  "phrases": [
    {{
      "name": "<string>",
      "intent": "<string — WHY this phrase exists>",
      "gap_after": <float>,
      "motifs": [
        {{
          "move": "<MOVE_NAME>",
          "params": {{<move-specific params or empty object>}},
          "energy": <float>,
          "texture": "<texture_name>",
          "gap_before": <float>,
          "modifier": <modifier object or null>,
          "annotation": "<string>"
        }}
      ]
    }}
  ]
}}

━━ Move parameters ━━
Moves accept an optional "params" object with move-specific arguments:
  Step:         {{"direction": "forward"|"backward"}}
  Glide:        {{"direction": "forward"|"backward"}}
  Spin:         {{"angle": <degrees, positive=CCW, negative=CW. e.g. 90, -180, 360>}}
  Arc:          {{"direction": "left"|"right", "angle": <degrees, default 180>}}
  Tap:          {{"side": "left"|"right"}}
  Pirouette:    {{"side": "left"|"right"}}
  Zigzag:       {{"direction": "forward"|"backward"}}
  Slalom:       {{"direction": "forward"|"backward"}}
  TeacupSpin:   {{"side": "left"|"right"}}
  TeacupCircle: {{"direction": "left"|"right"}}
  Spiral:       {{"direction": "left"|"right"}}
  Glance, Bow, Shimmy, Pulse, Vibrate, WagWalk, FigureEight, Flower: no params needed (use {{}})

━━ Dancer design thinking guidelines ━━
1. Organise into 3–5 named phrases (intro, buildup, climax, wind-down, outro).
2. Each phrase needs an "intent" — the reason behind the movements.
   Think about WHY: the music, the space, the previous movement, the inner emotion.
3. Use texture to create movement quality:
   — honey for flowing, sustained passages
   — staccato for sharp, rhythmic accents
   — cloud for gentle, floating sections
   — ice for smooth gliding transitions
4. Use the power of the pause: the most intense moment is often stillness.
   Insert asymmetric pauses — they let the audience see the robot "deciding."
5. Repetition + Deviation (A-A-A-B): repeat to build meaning, then break
   the pattern on the 4th cycle to create surprise.
6. Use tension modifier before dramatic or climactic moves.
7. Use crescendo for a series that builds to a peak.
8. Use decay for a series that winds down.
9. Vary energy AND texture together for expressive contrast:
   — low energy + honey = contemplative
   — high energy + staccato = explosive
   — low energy + cloud = ethereal
10. Never invent move names — only use the list above.
11. Budget for ~3–8 seconds per move. Total sequence: 30–120 s unless specified.
12. Keep annotations short (under 50 chars). Do NOT embed position tracking in annotations.
13. Return ONLY the JSON object. No markdown fences, no extra text.
{stage_constraints}\
"""

    def __init__(self, api_key: str, platform=None, model: str = "gemini-2.5-flash",
                 stage_width: float = 0.0, stage_depth: float = 0.0):
        try:
            from google import genai
        except ImportError as exc:
            raise ImportError(
                "The 'google-genai' package is required for AIChoreographer. "
                "Install it with: pip install google-genai"
            ) from exc
        self._client = genai.Client(api_key=api_key)
        self._model = model
        self._platform = platform
        self._stage_width = stage_width
        self._stage_depth = stage_depth

    def _get_available_moves(self) -> list[str]:
        if self._platform is not None:
            return self._platform.get_available_moves()
        return self.DEFAULT_MOVES

    def _get_move_list_str(self) -> str:
        if self._platform is not None and hasattr(self._platform, 'get_move_categories'):
            categories = self._platform.get_move_categories()
            lines = []
            for cat, moves in categories.items():
                lines.append(f"{cat.upper():12s}: {', '.join(moves)}")
            return "\n".join(lines)
        # Fallback: flat list
        return ", ".join(self._get_available_moves())

    def _get_platform_description(self) -> str:
        if self._platform is not None:
            return self._platform.get_platform_description()
        return (
            "A differential-drive dance robot with two wheels. "
            "It can move forward/backward, spin in place, pivot around one wheel, "
            "and drive arcs, spirals, figure-eights, and flower curves."
        )

    def _get_displacement_str(self) -> str:
        """Format per-move displacement info for the AI prompt."""
        if self._platform is None or not hasattr(self._platform, 'get_move_displacements'):
            return ""
        displacements = self._platform.get_move_displacements()
        lines = []
        for name, d in displacements.items():
            parts = []
            if d.get("dx", 0) != 0:
                parts.append(f"fwd={d['dx']:+.1f}m")
            if d.get("dy", 0) != 0:
                parts.append(f"lat={d['dy']:+.1f}m")
            if d.get("dtheta", 0) != 0:
                deg = math.degrees(d["dtheta"])
                parts.append(f"turn={deg:+.0f}deg")
            if d.get("radius", 0) > 0:
                parts.append(f"sweep={d['radius']:.1f}m")
            if d.get("returns", False):
                parts.append("returns-to-start")
            if parts:
                lines.append(f"  {name:28s} {', '.join(parts)}")
            else:
                lines.append(f"  {name:28s} in-place, no displacement")
        return "\n".join(lines)

    def _get_stage_constraints_str(self) -> str:
        """Build the stage boundary section for the system prompt."""
        if self._stage_width <= 0 and self._stage_depth <= 0:
            return ""

        displacement_table = self._get_displacement_str()

        parts = ["\n━━ Stage boundary constraints ━━"]
        if self._stage_width > 0 and self._stage_depth > 0:
            parts.append(
                f"Stage size: {self._stage_width:.1f}m wide x {self._stage_depth:.1f}m deep.")
            parts.append(f"The robot starts at stage center (0, 0). Boundaries are:")
            parts.append(f"  Forward/backward: +/- {self._stage_depth / 2:.1f}m")
            parts.append(f"  Left/right:       +/- {self._stage_width / 2:.1f}m")
        elif self._stage_width > 0:
            parts.append(
                f"Stage width: {self._stage_width:.1f}m. "
                f"Left/right: +/- {self._stage_width / 2:.1f}m from center.")
        else:
            parts.append(
                f"Stage depth: {self._stage_depth:.1f}m. "
                f"Forward/backward: +/- {self._stage_depth / 2:.1f}m from center.")

        if displacement_table:
            parts.append("")
            parts.append("Approximate displacement per move (at nominal energy):")
            parts.append(displacement_table)

        parts.append("")
        parts.append("IMPORTANT: Mentally track the robot's cumulative (x, y) position as you "
                      "build the sequence.")
        parts.append("After a heading change (e.g., Spin90CW), forward moves go in the NEW "
                      "heading direction.")
        parts.append("Do not plan moves that would take the robot beyond the stage boundaries.")
        parts.append("Prefer in-place and returning moves when near the edges.")
        parts.append("If unsure, use smaller moves (Step over Glide, Glide over Slalom).")

        return "\n".join(parts)

    def generate(self, description: str, max_retries: int = 2) -> Sequence:
        """Generate a Sequence from a natural language choreography description.

        Args:
            description: Human description of the desired dance.
            max_retries: How many times to ask Gemini to fix invalid JSON.

        Returns:
            A populated Sequence dataclass.

        Raises:
            ValueError: If the response cannot be parsed after max_retries.
        """
        from google.genai import types

        system = self.SYSTEM_PROMPT_TEMPLATE.format(
            platform_description=self._get_platform_description(),
            move_list=self._get_move_list_str(),
            texture_options=self.TEXTURE_OPTIONS,
            modifier_schema=self.MODIFIER_SCHEMA,
            stage_constraints=self._get_stage_constraints_str(),
        )
        contents = [description]

        for attempt in range(max_retries + 1):
            response = self._client.models.generate_content(
                model=self._model,
                contents=contents,
                config=types.GenerateContentConfig(
                    system_instruction=system,
                    max_output_tokens=8192,
                    response_mime_type="application/json",
                    thinking_config=types.ThinkingConfig(thinking_budget=0),
                ),
            )
            raw = response.text.strip()

            try:
                data = self._parse_json(raw)
                return self._dict_to_sequence(data)
            except (json.JSONDecodeError, KeyError, ValueError) as exc:
                if attempt < max_retries:
                    contents.append(raw)
                    contents.append(
                        f"That response was invalid: {exc}. "
                        "Please return only the JSON object, no other text."
                    )
                else:
                    raise ValueError(
                        f"Could not parse AI response after {max_retries} retries: {exc}\n"
                        f"Raw output:\n{raw}"
                    ) from exc

    def _parse_json(self, text: str) -> dict:
        import re
        text = text.strip()
        # Strip markdown fences
        if text.startswith("```"):
            text = text[text.find("\n") + 1:]
            if "```" in text:
                text = text[: text.rfind("```")]
        text = text.strip()
        # Remove trailing commas before } or ] (common LLM mistake)
        text = re.sub(r",\s*([}\]])", r"\1", text)
        return json.loads(text)

    def _dict_to_sequence(self, data: dict) -> Sequence:
        valid = set(self._get_available_moves())
        valid_textures = {"neutral", "honey", "staccato", "ice", "cloud", "magnet"}
        phrases = []
        for ph in data.get("phrases", []):
            motifs = []
            for m in ph.get("motifs", []):
                move = m.get("move", "")
                if move not in valid:
                    raise ValueError(
                        f"Unknown move '{move}'. Valid moves: {sorted(valid)}"
                    )
                texture = str(m.get("texture", "neutral")).lower()
                if texture not in valid_textures:
                    texture = "neutral"
                motifs.append(Motif(
                    move=move,
                    params=m.get("params", {}),
                    energy=float(m.get("energy", 0.5)),
                    texture=texture,
                    gap_before=float(m.get("gap_before", 0.0)),
                    modifier=m.get("modifier"),
                    annotation=str(m.get("annotation", "")),
                ))
            phrases.append(Phrase(
                motifs=motifs,
                name=str(ph.get("name", "")),
                intent=str(ph.get("intent", "")),
                gap_after=float(ph.get("gap_after", 0.0)),
            ))
        return Sequence(
            phrases=phrases,
            title=str(data.get("title", "")),
            tempo_bpm=float(data.get("tempo_bpm", 120.0)),
            mood=str(data.get("mood", "neutral")),
            description=str(data.get("description", "")),
        )

    def describe(self, sequence: Sequence) -> str:
        """Return a human-readable summary of a Sequence for debugging."""
        lines = [
            f"Title: {sequence.title}",
            f"Mood:  {sequence.mood}",
            f"Tempo: {sequence.tempo_bpm} BPM",
            f"Desc:  {sequence.description}",
            "",
        ]
        for phrase in sequence.phrases:
            intent_str = f" — {phrase.intent}" if phrase.intent else ""
            lines.append(f"[{phrase.name}]{intent_str}")
            for motif in phrase.motifs:
                mod_str = f"  mod={motif.modifier}" if motif.modifier else ""
                note = f"  # {motif.annotation}" if motif.annotation else ""
                lines.append(
                    f"  {motif.move:<28} energy={motif.energy:.1f}"
                    f"  texture={motif.texture}"
                    f"  gap={motif.gap_before:.1f}s{mod_str}{note}"
                )
            if phrase.gap_after:
                lines.append(f"  ... pause {phrase.gap_after:.1f}s")
            lines.append("")
        return "\n".join(lines)


# ── CLI entry point ───────────────────────────────────────────────────────────

def main():
    """CLI: generate a choreography from a text description.

    Usage (after building the package):
        ros2 run dance_manager choreographer "An excited celebratory dance"

    Set the GOOGLE_API_KEY environment variable before running, or edit
    the api_key argument below.

    Pass --dry-run to print the plan without executing on the robot.
    """
    import os
    import sys
    import argparse

    parser = argparse.ArgumentParser(description="AI Dance Choreographer")
    parser.add_argument("description", nargs="?",
                        default="A playful, curious exploration with a joyful finale.",
                        help="Natural language choreography description")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the generated sequence without executing")
    parser.add_argument("--model", default="gemini-2.5-flash",
                        help="Gemini model ID")
    parser.add_argument("--stage-width", type=float, default=0.0,
                        help="Stage width [m]. 0 = no boundary constraint.")
    parser.add_argument("--stage-depth", type=float, default=0.0,
                        help="Stage depth [m]. 0 = no boundary constraint.")
    args = parser.parse_args()

    api_key = os.environ.get("GOOGLE_API_KEY", "")
    if not api_key:
        print("Error: set the GOOGLE_API_KEY environment variable.", file=sys.stderr)
        sys.exit(1)

    if args.stage_width > 0 or args.stage_depth > 0:
        print(f"Stage bounds: {args.stage_width}m wide x {args.stage_depth}m deep")

    print(f"Generating choreography for: {args.description!r}")
    ai = AIChoreographer(api_key=api_key, model=args.model,
                         stage_width=args.stage_width, stage_depth=args.stage_depth)
    sequence = ai.generate(args.description)

    print("\n" + ai.describe(sequence))

    if args.dry_run:
        print("[dry-run] Skipping robot execution.")
        return

    import rclpy
    from dance_manager.dance_client import DanceActionClient

    rclpy.init()
    client = DanceActionClient()

    stage_tracker = None
    if args.stage_width > 0 or args.stage_depth > 0:
        from dance_manager.stage_tracker import StageTracker
        stage_tracker = StageTracker(
            client, stage_width=args.stage_width, stage_depth=args.stage_depth)

    try:
        run_sequence(client, sequence, stage_tracker=stage_tracker)
    finally:
        if stage_tracker:
            stage_tracker.destroy()
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
