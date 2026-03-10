"""
Layer 3 — Choreographic Structure

Provides high-level data structures (Motif, Phrase, Sequence) and an
AI-powered choreographer that generates full sequences from natural language
descriptions using the Anthropic Claude API.

Architecture
------------
Motif    : atomic unit — one named move + optional modifier spec + energy
Phrase   : ordered list of Motifs with a name and optional trailing pause
Sequence : ordered list of Phrases with metadata (tempo, mood, title)

Workflow
--------
1. Programmatic: build Motif / Phrase / Sequence manually, call run_sequence().
2. AI-driven: instantiate AIChoreographer, call .generate(description),
   receive a Sequence, then call run_sequence().

The run_sequence() function sends goals to the existing DanceActionServer via
the DanceActionClient.send_goal_and_wait() method. No changes to the server
or the action interface are needed.

Usage — programmatic
---------------------
    from dance_manager.choreographer import Motif, Phrase, Sequence, run_sequence
    from dance_manager.movement_modifiers import seq_repeat, seq_crescendo

    sequence = Sequence(
        title="Simple Groove",
        tempo_bpm=120.0,
        mood="playful",
        phrases=[
            Phrase(name="intro", motifs=[
                Motif("Greeting", energy=0.6),
                Motif("WagWalk", energy=0.5, gap_before=0.3),
            ]),
            Phrase(name="climax", gap_after=1.0, motifs=[
                Motif("TapOnRight", modifier=seq_crescendo(n=4), annotation="build up"),
                Motif("SpinCounterClockwise", energy=0.9),
            ]),
            Phrase(name="outro", motifs=[
                Motif("Bow", energy=0.4),
            ]),
        ],
    )
    run_sequence(action_client, sequence)

Usage — AI choreography
------------------------
    import rclpy
    from dance_manager.dance_client import DanceActionClient
    from dance_manager.choreographer import AIChoreographer, run_sequence

    rclpy.init()
    client = DanceActionClient()

    ai = AIChoreographer(api_key="sk-ant-...")
    sequence = ai.generate(
        "A curious, exploratory dance: start cautious, then grow more playful "
        "and energetic, end with a graceful bow."
    )
    run_sequence(client, sequence)

    client.destroy_node()
    rclpy.shutdown()
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Optional


# ── Data Structures ───────────────────────────────────────────────────────────

@dataclass
class Motif:
    """The atomic unit of choreography: one named move with optional modifiers.

    Attributes:
        move: A move name recognised by the DanceActionServer (e.g. "TapOnRight").
        energy: Expressive intensity [0.0–1.0]. Used by run_sequence to scale
                inter-move gaps (lighter energy → more breathing room before).
        gap_before: Explicit pause [s] inserted before sending this goal.
                    Overrides flow-based gap calculations when > 0.
        modifier: Optional dict produced by a seq_* helper in movement_modifiers
                  (e.g. seq_repeat(4, gap=0.2)). Expanded by run_sequence.
        annotation: Human-readable note shown in logs. Has no effect on motion.
    """
    move: str
    energy: float = 0.5
    gap_before: float = 0.0
    modifier: Optional[dict] = None
    annotation: str = ""


@dataclass
class Phrase:
    """An ordered list of Motifs forming a musical phrase or section.

    Attributes:
        motifs: Ordered sequence of Motif objects.
        name: Human-readable label (e.g. 'intro', 'chorus', 'bridge').
        gap_after: Seconds of stillness appended after the final motif.
    """
    motifs: list = field(default_factory=list)
    name: str = ""
    gap_after: float = 0.0


@dataclass
class Sequence:
    """A complete choreographic sequence with metadata.

    Attributes:
        phrases: Ordered list of Phrase objects.
        title: Human-readable title of the dance.
        tempo_bpm: Musical tempo used to scale gap timing globally.
                   Higher BPM → shorter beat duration → tighter gaps.
        mood: Single expressive descriptor (e.g. 'playful', 'dramatic', 'dreamy').
        description: Free-text summary of the choreographic intention.
    """
    phrases: list = field(default_factory=list)
    title: str = ""
    tempo_bpm: float = 120.0
    mood: str = "neutral"
    description: str = ""


# ── Sequence Runner ───────────────────────────────────────────────────────────

def run_sequence(action_client, sequence: Sequence) -> None:
    """Execute a Sequence by sending goals to the DanceActionServer.

    Iterates through every Phrase and Motif in order. For each Motif:
    - Expands any modifier spec into a list of (move_name, gap_before) pairs.
    - Sleeps gap_before seconds before sending each goal.
    - Calls action_client.send_goal_and_wait(move_name) and blocks until done.

    After each Phrase, sleeps phrase.gap_after seconds.

    Args:
        action_client: An instance of DanceActionClient (or any object with
                       a .send_goal_and_wait(move_name: str) method).
        sequence: A populated Sequence dataclass.
    """
    beat = 60.0 / max(1.0, sequence.tempo_bpm)

    def _expand(motif: Motif) -> list:
        """Expand one Motif into a list of (move_name, gap_before) pairs."""
        mod = motif.modifier
        base_gap = motif.gap_before

        if mod is None:
            return [(motif.move, base_gap)]

        mtype = mod.get("type", "")

        if mtype == "repeat":
            n = mod.get("n", 1)
            gap = mod.get("gap", 0.0)
            return [(motif.move, base_gap if i == 0 else gap) for i in range(n)]

        if mtype == "mirror":
            gap = mod.get("gap", 0.0)
            left_name = _mirror_name(motif.move, to="left")
            right_name = _mirror_name(motif.move, to="right")
            return [(left_name, base_gap), (right_name, gap)]

        if mtype == "decay":
            n = mod.get("n", 3)
            factor = mod.get("factor", 0.7)
            # Model decay as increasing pre-move gap (energy fades → more rest)
            pairs = []
            gap = base_gap
            for i in range(n):
                pairs.append((motif.move, max(0.0, gap)))
                gap += beat * (1.0 - factor)
            return pairs

        if mtype == "crescendo":
            n = mod.get("n", 3)
            factor = mod.get("factor", 1.3)
            # Model crescendo as shrinking pre-move gap (urgency builds)
            pairs = []
            gap = base_gap + beat * (n - 1) * (factor - 1.0) * 0.5
            for i in range(n):
                pairs.append((motif.move, max(0.0, gap)))
                gap -= beat * (factor - 1.0)
            return pairs

        if mtype == "tension":
            hold = mod.get("hold_duration", 1.0)
            return [(motif.move, base_gap + hold)]

        if mtype == "alternate":
            other = mod.get("other_move", motif.move)
            n = mod.get("n", 2)
            gap = mod.get("gap", 0.0)
            pairs = []
            for i in range(n):
                pairs.append((motif.move, base_gap if i == 0 else gap))
                pairs.append((other, gap))
            return pairs

        # Unknown modifier type — fall back to bare move
        return [(motif.move, base_gap)]

    for phrase in sequence.phrases:
        if phrase.name:
            print(f"[Choreographer] Phrase: {phrase.name}")
        for motif in phrase.motifs:
            expanded = _expand(motif)
            for move_name, gap in expanded:
                if gap > 0.0:
                    time.sleep(gap)
                if motif.annotation:
                    print(f"[Choreographer]   {move_name}  # {motif.annotation}")
                else:
                    print(f"[Choreographer]   {move_name}")
                action_client.send_goal_and_wait(move_name)
        if phrase.gap_after > 0.0:
            time.sleep(phrase.gap_after)


def _mirror_name(move_name: str, to: str) -> str:
    """Replace Left/Right suffix in a move name with the requested side.

    Examples:
        _mirror_name("PirouetteLeft",  to="right") → "PirouetteRight"
        _mirror_name("PirouetteRight", to="left")  → "PirouetteLeft"
        _mirror_name("SlalomForward",  to="right") → "SlalomForward"  (unchanged)
    """
    cap = to.capitalize()
    opposite = "Right" if cap == "Left" else "Left"
    if move_name.endswith(opposite):
        return move_name[: -len(opposite)] + cap
    if move_name.endswith(cap):
        return move_name  # already correct side
    return move_name  # no side suffix found


# ── AI Choreographer ──────────────────────────────────────────────────────────

class AIChoreographer:
    """Generate Sequence objects from natural language using the Claude API.

    Requires the `anthropic` package: pip install anthropic

    Args:
        api_key (str): Anthropic API key (sk-ant-...).
        model (str): Claude model ID. Defaults to claude-opus-4-5.
    """

    # All move names registered in dance_server.py
    AVAILABLE_MOVES = [
        # Social gestures
        "Greeting", "PeekLeftRight", "Bow",
        # Linear steps
        "InchForward", "StepForward", "RollForward",
        "InchBackward", "StepBackward",
        "GlideForward", "GlideBackward",
        # Expressive in-place
        "Shimmy", "ShimmyFast", "Pulse", "Vibrate",
        # Pivots and taps
        "TapOnLeft", "TapOnRight",
        "PirouetteLeft", "PirouetteRight",
        # Axis spins
        "SpinClockwise", "SpinCounterClockwise",
        "Spin180CW", "Spin180CCW",
        "Spin90CW", "Spin90CCW",
        "Spin15CW", "Spin15CCW",
        # Weaving paths
        "ZigZaggingForward", "ZigZaggingBackward",
        "SlalomForward", "SlalomBackward",
        "WagWalk",
        # Arc / circle patterns
        "ArcLeft", "ArcRight",
        "TeacupSpinLeft", "TeacupSpinRight",
        "TeacupCircleLeft", "TeacupCircleRight",
        # Complex paths
        "SpiralLeft", "SpiralRight",
        "FigureEight", "FlowerDance",
    ]

    MODIFIER_SCHEMA = """\
modifier (optional, null for no modifier):
  { "type": "repeat",    "n": <int>,   "gap": <float> }
  { "type": "mirror",    "gap": <float> }
  { "type": "decay",     "n": <int>,   "factor": <float 0-1> }
  { "type": "crescendo", "n": <int>,   "factor": <float >1> }
  { "type": "tension",   "hold_duration": <float> }
  { "type": "alternate", "other_move": "<move_name>", "n": <int>, "gap": <float> }\
"""

    SYSTEM_PROMPT = """\
You are a choreographer for a differential-drive dance robot. \
Generate dance sequences as strict JSON — no prose, no markdown fences.

━━ Robot capabilities ━━
The robot has two wheels. It can move forward/backward, spin in place, \
pivot around one wheel, and drive arcs, spirals, figure-eights, and flower curves.

━━ Available moves ━━
SOCIAL:     Greeting, Bow, PeekLeftRight
FORWARD:    InchForward, StepForward, RollForward, WagWalk, SlalomForward, ZigZaggingForward
BACKWARD:   InchBackward, StepBackward, SlalomBackward, ZigZaggingBackward
SPIN:       SpinClockwise, SpinCounterClockwise, Spin180CW, Spin180CCW,
            Spin90CW, Spin90CCW, Spin15CW, Spin15CCW
PIVOT:      PirouetteLeft, PirouetteRight, TeacupSpinLeft, TeacupSpinRight
ARC:        TeacupCircleLeft, TeacupCircleRight
COMPLEX:    SpiralLeft, SpiralRight, FigureEight, FlowerDance
PERCUSSIVE: TapOnLeft, TapOnRight

━━ Expressive parameters ━━
energy      float 0–1   0=restrained/slow, 0.5=normal, 1=explosive/fast
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
      "gap_after": <float>,
      "motifs": [
        {{
          "move": "<MOVE_NAME>",
          "energy": <float>,
          "gap_before": <float>,
          "modifier": <modifier object or null>,
          "annotation": "<string>"
        }}
      ]
    }}
  ]
}}

━━ Choreography guidelines ━━
1. Organise into 3–5 named phrases (intro, buildup, climax, wind-down, outro).
2. Match move character to mood:
   — spinning moves  → excitement, dizziness, joy
   — slalom / wag   → playfulness, curiosity
   — bow / greeting → social, finale
   — spiral / flower → grace, contemplation
3. Use tension modifier before dramatic or climactic moves.
4. Use crescendo for a series that builds to a peak.
5. Use decay for a series that winds down.
6. Use alternate for call-and-response (e.g. TapOnLeft ↔ TapOnRight).
7. Use mirror for symmetric bilateral moves (e.g. PirouetteLeft + PirouetteRight).
8. Never invent move names — only use the list above.
9. Budget for ~3–8 seconds per move. Total sequence: 30–120 s unless specified.
10. Return ONLY the JSON object. No extra text.\
"""

    def __init__(self, api_key: str, model: str = "claude-opus-4-5"):
        try:
            import anthropic
        except ImportError as exc:
            raise ImportError(
                "The 'anthropic' package is required for AIChoreographer. "
                "Install it with: pip install anthropic"
            ) from exc
        self._client = anthropic.Anthropic(api_key=api_key)
        self._model = model

    def generate(self, description: str, max_retries: int = 2) -> Sequence:
        """Generate a Sequence from a natural language choreography description.

        Calls the Claude API, parses the JSON response, validates move names,
        and returns a Sequence ready for run_sequence().

        Args:
            description: Human description of the desired dance (mood, tempo,
                         narrative arc, style, etc.).
            max_retries: How many times to ask Claude to fix invalid JSON.

        Returns:
            A populated Sequence dataclass.

        Raises:
            ValueError: If the response cannot be parsed after max_retries.
        """
        system = self.SYSTEM_PROMPT.format(modifier_schema=self.MODIFIER_SCHEMA)
        messages = [{"role": "user", "content": description}]

        for attempt in range(max_retries + 1):
            response = self._client.messages.create(
                model=self._model,
                max_tokens=4096,
                system=system,
                messages=messages,
            )
            raw = response.content[0].text.strip()

            try:
                data = self._parse_json(raw)
                return self._dict_to_sequence(data)
            except (json.JSONDecodeError, KeyError, ValueError) as exc:
                if attempt < max_retries:
                    messages.append({"role": "assistant", "content": raw})
                    messages.append({
                        "role": "user",
                        "content": (
                            f"That response was invalid: {exc}. "
                            "Please return only the JSON object, no other text."
                        ),
                    })
                else:
                    raise ValueError(
                        f"Could not parse AI response after {max_retries} retries: {exc}\n"
                        f"Raw output:\n{raw}"
                    ) from exc

    def _parse_json(self, text: str) -> dict:
        """Strip optional markdown fences and parse JSON."""
        text = text.strip()
        if text.startswith("```"):
            # Remove opening fence line
            text = text[text.find("\n") + 1:]
            # Remove closing fence
            if "```" in text:
                text = text[: text.rfind("```")]
        return json.loads(text.strip())

    def _dict_to_sequence(self, data: dict) -> Sequence:
        """Convert a parsed dict into a Sequence, validating move names."""
        valid = set(self.AVAILABLE_MOVES)
        phrases = []
        for ph in data.get("phrases", []):
            motifs = []
            for m in ph.get("motifs", []):
                move = m.get("move", "")
                if move not in valid:
                    raise ValueError(
                        f"Unknown move '{move}'. "
                        f"Valid moves: {sorted(valid)}"
                    )
                motifs.append(Motif(
                    move=move,
                    energy=float(m.get("energy", 0.5)),
                    gap_before=float(m.get("gap_before", 0.0)),
                    modifier=m.get("modifier"),
                    annotation=str(m.get("annotation", "")),
                ))
            phrases.append(Phrase(
                motifs=motifs,
                name=str(ph.get("name", "")),
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
            lines.append(f"[{phrase.name}]")
            for motif in phrase.motifs:
                mod_str = f"  mod={motif.modifier}" if motif.modifier else ""
                note = f"  # {motif.annotation}" if motif.annotation else ""
                lines.append(
                    f"  {motif.move:<28} energy={motif.energy:.1f}"
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

    Set the ANTHROPIC_API_KEY environment variable before running, or edit
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
    parser.add_argument("--model", default="claude-opus-4-5",
                        help="Claude model ID")
    args = parser.parse_args()

    api_key = os.environ.get("ANTHROPIC_API_KEY", "")
    if not api_key:
        print("Error: set the ANTHROPIC_API_KEY environment variable.", file=sys.stderr)
        sys.exit(1)

    print(f"Generating choreography for: {args.description!r}")
    ai = AIChoreographer(api_key=api_key, model=args.model)
    sequence = ai.generate(args.description)

    print("\n" + ai.describe(sequence))

    if args.dry_run:
        print("[dry-run] Skipping robot execution.")
        return

    import rclpy
    from dance_manager.dance_client import DanceActionClient

    rclpy.init()
    client = DanceActionClient()
    try:
        run_sequence(client, sequence)
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
