# dance_manager — Documentation

This document provides a deeper look at the `dance_manager` package: its purpose, components, example workflows, and development notes.

## Overview

`dance_manager` coordinates dance choreography for the robot. It exposes an action server (based on the `Dance` action in `dance_interfaces`) which clients can use to request dance routines. The manager is responsible for validating goals, scheduling moves, providing feedback, and returning a result when the dance completes or is canceled.

Where to look in the source tree

- `dance_manager/dance_manager/dance_server.py` — main action server logic and orchestration, waiting for the client to send moves.
- `dance_manager/dance_manager/dance_client.py` — helper client code for sending a series of dance moves and handling feedback.
- `dance_manager/dance_manager/dance_moves.py` — pre-defined moves, mostly to keep the workspace organized. Can think of it as the library of dance moves.
- `dance_manager/launch/dance_server_launch.py` — launch file that starts the server node defined in `dance_server.py`.

## Architecture

The dance system uses a three-layer architecture:

```
Choreography Layer    choreographer.py     (platform-agnostic: Sequence, Phrase, Motif)
        │
Platform Abstraction  platform.py          (RobotPlatform ABC, MoveContext)
        │
Platform Implementation  platforms/differential_drive.py  (DiffDrivePlatform)
```

- `dance_server.py` is a **platform-agnostic dispatcher** — it receives action goals and delegates to the loaded `RobotPlatform`. It does NOT contain move logic or registration.
- `dance_moves.py` contains the low-level **motion primitives** (twist publishing loops).
- `platforms/differential_drive.py` contains the **move registry** mapping 19 primitive names to `dance_moves.py` functions, plus displacement estimates, texture profiles, and categories.

## Adding new dance moves

> [Logic]
>
> 1. Implement the motion primitive in `dance_moves.py`
> 2. Register it in `platforms/differential_drive.py` (move registry)
> 3. Test via CLI or the Dance Studio web tool

### 1) Implement the primitive

Open `dance_manager/dance_manager/dance_moves.py` and add a function. Requirements:
- First argument is `twist_pub` (ROS 2 Twist publisher)
- Accept `**kwargs` for the dancer vocabulary system (energy, weight, noise_level)
- Call `_apply_vocab(kwargs, ...)` to resolve energy/weight into speed/timing
- The robot reads `twist.linear.x` (forward velocity) and `twist.angular.z` (turn rate)

Example:

```python
def sway(twist_pub, duration=3.0, amplitude=0.5, frequency=1.0, cmd_dt=0.05, **kwargs):
    """Gentle side-to-side rocking motion."""
    speed_scale, _, _ = _apply_vocab(kwargs, base_speed=amplitude, ramp_up=duration, ramp_down=duration)
    amplitude *= speed_scale

    t = Twist()
    start = time.time()
    while time.time() < start + duration:
        elapsed = time.time() - start
        t.angular.z = amplitude * math.sin(2 * math.pi * frequency * elapsed)
        twist_pub.publish(t)
        time.sleep(cmd_dt)
    _stop(twist_pub)
```

### 2) Register in the platform

Open `platforms/differential_drive.py` and add the move to `_build_move_registry()`:

```python
"Sway": lambda p, **kw: sway(tp, **kw),
```

The first argument `p` is a params dict (for moves that accept direction/side/angle). Pass it through if your move needs parameters, or ignore it for parameterless moves.

Also update:
- `get_move_displacements()` — add displacement estimate for boundary awareness
- `get_move_categories()` — add to the appropriate category

### 3) Test

```bash
cd ~/dancerobot_ws
colcon build --packages-select dance_manager
source install/setup.bash

# Test via CLI
ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"Sway"}'

# Or via the Dance Studio web tool
python dance_studio/app.py
```

### Current primitive moves (19)

| Move | Params | Description |
|------|--------|-------------|
| Glance | — | Look left, right, center |
| Bow | — | Forward bow with shimmy |
| Step | direction | Short translational step |
| Glide | direction | Constant-speed travel |
| Shimmy | — | Angular oscillation in place |
| Pulse | — | Forward-backward beats |
| Vibrate | — | High-frequency trembling |
| Tap | side | Pivot on one wheel |
| Pirouette | side | 360° around one wheel |
| Spin | angle (degrees) | In-place rotation |
| Zigzag | direction | Alternating pivot arcs |
| Slalom | direction | Sine-wave weaving |
| WagWalk | — | Forward with head wagging |
| Arc | direction, angle | Circular arc |
| TeacupSpin | side | 360° with lateral displacement |
| TeacupCircle | direction | Orbit + spin pattern |
| Spiral | direction | Expanding spiral |
| FigureEight | — | Two tangent circles |
| Flower | — | Rose-curve pattern |

3) Test with the action server

 - Start the `dance_server` (launch file provided):

```bash
ros2 launch dance_manager dance_server_launch.py
```

 - Send the move name from the CLI to verify the server selects and runs the move:

```bash
ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"Step", "params":"{\"direction\":\"forward\"}"}'
```

 - Alternatively, use the package's `DanceActionClient` (see `dance_client.py`) to send the same string programmatically and observe logs/feedback.

## AI Choreographer

The `choreographer` module uses Google Gemini to generate full dance sequences from natural language descriptions. It produces structured `Sequence` → `Phrase` → `Motif` hierarchies with expressive parameters (energy, texture, modifiers).

### Prerequisites

```bash
pip install google-genai
export GOOGLE_API_KEY="your-key-here"
```

### CLI usage

```bash
# Generate and execute a choreography
ros2 run dance_manager choreographer "A curious, exploratory dance"

# Preview without executing on the robot
ros2 run dance_manager choreographer --dry-run "An excited celebratory dance"

# Use a specific Gemini model (default: gemini-2.5-flash)
ros2 run dance_manager choreographer --model gemini-2.5-pro "A dramatic performance"
```

### Programmatic usage

```python
import rclpy
from dance_manager.dance_client import DanceActionClient
from dance_manager.choreographer import AIChoreographer, run_sequence

rclpy.init()
client = DanceActionClient()

ai = AIChoreographer(api_key="your-key-here")
sequence = ai.generate("A playful greeting that builds to a spin finale")
print(ai.describe(sequence))  # preview the plan
run_sequence(client, sequence)

client.destroy_node()
rclpy.shutdown()
```

### Building sequences manually

You can also build sequences without AI:

```python
from dance_manager.choreographer import Motif, Phrase, Sequence, run_sequence
from dance_manager.movement_modifiers import seq_crescendo

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
```

### Expressive parameters

| Parameter | Values | Description |
|-----------|--------|-------------|
| `energy` | 0.0–1.0 | Intensity (0=restrained, 0.5=normal, 1=explosive) |
| `texture` | neutral, honey, staccato, ice, cloud, magnet | Movement quality preset |
| `gap_before` | float (seconds) | Pause before the move (0=bound, 0.8=suspended) |

### Modifiers

Modifiers are applied to individual motifs to create patterns:

| Modifier | Effect |
|----------|--------|
| `seq_repeat(n, gap)` | Repeat a move n times |
| `seq_mirror(gap)` | Play left then right variant |
| `seq_decay(n, factor)` | Repeat with increasing gaps (winding down) |
| `seq_crescendo(n, factor)` | Repeat with decreasing gaps (building up) |
| `seq_tension(hold)` | Hold pause before the move |
| `seq_alternate(other, n, gap)` | Alternate between two moves |
| `seq_asymmetric_pause(short, long, n)` | Uneven rhythm pattern |

## Workflow examples

1) Start the server (on a machine with ROS 2 and the workspace sourced):

```bash
$ cd ~/dancerobot_ws/src/mobilehri2023
$ source install/setup.bash
$ ros2 launch dance_manager dance_server_launch.py
# in a seperate terminal, remember to source
$ ros2 launch mobile_robot_control mobile_robot_launch.py
```

2) Send a goal from the CLI (fields depend on the action definition):

```bash
$ ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"Zigzag", "params":"{\"direction\":\"forward\"}"}'
$ ros2 action send_goal /dance dance_interfaces/action/Dance '{"dance_move":"Step", "params":"{\"direction\":\"forward\"}"}'
```

3) Programmatic client example:
If you want to see how it looks like with multiple moves executed sequentially, use `dance_client.py`.
```python
def main(args=None):
    rclpy.init(args=args)

    action_client = DanceActionClient()
    
    # Send goals using the improved method
    print("Sending first move...")
    success1 = action_client.send_goal_and_wait("Step", params={"direction": "forward"})
    print(f"First goal completed: {success1}")

    print("Sending second move...")
    success2 = action_client.send_goal_and_wait("Zigzag", params={"direction": "forward"})
    print(f"second goal completed: {success2}")
    
    action_client.destroy_node()
    rclpy.shutdown()
```

```bash
$ cd ~/dancerobot_ws/src/mobilehri2023/dance_manager/dance_manager
$ source ~/dancerobot_ws/install/setup.bash
$ python3 dance_client.py
```
