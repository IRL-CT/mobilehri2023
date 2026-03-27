# Robot Choreography System — Refactoring Plan

## Goal
Refactor the dance system into a **generalizable, modular platform** that cleanly separates:
- **High-level choreography** (platform-agnostic planning, structure, expression)
- **Low-level control** (robot-specific move implementations)

Grounded in the dancer design thinking from the Notion page: "From Dancer to Robot."

---

## Current State

| File | Role | Platform Coupling |
|------|------|-------------------|
| `choreographer.py` | Sequence/Phrase/Motif + AI generation + `run_sequence()` | **None** (clean) |
| `dance_moves.py` | 35+ move primitives publishing `Twist` directly | **Tight** (diff-drive) |
| `dance_server.py` | Action server mapping move names → functions | **Tight** (hardcoded dispatch) |
| `movement_modifiers.py` | `seq_*` helpers (abstract) + `twist_pub` wrappers (coupled) | **Mixed** |
| `Dance.action` | `string dance_move` → `int32 result_code` | Minimal |

**Key insight**: `choreographer.py` is already platform-agnostic. The tight coupling only lives in `dance_moves.py` and `dance_server.py`. The refactor targets those two files.

---

## Architecture After Refactoring

```
┌──────────────────────────────────────────────────────────┐
│           CHOREOGRAPHY LAYER (platform-agnostic)         │
│                                                          │
│  choreographer.py  (enhanced)                            │
│  ├─ Sequence / Phrase / Motif  (add texture, intent)     │
│  ├─ run_sequence()  (add pre-roll, active brake hooks)   │
│  ├─ AIChoreographer (platform-aware prompt selection)     │
│  └─ Dancer concepts:                                     │
│     • Motif Memory (§4 repeat-but-variate)               │
│     • Repetition + Deviation with decay/growth (§4)      │
│     • Asymmetric Pauses (§4 power of the pause)          │
│     • Texture as abstract intent: "honey"|"staccato"|... │
│     • Energy / Weight as expressive intent               │
├──────────────────────────────────────────────────────────┤
│         PLATFORM ABSTRACTION LAYER (new)                 │
│                                                          │
│  platform.py                                             │
│  ├─ ABC: RobotPlatform                                   │
│  │   ├─ execute_move(name, energy, texture, ...)         │
│  │   ├─ get_available_moves() → list[str]                │
│  │   ├─ pre_roll(move_name)       # §1 breath/prep      │
│  │   ├─ active_brake()            # §3 freeze = tension  │
│  │   ├─ apply_texture(preset)     # §1 env presets       │
│  │   ├─ spotting_anchor(...)      # §3 sensory reset     │
│  │   └─ lean_into_error(residual) # §3 error → variation │
│  │                                                       │
│  └─ Enhanced Dance.action:                               │
│       string dance_move                                  │
│       float32 energy                                     │
│       string texture          # "honey"|"staccato"|...   │
│       ---                                                │
│       int32 result_code                                  │
│       ---                                                │
│       int32 progress                                     │
├──────────────────────────────────────────────────────────┤
│        PLATFORM IMPLEMENTATIONS (robot-specific)         │
│                                                          │
│  platforms/differential_drive.py                         │
│  ├─ DiffDrivePlatform(RobotPlatform)                     │
│  ├─ All current dance_moves.py logic lives here          │
│  ├─ Twist publishing, track width, diff-drive math       │
│  ├─ pre_roll → 100ms micro-vibration or counter-steer    │
│  ├─ active_brake → brief counter-torque then zero        │
│  ├─ texture: honey=high damping, staccato=instant torque │
│  └─ spotting: virtual anchor via IMU/odometry            │
│                                                          │
│  platforms/quadruped.py  (future example)                │
│  platforms/arm.py        (future example)                │
└──────────────────────────────────────────────────────────┘
```

---

## Mapping Dancer Design Thinking → System Concepts

### §1 Foundational Sense of Movement
| Dancer Concept | System Concept | Layer |
|---|---|---|
| **Physicalizing the Breath** (prep micro-movement) | `RobotPlatform.pre_roll()` — each platform implements its own preparatory motion | Platform |
| **Sub-division of Internal Time** (non-linear decel) | Already in `dance_moves.py` via acceleration curves; formalize in platform's velocity profiles | Platform |
| **Movement Texture** (honey, ice, staccato) | `Motif.texture` field (abstract intent) → `RobotPlatform.apply_texture()` translates to motor params | Both |

### §2 Deconstructing & Learning Complex Movements
| Dancer Concept | System Concept | Layer |
|---|---|---|
| **Isolation & Layering** | Compose moves from independent channels (path + expression + dynamics). Platform defines available channels | Platform |
| **Marking** (low-energy rehearsal) | `energy=0.1` in Motif — platform scales all params down. Used for testing/previewing | Choreography |
| **Checkpoint** (key-frame orientations) | `RobotPlatform.checkpoint(orientation)` — platform uses odometry/IMU to hit target orientations at beat boundaries | Platform |

### §3 Maintaining Control Through Instability
| Dancer Concept | System Concept | Layer |
|---|---|---|
| **Spotting** (sensory reset during spins) | `RobotPlatform.spotting_anchor()` — lock front-facing logic onto virtual anchor during spins | Platform |
| **Active Brake** (freeze = tension, not zero) | `RobotPlatform.active_brake()` — brief counter-torque instead of just publishing zero velocity | Platform |
| **Leaning into Error** (error → variation) | `RobotPlatform.lean_into_error()` — when odometry drift detected, let residual momentum become next move's input instead of hard correction | Platform |

### §4 Choreographing Through Structure & Choice
| Dancer Concept | System Concept | Layer |
|---|---|---|
| **Repetition + Deviation (A-A-A-B)** | Enhance `seq_repeat` modifier with `decay`/`growth` factor — already partially in `_expand()` | Choreography |
| **Power of the Pause** | `Motif.gap_before` with asymmetric values; add `seq_asymmetric_pause()` modifier | Choreography |
| **Reason behind movement** | `Motif.annotation` + `Phrase.intent` — drive AI choreographer to think about WHY each move follows | Choreography |
| **Repeat but Variate (Motif Memory)** | New `MotifMemory` class tracks used motifs, can recall "signature moves" at intervals to build personality | Choreography |

---

## Implementation Steps

### Step 1: Create `platform.py` — Abstract Base Class
- Define `RobotPlatform` ABC with all methods
- Define `MoveContext` dataclass (energy, texture, weight, etc.) to pass expressive intent
- No changes to existing files yet

### Step 2: Create `platforms/differential_drive.py`
- Extract all move functions from `dance_moves.py` into `DiffDrivePlatform`
- Each move becomes a method (or registered callable)
- Implement `pre_roll()`, `active_brake()`, `apply_texture()`, `spotting_anchor()`
- Keeps all Twist/diff-drive logic isolated here

### Step 3: Refactor `dance_server.py`
- Accept a `RobotPlatform` instance (configured via ROS param or launch file)
- Replace hardcoded move dictionary with `platform.get_available_moves()` + `platform.execute_move()`
- Server becomes platform-agnostic dispatcher

### Step 4: Enhance `Dance.action` interface
- Add `energy`, `texture` fields to request
- Server passes these to `platform.execute_move()`

### Step 5: Enhance `choreographer.py`
- Add `texture` field to `Motif`
- Add `intent` field to `Phrase`
- Update `run_sequence()` to:
  - Call `platform.pre_roll()` before each move
  - Call `platform.active_brake()` after each move (when appropriate)
  - Pass energy/texture through the action goal
- Add `MotifMemory` class for signature move recall
- Update `AIChoreographer` to be platform-aware (query platform for available moves)

### Step 6: Update `movement_modifiers.py`
- Remove twist_pub-dependent wrappers (moved to platform layer)
- Keep and enhance `seq_*` helpers (they're already abstract)
- Add `seq_asymmetric_pause()` for dancer-style pauses

---

## File Changes Summary

| Action | File |
|--------|------|
| **NEW** | `dance_manager/dance_manager/platform.py` |
| **NEW** | `dance_manager/dance_manager/platforms/__init__.py` |
| **NEW** | `dance_manager/dance_manager/platforms/differential_drive.py` |
| **MODIFY** | `dance_manager/dance_manager/dance_server.py` (platform-agnostic dispatch) |
| **MODIFY** | `dance_manager/dance_manager/choreographer.py` (texture, motif memory, platform hooks) |
| **MODIFY** | `dance_manager/dance_manager/movement_modifiers.py` (remove coupled wrappers) |
| **MODIFY** | `dance_interfaces/action/Dance.action` (add energy, texture fields) |
| **KEEP** | `dance_manager/dance_manager/dance_moves.py` (kept for backward compat, deprecated) |

---

## Key Design Decisions

1. **Backward compatibility**: `dance_moves.py` stays untouched initially. New platform code imports its functions. Old code still works.
2. **Platform selection via ROS param**: `dance_server` takes a `platform` parameter (e.g., `"differential_drive"`) and loads the right implementation.
3. **Texture is abstract intent, not motor params**: Choreographer says `texture="honey"`, platform translates to its own motor characteristics.
4. **Pre-roll and active brake are opt-in hooks**: Platform can return early (no-op) if the concept doesn't apply.
5. **AIChoreographer queries platform capabilities**: Instead of hardcoding `AVAILABLE_MOVES`, it asks the loaded platform what moves it supports.
