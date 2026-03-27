# Simulation (Gazebo)

Run the chair robot in simulation without any hardware. Useful for testing choreography and developing new moves.

## Prerequisites

```bash
# ROS 2 Humble + Gazebo packages
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-twist-mux
```

## Build

```bash
cd ~/dancerobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dance_interfaces dance_manager chair_dance_sim
source install/setup.bash
```

#### It is important that you source in every terminal you open:
```bash
source /opt/ros/humble/setup.bash
source ~/dancerobot_ws/install/setup.bash
```

## View the Robot Model (RViz only, no physics)

Quick sanity check to verify the URDF looks correct:

```bash
ros2 launch chair_dance_sim view_robot_launch.py
```

## Start Dance Server

In a terminal, start the dance server first:
```bash
ros2 launch dance_manager dance_server_launch.py
```

## Run Full Simulation

In a separate terminal:

**Option A — With Gazebo GUI:**
```bash
ros2 launch chair_dance_sim simulation_launch.py
```

**Option B — Headless + RViz (recommended for WSL):**

If the Gazebo GUI crashes (common on WSL due to OpenGL), run the physics server only and visualize in RViz:

```bash
ros2 launch chair_dance_sim simulation_launch.py headless:=true use_rviz:=true
```

**Option C — Gazebo GUI with software rendering (WSL fallback):**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch chair_dance_sim simulation_launch.py
```

## Send Dance Commands

In a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/dancerobot_ws/install/setup.bash

# Single move
ros2 action send_goal /dance dance_interfaces/action/Dance \
    "{dance_move: 'SpinOnAxis'}"

# Run the choreographer
ros2 run dance_manager choreographer
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `headless` | `false` | Run Gazebo server only (no GUI) |
| `use_rviz` | `false` | Launch RViz2 for visualization |
| `platform` | `differential_drive` | Robot platform for the dance server |
