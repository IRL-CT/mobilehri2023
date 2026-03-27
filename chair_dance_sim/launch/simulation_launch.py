"""
Full simulation launch — Gazebo + Chair Robot + Dance System

Launches:
  1. Gazebo Classic with the dance stage world
  2. Robot state publisher (URDF → /robot_description, /tf)
  3. Chair robot spawned in Gazebo
  4. Twist mux (routes /dance_manager/cmd_vel → /cmd_vel)
  5. Dance action server (platform=differential_drive)

Usage:
  ros2 launch chair_dance_sim simulation_launch.py

  # Headless mode (WSL — no Gazebo GUI, use RViz instead):
  ros2 launch chair_dance_sim simulation_launch.py headless:=true use_rviz:=true

Then send dance commands:
  ros2 action send_goal /dance dance_interfaces/action/Dance \
      "{dance_move: 'SpinOnAxis', energy: 0.7, texture: 'staccato'}"
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ───────────────────────────────────────────────────
    sim_pkg = get_package_share_directory('chair_dance_sim')

    urdf_file = os.path.join(sim_pkg, 'urdf', 'chair_robot.urdf.xacro')
    world_file = os.path.join(sim_pkg, 'worlds', 'stage.world')
    twist_mux_cfg = os.path.join(sim_pkg, 'config', 'twist_mux.yaml')

    # ── Launch arguments ────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 alongside Gazebo'
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo server only (no GUI) — useful on WSL'
    )

    platform_arg = DeclareLaunchArgument(
        'platform', default_value='differential_drive',
        description='Robot platform for the dance server'
    )

    # ── Process URDF via xacro ──────────────────────────────────────────
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])

    # ── Nodes ───────────────────────────────────────────────────────────

    # 1a. Gazebo with GUI (default)
    gazebo_gui = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    # 1b. Gazebo server only (headless — no Ogre, no crash on WSL)
    gazebo_headless = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    # 2. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
    )

    # 3. Spawn the chair robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_chair_robot',
        output='screen',
        arguments=[
            '-entity', 'chair_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',  # slightly above stage to avoid clipping
            '-Y', '0.0',   # facing downstage (towards audience)
        ],
    )

    # 4. Twist mux (same config as real robot)
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # 5. Dance action server
    dance_server = Node(
        package='dance_manager',
        executable='dance_action_server',
        name='dance_server',
        output='screen',
        parameters=[{
            'platform': LaunchConfiguration('platform'),
        }],
    )

    # 6. Optional RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ── Assemble ────────────────────────────────────────────────────────
    return LaunchDescription([
        use_rviz_arg,
        headless_arg,
        platform_arg,

        # Start Gazebo + robot state publisher first
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,

        # Give Gazebo 3 seconds to start, then spawn robot + start dance system
        TimerAction(
            period=3.0,
            actions=[
                spawn_robot,
                twist_mux,
                dance_server,
            ],
        ),

        rviz,
    ])
