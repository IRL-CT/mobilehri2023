"""
Full simulation launch — Gz Sim (Fortress) + Chair Robot + Dance System

Launches:
  1. Gz Sim with the dance stage world
  2. Robot state publisher (URDF → /robot_description, /tf)
  3. Chair robot spawned in Gz Sim
  4. ros_gz_bridge (bridges /cmd_vel, /odom, /joint_states, /clock)
  5. Twist mux (routes /dance_manager/cmd_vel → /cmd_vel)
  6. Dance action server (platform=differential_drive)

Usage:
  ros2 launch chair_dance_sim simulation_launch.py

  # Headless mode (WSL / Jetson — no GUI, use RViz instead):
  ros2 launch chair_dance_sim simulation_launch.py headless:=true use_rviz:=true

Then send dance commands:
  ros2 action send_goal /dance dance_interfaces/action/Dance \
      "{dance_move: 'Spin', params: '{\"angle\": 360}', energy: 0.7, texture: 'staccato'}"
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ───────────────────────────────────────────────────
    sim_pkg = get_package_share_directory('chair_dance_sim')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(sim_pkg, 'urdf', 'chair_robot.urdf.xacro')
    world_file = os.path.join(sim_pkg, 'worlds', 'stage.sdf')
    twist_mux_cfg = os.path.join(sim_pkg, 'config', 'twist_mux.yaml')

    # ── Launch arguments ────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 alongside Gz Sim'
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gz Sim server only (no GUI) — useful on WSL / Jetson'
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

    # 1a. Gz Sim with GUI (default)
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    # 1b. Gz Sim server only (headless)
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': f'-r -s {world_file}',
        }.items(),
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

    # 3. Spawn the chair robot in Gz Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_chair_robot',
        output='screen',
        arguments=[
            '-name', 'chair_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-Y', '0.0',
        ],
    )

    # 4. ros_gz_bridge — bridge topics between Gz Sim and ROS 2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
    )

    # 5. Twist mux (same config as real robot)
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # 6. Dance action server
    dance_server = Node(
        package='dance_manager',
        executable='dance_action_server',
        name='dance_server',
        output='screen',
        parameters=[{
            'platform': LaunchConfiguration('platform'),
        }],
    )

    # 7. Dance visualizer (stage boundary, path trail, move labels in RViz/Gazebo)
    dance_visualizer = Node(
        package='dance_manager',
        executable='dance_visualizer',
        name='dance_visualizer',
        output='screen',
        parameters=[{
            'stage_width': 6.0,
            'stage_depth': 8.0,
        }],
    )

    # 8. Optional RViz
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

        # Start Gz Sim + robot state publisher first
        gz_sim_gui,
        gz_sim_headless,
        robot_state_publisher,

        # Give Gz Sim 3 seconds to start, then spawn robot + start dance system
        TimerAction(
            period=3.0,
            actions=[
                spawn_robot,
                gz_bridge,
                twist_mux,
                dance_server,
                dance_visualizer,
            ],
        ),

        rviz,
    ])
