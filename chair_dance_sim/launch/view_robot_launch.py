"""
View the chair robot URDF in RViz2 (no Gazebo required).

Usage:
  ros2 launch chair_dance_sim view_robot_launch.py

This is useful for:
  - Verifying the URDF looks correct before running full sim
  - Checking joint axes and link positions
  - Quick iteration on the robot model
"""

import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    sim_pkg = get_package_share_directory('chair_dance_sim')
    urdf_file = os.path.join(sim_pkg, 'urdf', 'chair_robot.urdf.xacro')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(sim_pkg, 'config', 'view_robot.rviz')],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
