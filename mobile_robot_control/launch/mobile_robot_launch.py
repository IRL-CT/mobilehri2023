import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('mobile_robot_control')
    cfg = os.path.normpath(os.path.join(pkg_share, 'config', 'twist_mux.yaml'))

    mobile_robot_control_node = Node(
            package='mobile_robot_control',
            executable='mobile_robot_control_node',
            name='robot_controll'
        )
    joy_teleop_keymapping_node = Node(
            package='joy_teleop_keymapping',
            executable='keymapping_node',
            name='keymap'
        )
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
        )

    mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[cfg],
            remappings=[('cmd_vel_out', '/cmd_vel')],

        )

    return LaunchDescription([
        mobile_robot_control_node,
        joy_teleop_keymapping_node,
        joy_node,
        mux_node
    ])