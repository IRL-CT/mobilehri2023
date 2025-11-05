
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    dance_server_node = Node(
            package='dance_manager',
            executable='dance_action_server',
            name='dance_server'
        )

    return LaunchDescription([
        dance_server_node,
    ])
