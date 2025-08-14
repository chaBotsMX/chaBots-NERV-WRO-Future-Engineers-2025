from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='otos_reader',
            executable='otos_node',
            name='otos_reader',
            output='screen'
        )
    ])
