from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teensy_communication',
            executable='teensy_comm_node',
            name='teensy_comm',
            output='screen'
        )
    ])
