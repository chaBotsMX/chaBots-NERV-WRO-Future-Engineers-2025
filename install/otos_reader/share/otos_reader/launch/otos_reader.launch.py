from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    venv_python = "/home/chabots/ros2_ws/otos_env/bin/python"
    return LaunchDescription([
        Node(
            package='otos_reader',
            executable='otos_node',
            name='otos_reader',
            output='screen'
        )
    ])
