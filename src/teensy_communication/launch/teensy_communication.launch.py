from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teensy_communication',
            executable='teensy_comm_node',
            name='teensy_comm',
            output='screen',
            parameters=[{
                'port': '/dev/ttyAMA0',
                'baud': 115200,
                'rate_hz': 50,
                'send_mode': 'constant_one',  # 'constant_one' | 'angle'
                'angle_topic': '/desired_angle_deg'
            }]
        )
    ])
