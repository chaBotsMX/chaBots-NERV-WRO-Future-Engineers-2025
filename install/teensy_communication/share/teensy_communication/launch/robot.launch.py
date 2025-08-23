#!/usr/bin/env python3

import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration , PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from shutil import which



def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # Paths
    teensy_comm_dir = get_package_share_directory('teensy_communication')
    urdf_path = os.path.join(teensy_comm_dir, 'urdf', 'robot.urdf.xacro')
    config = os.path.join(
        get_package_share_directory('teensy_communication'),
        'config',
        'filtros.yaml'
    )

    otos_reader_dir = get_package_share_directory('otos_reader')
    # Definir la ruta al archivo XACRO
    urdf_path = "/home/chabots/ros2_ws/install/teensy_communication/share/teensy_communication/urdf/robot.urdf"
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Publicar el modelo URDF y los TF estáticos usando método recomendado
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
            output="screen",
        ),
        # Lanzar el nodo otos_reader
        Node(
            package='otos_reader',
            executable='otos_node',
            name='otos_reader',
            output='screen',
            prefix=['/home/chabots/ros2_ws/otos_env/bin/python', ' -u '],
        ),
        # Lanzar el nodo foxglove_bridge usando Node()
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filters',
            output='screen',
            parameters=[config],  # Aquí solo el archivo
            remappings=[
                ('scan', 'scan'),                 # entrada
                ('scan_filtered', 'scan_filtered')# salida
            ]
        ),
    
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen'),

        Node(
            package='teensy_communication',
            executable='teensy_comm_node',
            name='teensy_comm',
            output='screen'
        ),


    ])
