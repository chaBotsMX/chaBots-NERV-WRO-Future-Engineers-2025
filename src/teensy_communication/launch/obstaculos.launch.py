#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --------- Args LIDAR ----------
    channel_type     = LaunchConfiguration('channel_type', default='serial')
    serial_port      = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate  = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id         = LaunchConfiguration('frame_id', default='laser')
    inverted         = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode        = LaunchConfiguration('scan_mode', default='Standard')

    # --------- Args CÁMARA ----------
    cam_device = LaunchConfiguration(
        'camera',
        default='/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a'
    )
    cam_format = LaunchConfiguration('format', default='RGB888')  # más ligero que XRGB8888
    cam_width  = LaunchConfiguration('width',  default='1920')
    cam_height = LaunchConfiguration('height', default='1080')
    # Opcional: URL de calibración (deja vacío si no tienes YAML aún)
    cam_info_url = LaunchConfiguration('camera_info_url', default='')

    # --------- Rutas y config existentes ----------
    teensy_comm_dir = get_package_share_directory('teensy_communication')
    config = os.path.join(teensy_comm_dir, 'config', 'filtros.yaml')

    # URDF ya instalado (tal como tenías)
    urdf_path = "/home/chabots/ros2_ws/install/teensy_communication/share/teensy_communication/urdf/robot.urdf"
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        # ===== Entorno para libcamera PiSP =====
        SetEnvironmentVariable(
            name='LIBCAMERA_IPA_MODULE_PATH',
            value='/usr/local/lib/aarch64-linux-gnu/libcamera/ipa'
        ),
        # Anteponer /usr/local para que coja tu libcamera compilado
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value='/usr/local/lib/aarch64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')
        ),

        # ===== Robot State Publisher =====
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
            output="screen",
        ),

        # ===== OTOS Reader =====
        Node(
            package='otos_reader',
            executable='otos_node',
            name='otos_reader',
            output='screen',
            prefix=['/home/chabots/ros2_ws/otos_env/bin/python', ' -u '],
        ),

        # ===== Foxglove Bridge =====
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),

        # ===== Filtros láser =====
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filters',
            output='screen',
            parameters=[config],
            remappings=[
                ('scan', 'scan'),
                ('scan_filtered', 'scan_filtered')
            ]
        ),

        # ===== RPLIDAR =====
        DeclareLaunchArgument('channel_type',     default_value=channel_type),
        DeclareLaunchArgument('serial_port',      default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate',  default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id',         default_value=frame_id),
        DeclareLaunchArgument('inverted',         default_value=inverted),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),
        DeclareLaunchArgument('scan_mode',        default_value=scan_mode),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type':     channel_type,
                'serial_port':      serial_port,
                'serial_baudrate':  serial_baudrate,
                'frame_id':         frame_id,
                'inverted':         inverted,
                'angle_compensate': angle_compensate,
                'scan_mode':        scan_mode
            }],
            output='screen'
        ),

        # ===== Teensy obs =====
        Node(
            package='teensy_communication',
            executable='teensy_obs_node',
            name='teensy_obs',
            output='screen'
        ),

        # ===== CÁMARA (libcamera PiSP) =====
        DeclareLaunchArgument('camera',          default_value=cam_device),
        DeclareLaunchArgument('format',          default_value=cam_format),
        DeclareLaunchArgument('width',           default_value=cam_width),
        DeclareLaunchArgument('height',          default_value=cam_height),
        DeclareLaunchArgument('camera_info_url', default_value=cam_info_url),

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
            'format': 'RGB888',
            'width': 1920,
            'height': 1080,
            'controls': {
                'FrameDurationLimits': [33333, 33333],  # 30 fps        
            }
            }],
    
            env={
                **os.environ,
                'HOME': '/home/chabots',
                'ROS_HOME': '/home/chabots/.ros',
                'ROS_LOG_DIR': '/home/chabots/.ros/log',
                'LD_LIBRARY_PATH': f"/usr/local/lib/aarch64-linux-gnu:{os.environ.get('LD_LIBRARY_PATH','')}",
                'LIBCAMERA_IPA_MODULE_PATH': '/usr/local/lib/aarch64-linux-gnu/libcamera/ipa',
                'LIBCAMERA_LOG_LEVELS': '*:INFO',
                # descomenta si quieres desactivar el backend de archivo
                # 'RCL_LOGGING_IMPLEMENTATION': 'rcl_logging_noop',
            }
        ),

        ### VISION NODE
        Node(
            package='vision_node',
            executable='color_detection_node',
            name='color_detection',
            output='screen',
        ),

    ])
