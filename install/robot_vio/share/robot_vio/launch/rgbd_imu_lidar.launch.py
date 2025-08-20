from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- User-tunable args ---
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_baud       = LaunchConfiguration('lidar_baud')
    base_frame       = LaunchConfiguration('base_frame')
    camera_frame     = LaunchConfiguration('camera_frame')
    laser_frame      = LaunchConfiguration('laser_frame')

    return LaunchDescription([
        # ====== Arguments ======
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud',        default_value='1000000'),  # S3 default
        DeclareLaunchArgument('base_frame',        default_value='base_link'),
        DeclareLaunchArgument('camera_frame',      default_value='camera_link'),
        DeclareLaunchArgument('laser_frame',       default_value='laser_frame'),

        # ====== RealSense D455 ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_gyro':  'true',
                'enable_accel': 'true',
                'enable_sync':  'true',
                'unite_imu_method': '2',             # 2=linear_interpolation
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'color_fps': '30',
                'depth_fps': '30'
            }.items()
        ),

        # ====== IMU filter (Madgwick) ======
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
            name='imu_filter', output='screen',
            parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}],
            remappings=[
                ('/imu/data_raw', '/camera/camera/imu'),
                ('/imu/data',     '/rtabmap/imu')
            ]
        ),

        # ====== RPLidar S3 (sllidar_ros2) ======
        Node(
            package='sllidar_ros2', executable='sllidar_node', name='rplidar_s3', output='screen',
            parameters=[{
                'serial_port': lidar_serial_port,
                'serial_baudrate': lidar_baud,       # 1,000,000 for RPLidar S3
                'frame_id': laser_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': ''                       # empty=>driver default; or e.g. 'Standard'
            }]
        ),

        # ====== Static TFs (measure and update XYZ/RPY!) ======
        # base_link -> camera_link
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='tf_base_to_camera',
            arguments=['0.0','0.0','0.0', '0','0','0', base_frame, camera_frame]
        ),
        # base_link -> laser_frame
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='tf_base_to_laser',
            arguments=['0.0','0.0','0.0', '0','0','0', base_frame, laser_frame]
        ),

        # ====== RTAB-Map (RGB‑D‑Inertial + LiDAR) ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
            launch_arguments={
                'rgb_topic':           '/camera/camera/color/image_raw',
                'depth_topic':         '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic':   '/camera/camera/color/camera_info',
                'imu_topic':           '/rtabmap/imu',
                'scan_topic':          '/scan',           # published by sllidar_ros2
                'subscribe_scan':      'true',
                'frame_id':            base_frame,
                'odom_frame_id':       'odom',
                'map_frame_id':        'map',
                'approx_sync':         'true',
                'qos':                 '1',               # SensorData QoS in many setups
                'wait_imu_to_init':    'true'
            }.items()
        ),
    ])

