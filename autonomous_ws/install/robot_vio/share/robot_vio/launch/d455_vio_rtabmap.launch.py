from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'enable_sync': 'true',
            'unite_imu_method': '2',          # 2=linear_interpolation, 1=copy
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'color_fps': '30',
            'depth_fps': '30'
        }.items()
    )

    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu'),
            ('/imu/data',     '/rtabmap/imu')
        ]
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/rtabmap/imu',
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'approx_sync': 'true',
            'qos': '1',
            'wait_imu_to_init': 'true'
        }.items()
    )

    return LaunchDescription([rs, imu_filter, rtabmap])

