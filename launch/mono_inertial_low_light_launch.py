from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('orbslam3')
    vocabulary_path = os.path.join(
        pkg_dir,
        'vocabulary', 'ORBvoc.txt')
    config_path = os.path.join(
        pkg_dir,
        'config', 'voxl-mono-inertial','low_light_down_mono_inertial.yaml')

    return LaunchDescription([
        Node(
            package='orbslam3',
            executable='mono-inertial',
            name='orbslam3_mono_inertial',
            output='screen',
            arguments=[
                vocabulary_path,
                config_path,
            ],
            parameters=[
                {"image_topic": "/low_light_down_misp_decoded"},
                {"imu_topic": "/imu_apps"},
                {"odometry_topic": "/orbslam3/odom"},
                {"parent_frame_id": "odom"},
                {"path_topic": "/orbslam3/path"},
                {"child_frame_id": "orbslam3/base_link"}
            ]
        )
    ])