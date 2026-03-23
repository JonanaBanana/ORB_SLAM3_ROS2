from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('orbslam3')
    vocabulary_path = os.path.join(
        get_package_share_directory('orbslam3'),
        'vocabulary', 'ORBvoc.txt')
    config_path = os.path.join(
        get_package_share_directory('orbslam3'),
        'config', 'voxl','low_light_down_mono.yaml')
    return LaunchDescription([
        Node(
            package='orbslam3',
            executable='mono',
            name='mono',
            output='screen',
            arguments=[
                vocabulary_path,
                config_path,
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_cam_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        )
    ])