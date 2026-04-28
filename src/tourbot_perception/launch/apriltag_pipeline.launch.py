from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tourbot_perception')
    tag_config = os.path.join(pkg_share, 'config', 'apriltags_36h11.yaml')

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[tag_config],
            remappings=[
                ('image_rect', '/oakd/rgb/preview/image_raw'),
                ('camera_info', '/oakd/rgb/preview/camera_info'),
            ],
        ),
    ])