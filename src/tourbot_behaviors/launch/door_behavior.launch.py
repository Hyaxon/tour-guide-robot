from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tourbot_behaviors')
    config_path = os.path.join(pkg_share, 'config', 'door_behavior.yaml')

    return LaunchDescription([
        Node(
            package='tourbot_behaviors',
            executable='door_behavior_server',
            name='door_behavior_server',
            output='screen',
            parameters=[config_path],
        ),
        Node(
            package='tourbot_behaviors',
            executable='manual_door_override_node',
            name='manual_door_override_node',
            output='screen',
        ),
    ])