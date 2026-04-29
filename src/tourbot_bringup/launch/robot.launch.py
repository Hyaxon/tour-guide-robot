from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    map_yaml = os.path.join(
        get_package_share_directory('tourbot_bringup'),
        'maps',
        'cardboard_city',
        'map_area.yaml'
    )

    nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'nav_bringup.launch.py'
            )
        ),
        launch_arguments={
            'slam': 'off',
            'localization': 'true',
            'map': map_yaml,
        }.items()
    )

    return LaunchDescription([
        LogInfo(msg=['Using map: ', map_yaml]),
        nav_bringup_launch,
    ])