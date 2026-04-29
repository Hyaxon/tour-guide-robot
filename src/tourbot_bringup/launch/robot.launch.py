from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    map_yaml = os.path.join(
        get_package_share_directory('tourbot_bringup'),
        'maps',
        'cardboard_city',
        'map_area.yaml'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'localization.launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'false',
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'nav2.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    view_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_viz'),
                'launch',
                'view_navigation.launch.py'
            )
        )
    )

    return LaunchDescription([
        localization_launch,
        nav2_launch,
        view_navigation_launch,
    ])