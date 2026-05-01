from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to predefined map
    map_yaml = os.path.join(
        get_package_share_directory('tourbot_bringup'),
        'maps',
        'cardboard_city',
        'map_area.yaml'
    )

    nav2_params = os.path.join(
        get_package_share_directory('tourbot_bringup'),
        'config',
        'nav2_params_backup.yaml'
    )

    # Launch rviz2 in navigation mode
    view_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_viz'),
                'launch',
                'view_navigation.launch.py'
            )
        )
    )

    # Localize TurtleBot4 using predefined map instead of running SLAM
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
            #'params_file': nav2_params,
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
            'params': nav2_params,
        }.items()
    )
    

    return LaunchDescription([
        view_navigation_launch,
        localization_launch,
        nav2_launch
    ])