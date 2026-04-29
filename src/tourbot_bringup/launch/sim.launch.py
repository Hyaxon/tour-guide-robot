from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

use_custom_sim = False

def generate_launch_description():
    turtlebot4_gz_bringup = FindPackageShare('turtlebot4_gz_bringup')

    default_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                turtlebot4_gz_bringup,
                'launch',
                'turtlebot4_gz.launch.py',
            ])
        ),
        launch_arguments={
            'nav2': 'true',
            'slam': 'false',
            'localization': 'true',
            'rviz': 'true',
        }.items(),
    )

    custom_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                turtlebot4_gz_bringup,
                'launch',
                'turtlebot4_gz.launch.py',
            ])
        ),
        launch_arguments={
            'nav2': 'true',
            'slam': 'false',
            'localization': 'true',
            'rviz': 'true',
        }.items(),
    )


    sim_launch = default_sim if not use_custom_sim else custom_sim

    return LaunchDescription([
        sim_launch,
    ])