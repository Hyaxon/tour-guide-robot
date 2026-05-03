from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    turtlebot4_gz_bringup = FindPackageShare('turtlebot4_gz_bringup')
    tourbot_bringup = FindPackageShare('tourbot_bringup')

    use_custom_sim = LaunchConfiguration('use_custom_sim')
    custom_world = LaunchConfiguration('custom_world')
    custom_map = LaunchConfiguration('custom_map')

    turtlebot4_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            turtlebot4_gz_bringup,
            'launch',
            'turtlebot4_gz.launch.py',
        ])
    )

    default_sim = IncludeLaunchDescription(
        turtlebot4_launch,
        condition=UnlessCondition(use_custom_sim),
        launch_arguments={
            'nav2': 'true',
            'slam': 'false',
            'localization': 'true',
            'rviz': 'true',
        }.items(),
    )

    custom_sim = IncludeLaunchDescription(
        turtlebot4_launch,
        condition=IfCondition(use_custom_sim),
        launch_arguments={
            'nav2': 'true',
            'slam': 'false',
            'localization': 'true',
            'rviz': 'true',

            'world': custom_world,
            'map': custom_map,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_custom_sim',
            default_value='false',
            description='Launch with custom tourbot world and map',
        ),

        DeclareLaunchArgument(
            'custom_world',
            default_value=PathJoinSubstitution([
                tourbot_bringup,
                'worlds',
                'cardboard_city',
                'cardboard_city.sdf',
            ]),
            description='Custom Gazebo world name or path',
        ),

        DeclareLaunchArgument(
            'custom_map',
            default_value=PathJoinSubstitution([
                tourbot_bringup,
                'maps',
                'cardboard_city',
                'map_area.yaml',
            ]),
            description='Custom Nav2 map YAML path',
        ),

        default_sim,
        custom_sim,
    ])