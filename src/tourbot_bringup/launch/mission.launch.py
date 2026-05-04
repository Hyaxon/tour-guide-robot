from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Perception launch file
    apriltag_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tourbot_perception"),
                "launch",
                "apriltag_pipeline.launch.py",
            )
        )
    )

    # Behavior servers
    align_to_apriltag_server = Node(
        package="tourbot_behaviors",
        executable="align_to_apriltag_server",
        name="align_to_apriltag_server",    
    )

    wait_for_tag_removed_server = Node(
        package="tourbot_behaviors",
        executable="wait_for_tag_removed_server",
        name="wait_for_tag_removed_server",
    )

    door_behavior_server = Node(
        package="tourbot_behaviors",
        executable="door_behavior_server",
        name="door_behavior_server",
    )

    # Main mission node
    tour_deliberation_node = Node(
        package="tourbot_mission",
        executable="tour_deliberation_node",
        name="tour_deliberation_node",
    )

    return LaunchDescription(
        [
            # Start AprilTag detection first.
            apriltag_pipeline_launch,

            # Give perception a moment to start publishing.
            TimerAction(
                period=2.0,
                actions=[align_to_apriltag_server],
            ),

            TimerAction(
                period=2.5,
                actions=[wait_for_tag_removed_server],
            ),

            # Door behavior depends on odom/cmd_vel being available from robot bringup.
            TimerAction(
                period=3.0,
                actions=[door_behavior_server],
            ),

            # Start mission last so action servers are available before it sends goals.
            TimerAction(
                period=10.0,
                actions=[tour_deliberation_node],
            ),
        ]
    )