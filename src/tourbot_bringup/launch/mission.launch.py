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
        output="screen",
        parameters=[
            {
                "detections_topic": "/detections",
                "camera_info_topic": "/oakd/rgb/preview/camera_info",
                "cmd_vel_topic": "/cmd_vel",
            }
        ],
    )

    wait_for_tag_removed_server = Node(
        package="tourbot_behaviors",
        executable="wait_for_tag_removed_server",
        name="wait_for_tag_removed_server",
        output="screen",
        parameters=[
            {
                "detections_topic": "/detections",
                "control_rate_hz": 20.0,
            }
        ],
    )

    door_behavior_server = Node(
        package="tourbot_behaviors",
        executable="door_behavior_server",
        name="door_behavior_server",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": "/cmd_vel",
                "odom_topic": "/odom",
                "backup_distance_default": 0.9,
                "backup_speed_default": 0.15,
                "wait_seconds_default": 3.0,
                "forward_distance_default": 1.5,
                "forward_speed_default": 0.18,
                "control_rate_hz": 20.0,
            }
        ],
    )

    # Main mission node
    tour_deliberation_node = Node(
        package="tourbot_mission",
        executable="tour_deliberation_node",
        name="tour_deliberation_node",
        output="screen",
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