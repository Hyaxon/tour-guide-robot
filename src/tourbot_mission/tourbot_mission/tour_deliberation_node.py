import rclpy
import time
from rclpy.action import ActionClient

from tourbot_interfaces.action import AlignToAprilTag

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)

from tourbot_landmarks.landmarks_loader import load_landmarks


def to_direction(direction_name: str):
    try:
        return getattr(TurtleBot4Directions, direction_name)
    except AttributeError as e:
        raise ValueError(f"Invalid direction '{direction_name}' in landmarks.yaml") from e


def direction_to_degrees(direction_name: str) -> float:
    return float(to_direction(direction_name))


def landmark_to_pose(navigator: TurtleBot4Navigator, landmark: dict):
    return navigator.getPoseStamped(
        [float(landmark["x"]), float(landmark["y"])],
        direction_to_degrees(landmark["theta"])
    )


def landmark_to_rotated_pose(navigator: TurtleBot4Navigator, landmark: dict):
    theta = direction_to_degrees(landmark["theta"])
    rotated_theta = (theta + 180.0) % 360.0

    return navigator.getPoseStamped(
        [float(landmark["x"]), float(landmark["y"])],
        rotated_theta
    )


def get_nearest_landmark(l1x, l1y, landmark_raw_data):
    nearest_landmark = 0
    nearest_dist = float("inf")

    for i, landmark in enumerate(landmark_raw_data):
        dist = (l1x - landmark["x"]) ** 2 + (l1y - landmark["y"]) ** 2

        if dist < nearest_dist:
            nearest_landmark = i
            nearest_dist = dist

    return nearest_landmark


def call_action_and_wait(navigator, action_client, goal_msg):
    navigator.info("Waiting for action server...")

    if not action_client.wait_for_server(timeout_sec=10.0):
        navigator.error("Action server not available")
        return False

    navigator.info("Sending action goal...")

    send_goal_future = action_client.send_goal_async(goal_msg)

    while not send_goal_future.done():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        navigator.error("Action goal was rejected")
        return False

    navigator.info("Action goal accepted. Waiting for result...")

    result_future = goal_handle.get_result_async()

    while not result_future.done():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result_response = result_future.result()
    result = result_response.result

    if not result.success:
        navigator.error(f"Action failed: {result.message}")
        return False

    navigator.info(f"Action completed: {result.message}")
    return True


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    align_client = ActionClient(
        navigator,
        AlignToAprilTag,
        "align_to_apriltag"
    )

    map_name = "cardboard_city"
    landmark_data = load_landmarks(map_name)

    # Start on dock
    #if not navigator.getDockedStatus():
    #    navigator.info('Docking before intialising pose')
    #    navigator.dock()

    # Set initial pose
    initial_pose_data = landmark_data["home"]
    initial_pose = landmark_to_pose(navigator, initial_pose_data)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    landmark_raw_data = []
    for landmark in landmark_data["landmarks"]:
        landmark_raw_data.append(landmark)

    goal_landmarks = []
    last_landmark = landmark_data["home"]

    while landmark_raw_data:
        nearest_landmark = get_nearest_landmark(
            last_landmark["x"],
            last_landmark["y"],
            landmark_raw_data
        )

        next_landmark = landmark_raw_data.pop(nearest_landmark)
        goal_landmarks.append(next_landmark)
        last_landmark = next_landmark

    # Return home to end the tour
    goal_landmarks.append(landmark_data["home"])

    # Undock
    #navigator.undock()

    # Go to each goal pose
    # Go through all waypoints using Nav2 waypoint_follower
    for landmark in goal_landmarks:
        goal_pose = landmark_to_pose(navigator, landmark)
        navigator.startToPose(goal_pose)

        time.sleep(5.0)

        align_goal = AlignToAprilTag.Goal()
        align_goal.tag_id = int(landmark["tag_id"])
        align_goal.timeout_sec = 10.0
        align_goal.x_tolerance_px = 20.0

        aligned = call_action_and_wait(
            navigator,
            align_client,
            align_goal
        )

        if not aligned:
            navigator.error(
                f"Failed to align to AprilTag {landmark['tag_id']}. Skipping rotation."
            )
            continue

        rotated_pose = landmark_to_rotated_pose(navigator, landmark)
        navigator.startToPose(rotated_pose)

        time.sleep(5.0)

    navigator.info("Tour complete")


    #navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()