import rclpy
import time
from rclpy.action import ActionClient

from tourbot_interfaces.action import AlignToAprilTag
from tourbot_interfaces.action import WaitForTagRemoved
from tourbot_interfaces.action import DoorTraverse

from action_msgs.msg import GoalStatus

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

    while rclpy.ok() and not send_goal_future.done():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    goal_handle = send_goal_future.result()

    if goal_handle is None:
        navigator.error("Action goal response was None")
        return False

    if not goal_handle.accepted:
        navigator.error("Action goal was rejected")
        return False

    navigator.info("Action goal accepted. Waiting for result...")

    result_future = goal_handle.get_result_async()

    while rclpy.ok() and not result_future.done():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result_response = result_future.result()

    if result_response is None:
        navigator.error("Action result response was None")
        return False

    result = result_response.result
    status = result_response.status

    if status != GoalStatus.STATUS_SUCCEEDED:
        navigator.error(
            f"Action did not succeed. Status={status}, message={result.message}"
        )
        return False

    if not result.success:
        navigator.error(f"Action returned failure: {result.message}")
        return False

    navigator.info(f"Action completed successfully: {result.message}")
    return True

def is_door_tag(tag_id: int) -> bool:
    return tag_id in (1, 2)


def run_door_sequence(navigator, wait_client, door_client, landmark):
    tag_id = int(landmark["tag_id"])

    wait_goal = WaitForTagRemoved.Goal()
    wait_goal.tag_id = tag_id
    wait_goal.timeout_sec = 60.0
    wait_goal.missing_duration_sec = 3.0

    tag_removed = call_action_and_wait(
        navigator,
        wait_client,
        wait_goal
    )

    if not tag_removed:
        navigator.error(
            f"Tag {tag_id} was not removed from FOV. Skipping door traversal."
        )
        return False

    door_goal = DoorTraverse.Goal()

    # Use your DoorTraverse defaults by leaving these as 0.0,
    # assuming your action fields match the server code you showed.
    door_goal.backup_distance = 0.0
    door_goal.backup_speed = 0.0
    door_goal.wait_seconds = 0.0
    door_goal.forward_distance = 0.0
    door_goal.forward_speed = 0.0

    # If your DoorTraverse.action has a tag_id field, set it:
    door_goal.tag_id = tag_id

    door_done = call_action_and_wait(
        navigator,
        door_client,
        door_goal
    )

    if not door_done:
        navigator.error(f"Door traversal failed for tag {tag_id}.")
        return False

    return True

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    align_client = ActionClient(
        navigator,
        AlignToAprilTag,
        "align_to_apriltag"
    )

    wait_for_tag_removed_client = ActionClient(
        navigator,
        WaitForTagRemoved,
        "wait_for_tag_removed"
    )

    door_traverse_client = ActionClient(
        navigator,
        DoorTraverse,
        "door_traverse"
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

        time.sleep(1.5)

        tag_id = int(landmark["tag_id"])

        align_goal = AlignToAprilTag.Goal()
        align_goal.tag_id = tag_id
        align_goal.timeout_sec = 30.0
        align_goal.x_tolerance_px = 7.0

        aligned = call_action_and_wait(
            navigator,
            align_client,
            align_goal
        )

        if not aligned:
            navigator.error(
                f"Failed to align to AprilTag {tag_id}. Skipping landmark."
            )
            continue

        time.sleep(1.0)

        if is_door_tag(tag_id):
            navigator.info(
                f"Tag {tag_id} is a door tag. Waiting for removal before traversal."
            )

            door_ok = run_door_sequence(
                navigator,
                wait_for_tag_removed_client,
                door_traverse_client,
                landmark
            )

            if not door_ok:
                navigator.error(
                    f"Door sequence failed for tag {tag_id}. Skipping rotation."
                )
                continue

        if not is_door_tag(tag_id):
            navigator.info("Landmark interaction finished. Rotating 180 degrees now.")

            rotated_pose = landmark_to_rotated_pose(navigator, landmark)
            navigator.startToPose(rotated_pose)

        time.sleep(5.0)

    navigator.info("Tour complete")


    #navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()