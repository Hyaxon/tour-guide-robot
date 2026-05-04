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

# Utility function that converts a direction name like "NORTH" to angle in degrees.
def direction_to_degrees(direction_name: str) -> float:
    try:
        return float(getattr(TurtleBot4Directions, direction_name))
    except AttributeError as e:
        raise ValueError(f"Invalid direction '{direction_name}' in landmarks.yaml") from e


# Utility functions for converting landmark data to PoseStamped messages.
def landmark_to_pose(navigator: TurtleBot4Navigator, landmark: dict):
    return navigator.getPoseStamped(
        [float(landmark["x"]), float(landmark["y"])],
        direction_to_degrees(landmark["theta"])
    )

# Utility function to convert landmark data to a PoseStamped rotated 180 degrees. 
# Used to prevent Nav2 from ever navigating backwards, 
# which is problematic for TurtleBot4. 
def landmark_to_rotated_pose(navigator: TurtleBot4Navigator, landmark: dict):
    theta = direction_to_degrees(landmark["theta"])
    rotated_theta = (theta + 180.0) % 360.0

    return navigator.getPoseStamped(
        [float(landmark["x"]), float(landmark["y"])],
        rotated_theta
    )

# To plan a tour, we decided to use a simple greedy approach, 
# the tourbot will always navigate to the nearest unvisited landmark next.
def get_nearest_landmark(l1x, l1y, landmark_raw_data):
    nearest_landmark = 0
    nearest_dist = float("inf")

    for i, landmark in enumerate(landmark_raw_data):
        dist = (l1x - landmark["x"]) ** 2 + (l1y - landmark["y"]) ** 2

        if dist < nearest_dist:
            nearest_landmark = i
            nearest_dist = dist

    return nearest_landmark

# Utility function to call an action and wait for the result,
# while also spinning the navigator node to keep it responsive.
def call_action_and_wait(navigator, action_client, goal_msg):
    navigator.info("Waiting for action server...")

    if not action_client.wait_for_server(timeout_sec=10.0):
        navigator.error("Action server not available")
        return False

    navigator.info("Sending action goal...")

    send_goal_future = action_client.send_goal_async(goal_msg)

    # Wait for the goal to be accepted and get the result, while spinning the navigator to keep it responsive.
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

    # Wait for the result while spinning the navigator to keep it responsive. 
    # If ROS is shut down while waiting, abort the action and return failure.
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

# Simple utility for checking if the tag is a door tag. 
# We chose 1 and 2 to be door tags 
# 1 is an outward door and 2 is an inward door. 
def is_door_tag(tag_id: int) -> bool:
    return tag_id in (1, 2)

# Helper function to run the door sequence, 
# which waits for the door tag to be removed and then calls the door traverse action.
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

    # Rely on defaults
    door_goal.backup_distance = 0.0
    door_goal.backup_speed = 0.0
    door_goal.wait_seconds = 0.0
    door_goal.forward_distance = 0.0
    door_goal.forward_speed = 0.0
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

# Main function for the tour deliberation node, 
# which plans the tour and executes it by navigating to each landmark 
# and performing the appropriate actions based on whether it's a door or not.
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator() # The Turtlebot4Navigator handles the core navigation stack.

    # Define action clients for the behaviors we will need during the tour.
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

    # Load landmark data from the YAML file for the specified map.
    map_name = "cardboard_city"
    landmark_data = load_landmarks(map_name)

    # Docking behavior if needed. 
    # Start on dock
    #if not navigator.getDockedStatus():
    #    navigator.info('Docking before intialising pose')
    #    navigator.dock()

    # Set initial pose
    initial_pose_data = landmark_data["home"]
    initial_pose = landmark_to_pose(navigator, initial_pose_data)

    # Ran into issues with the initial pose not being set correctly, 
    # so we set it multiple times with some delay to ensure it is set.
    for _ in range(10):
        navigator.setInitialPose(initial_pose)
        rclpy.spin_once(navigator, timeout_sec=0.1)
        time.sleep(0.1)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Convert the landmark data into a tour order using a greedy nearest neighbor approach,
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

        # Align to the AprilTag at the landmark using the align_to_apriltag action server.
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

        # Wait a moment after alignment. 
        time.sleep(1.0)

        # If this is a door tag, run the door sequence.
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

        # If this is not a door tag, we can just rotate in place to away from the landmark,
        if not is_door_tag(tag_id):
            navigator.info("Landmark interaction finished. Rotating 180 degrees now.")

            rotated_pose = landmark_to_rotated_pose(navigator, landmark)
            navigator.startToPose(rotated_pose)

        time.sleep(5.0)

    navigator.info("Tour complete")


    #navigator.dock()

    rclpy.shutdown()

# Entry point for the tour deliberation node.
if __name__ == '__main__':
    main()