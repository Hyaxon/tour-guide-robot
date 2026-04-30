import rclpy

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


def landmark_to_pose(navigator: TurtleBot4Navigator, landmark: dict):
    return navigator.getPoseStamped(
        [float(landmark["x"]), float(landmark["y"])],
        to_direction(landmark["theta"])
    )

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    map_name = "cardboard_city"
    landmark_data = load_landmarks(map_name)

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose_data = landmark_data["home"]
    initial_pose = landmark_to_pose(navigator, initial_pose_data)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_poses = []
    for landmark in landmark_data["landmarks"]:
        goal_poses.append(landmark_to_pose(navigator, landmark))

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startThroughPoses(goal_poses)

    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()