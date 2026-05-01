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


def get_nearest_landmark(l1x, l1y, landmark_raw_data):
    nearest_landmark = 0
    nearest_dist = float("inf")

    for i, landmark in enumerate(landmark_raw_data):
        dist = (l1x - landmark["x"]) ** 2 + (l1y - landmark["y"]) ** 2

        if dist < nearest_dist:
            nearest_landmark = i
            nearest_dist = dist

    return nearest_landmark


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

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

    goal_poses = []
    last_landmark = landmark_data['home']
    while landmark_raw_data:
        nearest_landmark = get_nearest_landmark(
            last_landmark['x'], 
            last_landmark['y'], 
            landmark_raw_data
        )

        goal_poses.append(landmark_to_pose(navigator, landmark_raw_data[nearest_landmark]))
        last_landmark = landmark_raw_data.pop(nearest_landmark)

    # Return home to end the tour
    goal_poses.append(initial_pose)


    # Undock
    #navigator.undock()

    # Go to each goal pose
    for goal_pose in goal_poses:
        navigator.startToPose(goal_pose)

        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.1)

        navigator.info("Reached landmark")


    #navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()