# tour_guide_robot

ROS 2 TurtleBot4 tour guide robot that navigates a mapped indoor environment, visits predefined landmarks, uses AprilTags for landmark confirmation and alignment, and handles simplified door-aware behaviors through custom ROS 2 action servers.

## Overview

This project implements a hybrid deliberative/reactive robot architecture for an autonomous indoor tour guide. The robot uses Nav2 for map-based navigation between landmark poses, AprilTags for visual confirmation and alignment, and custom behavior action servers for tasks such as waiting for a door to open and traversing a doorway.

### Robot Duties

* Navigate using a prebuilt occupancy map

* Visit predefined landmarks loaded from YAML configuration files

* Confirm landmark arrival using AprilTags

* Align toward AprilTags before continuing the tour

* Detect simplified door state using AprilTag visibility

* Wait for human assistance at closed doors

* Execute staged door traversal behavior

* Coordinate behaviors using ROS 2 action servers

### Sensors Used

* **LiDAR (RPLIDAR)** - Mapping, localization, obstacle detection
* **Camera (OAK-D)** - AprilTag detection
* **Encoders** - Odometry

### Dependencies

* ROS 2 Jazzy
* Turtlebot4
* TurtleBot4 packages
* Nav2
* RViz2
* Gazebo Harmonic
* SLAM Toolbox
* apriltag_ros
* TF2
  
## Project Packages

### tourbot_bringup

Contains launch files, map files, and Nav2 configuration for starting the robot system.

Important files:

```text
tourbot_bringup/
├── config/
│   └── nav2_params.yaml
├── launch/
│   ├── robot.launch.py
│   ├── mission.launch.py
│   └── sim.launch.py
├── maps/
│   └── cardboard_city/
│       ├── map_area.pgm
│       └── map_area.yaml
└── worlds/
```

Launch files:

* `robot.launch.py`: starts localization, Nav2, and RViz using the cardboard city map

* `mission.launch.py`: starts AprilTag perception, behavior action servers, and the mission controller

* `sim.launch.py`: reserved for simulation bringup

### tourbot_landmarks

Stores landmark configuration files for different environments.

Important files:

```text
tourbot_landmarks/
└── config/
    └── cardboard_city/
        └── landmarks.yaml
```

The landmark YAML files define tour location poses, directions, and associated metadata such as AprilTag IDs.

### tourbot_mission

Contains the high-level mission logic for executing the tour.

Important files:

```text
tourbot_mission/
└── tourbot_mission/
    └── tour_deliberation_node.py
```

* `tour_deliberation_node.py`: controls the tour sequence and sends navigation/behavior goals
  
### tourbot_behaviors

Contains custom robot behavior servers.

Important files:

```text
tourbot_behaviors/
└── launch/
    ├── align_to_apriltag.launch.py
    └── door_behavior.launch.py
```

Main behaviors:

* `align_to_apriltag_server`: rotates the robot until a selected AprilTag is centered in the camera image

* `wait_for_tag_removed_server`: waits until a specified AprilTag is no longer visible

* `door_behavior_server`: executes a staged door traversal behavior using odometry and velocity commands

### tourbot_perception

Contains AprilTag perception configuration and launch files.

Important files:

```text
tourbot_perception/
├── config/
│   └── apriltags_36h11.yaml
└── launch/
    └── apriltag_pipeline.launch.py
```

This package launches the AprilTag detection pipeline using the robot’s OAK-D camera stream.

### tourbot_interfaces

Contains custom ROS 2 action definitions.

Important files:

```text
tourbot_interfaces/
└── action/
    ├── AlignToAprilTag.action
    ├── DoLandmarkTask.action # Currently unimplemented 
    ├── DoorTraverse.action
    └── WaitForTagRemoved.action
```

* `AlignToAprilTag.action`: goal/feedback/result interface for AprilTag alignment

* `DoorTraverse.action`: interface for staged door traversal

* `WaitForTagRemoved.actio`n: interface for waiting until a tag disappears

* `DoLandmarkTask.action`: interface for landmark-level task behavior

## System Flow

1. The robot is placed at the designated starting pose in the cardboard city map.
2. robot.launch.py starts localization, Nav2, and RViz.
3. mission.launch.py starts AprilTag perception, behavior servers, and the mission controller.
4. The mission controller loads landmark poses from YAML configuration.
5. The robot navigates to the next landmark using Nav2.
6. At landmarks, the robot searches for and aligns to the associated AprilTag.
7. If a door-related tag is detected, the robot waits for the tag to disappear, representing the door being opened.
8. The robot executes the door traversal behavior when appropriate.
9. The robot continues through the landmark sequence.

## Behaviors

### AprilTag Alignment

The AprilTag alignment server rotates the robot in place until the target tag is horizontally centered in the camera image.

The current implementation uses a fixed search direction while looking for tags. This works, but right-side tags may take longer to detect because the robot may rotate the longer way around.

### Door Detection and Traversal

Door state is represented using AprilTag visibility.

* If the door tag is visible, the robot treats the door as closed.
* If the tag disappears, the robot treats the door as opened.
* The robot can then execute a staged traversal behavior.

The traversal behavior uses odometry and velocity commands to perform simple movement stages such as backing up, pausing, and moving forward.

## Setup and Execution

1. Build the packages

    From the workspace root:

    ```bash
    colcon build --symlink-install
    ```

2. Source the workspace

    ```bash
    source install/setup.bash
    ```

3. Start robot navigation bringup

    Launch localization, Nav2, and RViz:

    ```bash
    ros2 launch tourbot_bringup robot.launch.py
    ```

4. Start mission bringup

    In a second terminal, source the workspace again and launch the mission system:

    ```bash
    source install/setup.bash
    ros2 launch tourbot_bringup mission.launch.py
    ```

    This starts:

    * AprilTag perception pipeline
    * AprilTag alignment action server
    * wait-for-tag-removed action server
    * door behavior action server
    * tour deliberation node

    The mission controller starts last so that perception and behavior servers are available before goals are sent.

### Maps and Landmarks

The current tested map is stored in:

```text
tourbot_bringup/maps/cardboard_city/
```

The corresponding landmark file is stored in:

```text
tourbot_landmarks/config/cardboard_city/landmarks.yaml
```

To adapt the tour to a new environment, create a new map directory with a `map_area.pgm` and `map_area.yaml` and add a matching landmark YAML configuration file in the landmarks package.

## Current Limitations

* The robot relies on a prebuilt map.
* AprilTag detection depends on camera visibility, lighting, angle, and tag placement.
* Door state is simplified as tag visible = closed and tag removed = open.
* AprilTag search currently uses a fixed rotation direction.
* Human-following checks and battery-aware docking are planned but not fully implemented.
* No custom Gazebo world was fully completed for the cardboard city environment.

## Future Improvements

* Make AprilTag search direction-aware to reduce alignment time.
* Add sound cues so the robot can request human assistance at doors.
* Improve door-state detection beyond simple tag visibility.
* Add stronger dynamic obstacle and human-awareness behavior and safety features.
* Add battery-aware return-to-dock behavior.
* Improve simulation support with a complete custom Gazebo world.

## License

Original code in this repository is licensed under the MIT License. See
[`LICENSE`](LICENSE).

This repository also includes third-party software under separate licenses,
including `apriltag`, `apriltag_ros`, and `apriltag_msgs`. See
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) and the license files in the
corresponding package directories.