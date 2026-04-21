import os
import subprocess

import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray


class DoorDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('door_detector_node')

        # FIX: Configure which AprilTag IDs represent doors 
        # (needs to match the tags placed at doorways)
        self.door_tag_ids = [10, 11, 12]  

        # Track which doors that have been detected recently to avoid repeated beeps
        self.detected_doors = set()

        # Subscribe to AprilTag detections
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_ros/detections',
            self.detection_callback,
            10,
        )

        self.get_logger().info('Door Detector Node initialized')

    def detection_callback(self, msg: AprilTagDetectionArray) -> None:
        for detection in msg.detections:
            tag_id = detection.id

            # Check if this tag ID is a door
            if tag_id in self.door_tag_ids:
                # Only beep if we haven't already detected this door recently
                if tag_id not in self.detected_doors:
                    self.get_logger().info(f'Door detected! Tag ID: {tag_id}')
                    self.beep()
                    self.detected_doors.add(tag_id)

    def beep(self) -> None:
        try:
            # Method 1: Try PulseAudio 
            subprocess.run(
                ['paplay', '/usr/share/sounds/freedesktop/stereo/complete.oga'],
                timeout=2,
                check=False,
            )
            self.get_logger().info('Beep emitted (paplay)')
        except (FileNotFoundError, subprocess.TimeoutExpired):
            try:
                # Method 2: Try system beep command
                os.system('beep')
                self.get_logger().info('Beep emitted (system beep)')
            except Exception as e:
                self.get_logger().warning(f'Could not emit beep: {e}')

    def reset_detected_doors(self) -> None:
        self.detected_doors.clear()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = DoorDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Door Detector Node shutting down')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
