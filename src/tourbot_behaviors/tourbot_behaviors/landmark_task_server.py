import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from tourbot_interfaces.action import DoLandmarkTask


class LandmarkTaskServer(Node):
    def __init__(self):
        super().__init__("landmark_task_server")

        self._action_server = ActionServer(
            self,
            DoLandmarkTask,
            "/tourbot/do_landmark_task",
            self.execute_callback,
        )

        self.get_logger().info("Landmark task server started")

    async def execute_callback(self, goal_handle):
        waypoint_index = goal_handle.request.waypoint_index
        expected_tag_id = goal_handle.request.expected_tag_id

        self.get_logger().info(
            f"Running landmark task for waypoint {waypoint_index}"
        )

        feedback = DoLandmarkTask.Feedback()
        feedback.status = "Scanning for AprilTag"
        goal_handle.publish_feedback(feedback)

        detected_tag_id = self.scan_for_apriltag()

        if detected_tag_id is None:
            goal_handle.abort()
            result = DoLandmarkTask.Result()
            result.success = False
            result.detected_tag_id = -1
            result.message = "No AprilTag detected"
            return result

        feedback.status = f"Detected AprilTag {detected_tag_id}"
        goal_handle.publish_feedback(feedback)

        if expected_tag_id != -1 and detected_tag_id != expected_tag_id:
            goal_handle.abort()
            result = DoLandmarkTask.Result()
            result.success = False
            result.detected_tag_id = detected_tag_id
            result.message = (
                f"Expected tag {expected_tag_id}, but detected tag {detected_tag_id}"
            )
            return result

        feedback.status = f"Aligning with AprilTag {detected_tag_id}"
        goal_handle.publish_feedback(feedback)

        aligned = self.align_with_apriltag(detected_tag_id)

        if not aligned:
            goal_handle.abort()
            result = DoLandmarkTask.Result()
            result.success = False
            result.detected_tag_id = detected_tag_id
            result.message = "Failed to align with AprilTag"
            return result

        feedback.status = f"Executing behavior for tag {detected_tag_id}"
        goal_handle.publish_feedback(feedback)

        success, message = self.execute_behavior_for_tag(detected_tag_id)

        result = DoLandmarkTask.Result()
        result.success = success
        result.detected_tag_id = detected_tag_id
        result.message = message

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def scan_for_apriltag(self):
        """
        Replace this with:
        - subscribing to your perception topic, or
        - calling a ScanForAprilTag action/service.
        """
        self.get_logger().info("TODO: scan_for_apriltag")
        return 3

    def align_with_apriltag(self, tag_id: int) -> bool:
        """
        Replace this with your align action client.
        """
        self.get_logger().info(f"TODO: align_with_apriltag({tag_id})")
        return True

    def execute_behavior_for_tag(self, tag_id: int):
        if tag_id == 0:
            return self.handle_start_tag()

        if tag_id == 1:
            return self.handle_outward_door()

        if tag_id == 2:
            return self.handle_inward_door()

        if 3 <= tag_id <= 10:
            return self.handle_normal_landmark(tag_id)

        return False, f"Unknown tag id {tag_id}"

    def handle_start_tag(self):
        self.get_logger().info("Handling start tag")
        return True, "Start tag handled"

    def handle_outward_door(self):
        self.get_logger().info("Handling outward-opening door")

        # Later:
        # 1. wait for someone to open door
        # 2. traverse doorway

        return True, "Outward door handled"

    def handle_inward_door(self):
        self.get_logger().info("Handling inward-opening door")

        # Later:
        # 1. wait for someone to open door
        # 2. traverse doorway

        return True, "Inward door handled"

    def handle_normal_landmark(self, tag_id: int):
        self.get_logger().info(f"Handling normal landmark tag {tag_id}")

        # Later:
        # play tune associated with this tag

        return True, f"Landmark {tag_id} handled"


def main(args=None):
    rclpy.init(args=args)

    node = LandmarkTaskServer()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()