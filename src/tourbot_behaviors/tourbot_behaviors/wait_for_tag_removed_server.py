import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from tourbot_interfaces.action import WaitForTagRemoved


class WaitForTagRemovedServer(Node):
    def __init__(self):
        super().__init__("wait_for_tag_removed_server")

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("control_rate_hz", 20.0)

        detections_topic = self.get_parameter("detections_topic").value
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.latest_detections: Optional[AprilTagDetectionArray] = None

        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self.detections_callback,
            10,
            callback_group=self.cb_group,
        )

        self.action_server = ActionServer(
            self,
            WaitForTagRemoved,
            "wait_for_tag_removed",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("WaitForTagRemoved action server ready.")
        self.get_logger().info(f"Listening for detections on: {detections_topic}")

    def detections_callback(self, msg: AprilTagDetectionArray):
        self.latest_detections = msg

    def goal_callback(self, goal_request: WaitForTagRemoved.Goal) -> int:
        self.get_logger().info(
            f"Received wait-for-tag-removed goal for tag_id={goal_request.tag_id}"
        )

        if goal_request.timeout_sec <= 0.0:
            self.get_logger().warn(
                "Rejected goal: timeout_sec must be positive."
            )
            return GoalResponse.REJECT

        if goal_request.missing_duration_sec <= 0.0:
            self.get_logger().warn(
                "Rejected goal: missing_duration_sec must be positive."
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> int:
        self.get_logger().info("Received request to cancel wait-for-tag-removed.")
        return CancelResponse.ACCEPT

    def tag_is_visible(self, tag_id: int) -> bool:
        if self.latest_detections is None:
            return False

        for detection in self.latest_detections.detections:
            if int(detection.id) == int(tag_id):
                return True

        return False

    def get_visible_tag_ids(self):
        if self.latest_detections is None:
            return []

        return [int(detection.id) for detection in self.latest_detections.detections]

    def publish_feedback(
        self,
        goal_handle,
        tag_visible: bool,
        missing_time_sec: float,
        state: str,
    ) -> None:
        feedback = WaitForTagRemoved.Feedback()
        feedback.tag_visible = bool(tag_visible)
        feedback.missing_time_sec = float(missing_time_sec)
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = WaitForTagRemoved.Result()

        self.get_logger().info(
            f"Waiting for AprilTag {goal.tag_id} to be removed from FOV."
        )

        start_time = time.monotonic()
        missing_start_time = None
        sleep_dt = 1.0 / self.control_rate_hz

        last_log_time = 0.0
        last_state = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result.success = False
                result.message = "WaitForTagRemoved canceled."
                return result

            now = time.monotonic()
            elapsed = now - start_time

            if elapsed > goal.timeout_sec:
                goal_handle.abort()

                visible_ids = self.get_visible_tag_ids()
                result.success = False
                result.message = (
                    f"Timed out waiting for tag {goal.tag_id} to be removed. "
                    f"Currently visible tags: {visible_ids}"
                )
                return result

            visible = self.tag_is_visible(goal.tag_id)

            if visible:
                missing_start_time = None
                missing_time = 0.0
                state = "TAG_VISIBLE"

                self.publish_feedback(
                    goal_handle,
                    tag_visible=True,
                    missing_time_sec=missing_time,
                    state=state,
                )

                if state != last_state or now - last_log_time > 1.0:
                    self.get_logger().info(
                        f"Tag {goal.tag_id} still visible. "
                        f"Visible tags: {self.get_visible_tag_ids()}"
                    )
                    last_state = state
                    last_log_time = now

                time.sleep(sleep_dt)
                continue

            if missing_start_time is None:
                missing_start_time = now

            missing_time = now - missing_start_time
            state = "TAG_MISSING"

            self.publish_feedback(
                goal_handle,
                tag_visible=False,
                missing_time_sec=missing_time,
                state=state,
            )

            if state != last_state or now - last_log_time > 0.5:
                self.get_logger().info(
                    f"Tag {goal.tag_id} missing for {missing_time:.2f}s / "
                    f"{goal.missing_duration_sec:.2f}s required."
                )
                last_state = state
                last_log_time = now

            if missing_time >= goal.missing_duration_sec:
                goal_handle.succeed()

                result.success = True
                result.message = (
                    f"Tag {goal.tag_id} removed from FOV for "
                    f"{missing_time:.2f} seconds."
                )
                return result

            time.sleep(sleep_dt)

        goal_handle.abort()

        result.success = False
        result.message = "ROS shutdown during WaitForTagRemoved."
        return result


def main(args=None):
    rclpy.init(args=args)

    node = WaitForTagRemovedServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()