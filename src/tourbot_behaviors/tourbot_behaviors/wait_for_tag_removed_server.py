import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from tourbot_interfaces.action import WaitForTagRemoved

#TODO: Add beeping sound to alert nearby humans for help 

class WaitForTagRemovedServer(Node):
    """Action server that waits for a specified AprilTag to be removed from the camera's field of view for a certain duration, with a timeout.
    """
    def __init__(self):
        super().__init__("wait_for_tag_removed_server")

        self.cb_group = ReentrantCallbackGroup() # Use a reentrant callback group to allow concurrent execution of callbacks

        # Server parameters
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("control_rate_hz", 20.0)

        detections_topic = self.get_parameter("detections_topic").value
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.latest_detections: Optional[AprilTagDetectionArray] = None

        # Subscribe to AprilTag detections to keep track of which tags are currently visible.
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self.detections_callback,
            10,
            callback_group=self.cb_group,
        )

        # Create the action server for WaitForTagRemoved.
        self.action_server = ActionServer(
            self,
            WaitForTagRemoved,
            "wait_for_tag_removed",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # Debug info
        self.get_logger().info("WaitForTagRemoved action server ready.")
        #self.get_logger().info(f"Listening for detections on: {detections_topic}")

    # Callback for receiving AprilTag detections
    def detections_callback(self, msg: AprilTagDetectionArray):
        self.latest_detections = msg

    # Action server callbacks
    def goal_callback(self, goal_request: WaitForTagRemoved.Goal) -> int:
        self.get_logger().info(
            f"Received wait-for-tag-removed goal for tag_id={goal_request.tag_id}"
        )

        # Validate the goal parameters before accepting it.
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

    # Callback for handling cancellation requests
    def cancel_callback(self, goal_handle) -> int:
        self.get_logger().info("Received request to cancel wait-for-tag-removed.")
        return CancelResponse.ACCEPT

    # Determine if the specified tag_id is currently visible in the latest detections.
    def tag_is_visible(self, tag_id: int) -> bool:
        if self.latest_detections is None:
            return False

        for detection in self.latest_detections.detections:
            if int(detection.id) == int(tag_id):
                return True

        return False

    # Helper function to get a list of currently visible tag IDs for logging/debugging purposes.
    def get_visible_tag_ids(self):
        if self.latest_detections is None:
            return []
    
        return [int(detection.id) for detection in self.latest_detections.detections]

    # Publish feedback about the current visibility state of the tag and how long it has been missing if applicable.
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

    # Main execution callback for the action server. Waits for the specified tag to be removed from view for the required duration, or until timeout/cancellation.
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = WaitForTagRemoved.Result()

        self.get_logger().info(
            f"Waiting for AprilTag {goal.tag_id} to be removed from FOV."
        )

        # Document start time to enforce the overall timeout, and track how long the tag has been missing.
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

            # If the timeout time has been exceeded, abort the action and return failure with a message. 
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

            # Tag is currently visible, reset missing timer and publish feedback. 
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

            # Tag is currently not visible and this is the first time we've seen it missing.
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

            # Count how long the tag has been missing and if it exceeds the required missing duration, succeed the action. 
            # Otherwise, keep waiting and publishing feedback.
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

        # rclpy is not okay, ROS is getting shut down.
        goal_handle.abort()

        result.success = False
        result.message = "ROS shutdown during WaitForTagRemoved."
        return result

# Node entry point
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

# Allow running the node directly with `python wait_for_tag_removed_server.py`
if __name__ == "__main__":
    main()