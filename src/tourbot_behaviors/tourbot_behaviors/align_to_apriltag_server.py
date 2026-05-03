import math
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CameraInfo

from apriltag_msgs.msg import AprilTagDetectionArray
from tourbot_interfaces.action import AlignToAprilTag


SEARCH_ANGULAR_SPEED = 0.20

CENTER_ALIGN_KP = 0.003
PERPENDICULAR_ALIGN_KP = 0.010

MAX_ANGULAR_SPEED = 0.30

# If the tag center is within this many pixels, we switch from centering to perpendicular alignment.
CENTER_SWITCH_TOLERANCE_PX = 25.0

# Difference between left and right tag-side heights, in pixels.
PERPENDICULAR_TOLERANCE_PX = 4.0


class AlignToAprilTagServer(Node):
    def __init__(self):
        super().__init__("align_to_apriltag_server")

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("camera_info_topic", "/oakd/rgb/preview/camera_info")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        detections_topic = self.get_parameter("detections_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.latest_detections: Optional[AprilTagDetectionArray] = None
        self.image_width: Optional[int] = None

        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self.detections_callback,
            10,
            callback_group=self.cb_group,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10,
            callback_group=self.cb_group,
        )

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            cmd_vel_topic,
            10,
        )

        self._action_server = ActionServer(
            self,
            AlignToAprilTag,
            "align_to_apriltag",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("AlignToAprilTag action server ready.")
        self.get_logger().info(f"Listening for detections on: {detections_topic}")
        self.get_logger().info(f"Listening for camera info on: {camera_info_topic}")
        self.get_logger().info(f"Publishing velocity commands on: {cmd_vel_topic}")

    def detections_callback(self, msg: AprilTagDetectionArray):
        self.latest_detections = msg

    def camera_info_callback(self, msg: CameraInfo):
        self.image_width = msg.width

    def goal_callback(self, goal_request: AlignToAprilTag.Goal) -> int:
        self.get_logger().info(
            f"Received alignment goal request for tag_id={goal_request.tag_id}"
        )

        if goal_request.timeout_sec <= 0.0:
            self.get_logger().warn(
                "Rejected alignment goal: timeout_sec must be positive."
            )
            return GoalResponse.REJECT

        if goal_request.x_tolerance_px <= 0.0:
            self.get_logger().warn(
                "Rejected alignment goal: x_tolerance_px must be positive."
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> int:
        self.get_logger().info("Received request to cancel alignment.")
        self.stop_robot()
        return CancelResponse.ACCEPT

    def make_twist_stamped(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def stop_robot(self) -> None:
        msg = self.make_twist_stamped()
        self.cmd_vel_pub.publish(msg)

    def publish_angular_velocity(self, angular_z: float) -> None:
        msg = self.make_twist_stamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def find_detection_by_id(self, tag_id: int):
        if self.latest_detections is None:
            return None

        for detection in self.latest_detections.detections:
            if int(detection.id) == int(tag_id):
                return detection

        return None

    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def pixel_distance(self, p1, p2) -> float:
        dx = float(p1.x) - float(p2.x)
        dy = float(p1.y) - float(p2.y)
        return math.sqrt(dx * dx + dy * dy)

    def get_side_height_error(self, detection) -> float:
        """
        Corners from apriltag_msgs are ordered around the tag.

        From your echo, the order looked like:
        corners[0] = bottom-left-ish
        corners[1] = bottom-right-ish
        corners[2] = top-right-ish
        corners[3] = top-left-ish

        left_height  = distance(corner 0, corner 3)
        right_height = distance(corner 1, corner 2)

        If these are equal, the tag is roughly front-facing/perpendicular.
        """
        corners = detection.corners

        left_height = self.pixel_distance(corners[0], corners[3])
        right_height = self.pixel_distance(corners[1], corners[2])

        return left_height - right_height

    def publish_feedback(
        self,
        goal_handle,
        tag_visible: bool,
        x_error_px: float,
        state: str,
    ) -> None:
        feedback = AlignToAprilTag.Feedback()
        feedback.x_error_px = float(x_error_px)
        feedback.tag_visible = bool(tag_visible)
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing AprilTag perpendicular alignment action.")

        goal = goal_handle.request
        result = AlignToAprilTag.Result()

        start_time = time.monotonic()
        sleep_dt = 1.0 / 20.0

        last_state = None
        last_log_time = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()

                result.success = False
                result.message = "Alignment canceled."
                return result

            elapsed = time.monotonic() - start_time

            if elapsed > goal.timeout_sec:
                self.stop_robot()
                goal_handle.abort()

                result.success = False
                result.message = f"Timed out while aligning to tag {goal.tag_id}."
                return result

            now = time.monotonic()

            if self.image_width is None:
                state = "WAITING_FOR_CAMERA_INFO"

                self.publish_feedback(
                    goal_handle,
                    tag_visible=False,
                    x_error_px=0.0,
                    state=state,
                )

                if state != last_state or now - last_log_time > 1.0:
                    self.get_logger().info(
                        "Waiting for camera info. Check /oakd/rgb/preview/camera_info."
                    )
                    last_state = state
                    last_log_time = now

                self.stop_robot()
                time.sleep(sleep_dt)
                continue

            detection = self.find_detection_by_id(goal.tag_id)

            if detection is None:
                state = "SEARCHING_FOR_TAG"

                self.publish_feedback(
                    goal_handle,
                    tag_visible=False,
                    x_error_px=0.0,
                    state=state,
                )

                if state != last_state or now - last_log_time > 1.0:
                    self.get_logger().info(
                        f"Searching for tag {goal.tag_id}. Image width={self.image_width}."
                    )
                    last_state = state
                    last_log_time = now

                self.publish_angular_velocity(SEARCH_ANGULAR_SPEED)
                time.sleep(sleep_dt)
                continue

            image_center_x = self.image_width / 2.0
            tag_center_x = float(detection.centre.x)
            center_error = tag_center_x - image_center_x

            side_height_error = self.get_side_height_error(detection)

            # Step 1: center the tag horizontally first.
            if abs(center_error) > CENTER_SWITCH_TOLERANCE_PX:
                state = "CENTERING_TAG"

                angular_z = -CENTER_ALIGN_KP * center_error
                angular_z = self.clamp(
                    angular_z,
                    -MAX_ANGULAR_SPEED,
                    MAX_ANGULAR_SPEED,
                )

                self.publish_feedback(
                    goal_handle,
                    tag_visible=True,
                    x_error_px=center_error,
                    state=state,
                )

                if state != last_state or now - last_log_time > 0.5:
                    self.get_logger().info(
                        f"Centering tag {goal.tag_id}: "
                        f"center_error={center_error:.2f}px, "
                        f"angular_z={angular_z:.3f}"
                    )
                    last_state = state
                    last_log_time = now

                self.publish_angular_velocity(angular_z)
                time.sleep(sleep_dt)
                continue

            # Step 2: rotate until tag appears front-facing/perpendicular.
            if abs(side_height_error) <= PERPENDICULAR_TOLERANCE_PX:
                self.stop_robot()

                self.publish_feedback(
                    goal_handle,
                    tag_visible=True,
                    x_error_px=center_error,
                    state="PERPENDICULAR_ALIGNED",
                )

                self.get_logger().info(
                    f"Perpendicular aligned to tag {goal.tag_id}. "
                    f"center_error={center_error:.2f}px, "
                    f"side_height_error={side_height_error:.2f}px"
                )

                goal_handle.succeed()

                result.success = True
                result.message = f"Perpendicular aligned to tag {goal.tag_id}."
                return result

            state = "ALIGNING_PERPENDICULAR"

            # Positive error means left side appears taller than right side.
            # The sign may need flipping depending on camera/robot convention.
            angular_z = -PERPENDICULAR_ALIGN_KP * side_height_error
            angular_z = self.clamp(
                angular_z,
                -MAX_ANGULAR_SPEED,
                MAX_ANGULAR_SPEED,
            )

            self.publish_feedback(
                goal_handle,
                tag_visible=True,
                x_error_px=center_error,
                state=state,
            )

            if state != last_state or now - last_log_time > 0.5:
                self.get_logger().info(
                    f"Perpendicular aligning tag {goal.tag_id}: "
                    f"center_error={center_error:.2f}px, "
                    f"side_height_error={side_height_error:.2f}px, "
                    f"angular_z={angular_z:.3f}"
                )
                last_state = state
                last_log_time = now

            self.publish_angular_velocity(angular_z)
            time.sleep(sleep_dt)

        self.stop_robot()
        goal_handle.abort()

        result.success = False
        result.message = "ROS shutdown during alignment."
        return result


def main(args=None):
    rclpy.init(args=args)

    node = AlignToAprilTagServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()