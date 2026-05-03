import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo

from apriltag_msgs.msg import AprilTagDetectionArray
from tourbot_interfaces.action import AlignToAprilTag


class AlignToAprilTagServer(Node):
    def __init__(self):
        super().__init__("align_to_apriltag_server")

        self.declare_parameter("detections_topic", "/apriltag/detections")
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
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.action_server = ActionServer(
            self,
            AlignToAprilTag,
            "align_to_apriltag",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("AlignToAprilTag action server ready")

    def detections_callback(self, msg: AprilTagDetectionArray):
        self.latest_detections = msg

    def camera_info_callback(self, msg: CameraInfo):
        self.image_width = msg.width

    def goal_callback(self, goal_request):
        if goal_request.timeout_sec <= 0.0:
            self.get_logger().warn("Rejected alignment goal: timeout_sec must be positive")
            return GoalResponse.REJECT

        if goal_request.x_tolerance_px <= 0.0:
            self.get_logger().warn("Rejected alignment goal: x_tolerance_px must be positive")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.stop_robot()
        return CancelResponse.ACCEPT

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def find_detection_by_id(self, tag_id: int):
        if self.latest_detections is None:
            return None

        for detection in self.latest_detections.detections:
            if int(detection.id) == int(tag_id):
                return detection

        return None

    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        result = AlignToAprilTag.Result()
        feedback = AlignToAprilTag.Feedback()

        start_time = time.monotonic()
        rate_hz = 20.0
        sleep_time = 1.0 / rate_hz

        self.get_logger().info(f"Aligning to AprilTag id={goal.tag_id}")

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = "Alignment canceled"
                return result

            elapsed = time.monotonic() - start_time
            if elapsed > goal.timeout_sec:
                self.stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = f"Timed out while aligning to tag {goal.tag_id}"
                return result

            if self.image_width is None:
                feedback.tag_visible = False
                feedback.x_error_px = 0.0
                feedback.state = "waiting_for_camera_info"
                goal_handle.publish_feedback(feedback)
                self.stop_robot()
                time.sleep(sleep_time)
                continue

            detection = self.find_detection_by_id(goal.tag_id)

            twist = Twist()

            if detection is None:
                feedback.tag_visible = False
                feedback.x_error_px = 0.0
                feedback.state = "searching_for_tag"
                goal_handle.publish_feedback(feedback)

                twist.angular.z = float(goal.search_angular_speed)
                self.cmd_vel_pub.publish(twist)

                time.sleep(sleep_time)
                continue

            image_center_x = self.image_width / 2.0
            tag_center_x = float(detection.centre.x)
            x_error = tag_center_x - image_center_x

            feedback.tag_visible = True
            feedback.x_error_px = float(x_error)

            if abs(x_error) <= goal.x_tolerance_px:
                self.stop_robot()

                feedback.state = "aligned"
                goal_handle.publish_feedback(feedback)

                goal_handle.succeed()
                result.success = True
                result.message = f"Aligned to tag {goal.tag_id}"
                return result

            feedback.state = "aligning"
            goal_handle.publish_feedback(feedback)

            angular_z = -goal.align_kp * x_error
            angular_z = self.clamp(
                angular_z,
                -goal.max_angular_speed,
                goal.max_angular_speed,
            )

            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)

            time.sleep(sleep_time)

        self.stop_robot()
        goal_handle.abort()
        result.success = False
        result.message = "rclpy shutdown during alignment"
        return result


def main(args=None):
    rclpy.init(args=args)

    node = AlignToAprilTagServer()

    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()