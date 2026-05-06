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


# Constants for alignment behavior
SEARCH_ANGULAR_SPEED = 0.20
ALIGN_KP = 0.003
MAX_ANGULAR_SPEED = 0.30

class AlignToAprilTagServer(Node):
    """ ROS 2 action server that aligns the robot to a specified AprilTag by rotating in place until the tag is centered in the camera's field of view.

    Subscribes to AprilTag detections and camera info to determine the position of the tag in the image. 
    Publishes velocity commands to rotate the robot until the tag is aligned within a specified pixel tolerance.
    """

    def __init__(self):
        super().__init__("align_to_apriltag_server")

        # Use a ReentrantCallbackGroup to allow simultaneous processing of action goals and incoming sensor data.
        self.cb_group = ReentrantCallbackGroup()

        # Declare parameters for topic names and control settings.
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("camera_info_topic", "/oakd/rgb/preview/camera_info")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        detections_topic = self.get_parameter("detections_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        # Initialize state variables for latest detections and camera info.
        self.latest_detections: Optional[AprilTagDetectionArray] = None
        self.image_width: Optional[int] = None

        # Subscribe to AprilTag detections and camera info.
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

        # Publisher for velocity commands to control the robot's rotation.
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            cmd_vel_topic,
            10,
        )

        # Create the action server for aligning to an AprilTag.
        self.action_server = ActionServer(
            self,
            AlignToAprilTag,
            "align_to_apriltag",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # Debug info
        self.get_logger().info("AlignToAprilTag action server ready.")
        #self.get_logger().info(f"Listening for detections on: {detections_topic}")
        #self.get_logger().info(f"Listening for camera info on: {camera_info_topic}")
        #self.get_logger().info(f"Publishing velocity commands on: {cmd_vel_topic}")

    # Callback for receiving AprilTag detections
    def detections_callback(self, msg: AprilTagDetectionArray):
        self.latest_detections = msg

    # Callback for receiving camera info 
    def camera_info_callback(self, msg: CameraInfo):
        self.image_width = msg.width

    # Callback for receiving action goals
    def goal_callback(self, goal_request):
        # Parameter validation for incoming goals
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

        self.get_logger().info(
            f"Accepted alignment goal for tag_id={goal_request.tag_id}"
        )
        return GoalResponse.ACCEPT

    # Callback for handling action cancellation requests
    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received request to cancel alignment.")
        self.stop_robot()
        return CancelResponse.ACCEPT

    # Helper method to create a TwistStamped message with the current timestamp and frame_id set to "base_link".
    def make_twist_stamped(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    # Helper method to stop the robot by publishing a zero velocity command.
    def stop_robot(self):
        msg = self.make_twist_stamped()
        self.cmd_vel_pub.publish(msg)

    # Helper method to publish an angular velocity command to rotate the robot.
    def publish_angular_velocity(self, angular_z: float):
        msg = self.make_twist_stamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    # Helper method to find a specific AprilTag detection by its ID from the latest detections published.
    def find_detection_by_id(self, tag_id: int):
        if self.latest_detections is None:
            return None

        # Iterate through the latest detections to find one that matches the specified tag_id.
        for detection in self.latest_detections.detections:
            if int(detection.id) == int(tag_id):
                return detection

        return None

    # Helper method to clamp a value between a specified low and high range.
    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    # Helper method to publish feedback for the action server. 
    def publish_feedback(
        self,
        goal_handle,
        tag_visible: bool,
        x_error_px: float,
        state: str,
    ):
        feedback = AlignToAprilTag.Feedback()
        feedback.tag_visible = bool(tag_visible)
        feedback.x_error_px = float(x_error_px)
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    # Main execution loop for the action server. Continuously checks for the specified AprilTag and publishes velocity commands to align the robot until the tag is centered within the specified tolerance or a timeout occurs.
    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        result = AlignToAprilTag.Result() # Initialize the result message

        start_time = time.monotonic()
        rate_hz = 20.0
        sleep_time = 1.0 / rate_hz

        self.get_logger().info(f"Aligning to AprilTag id={goal.tag_id}")

        while rclpy.ok():
            # Check for cancellation or timeout conditions at the start of each loop iteration.
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

            # Check if camera info is available, if not publish feedback and wait for it to arrive.
            if self.image_width is None:
                self.publish_feedback(
                    goal_handle,
                    tag_visible=False,
                    x_error_px=0.0,
                    state="waiting_for_camera_info",
                )

                self.stop_robot()
                time.sleep(sleep_time)
                continue

            # Look for the specified tag_id in the latest detections.
            detection = self.find_detection_by_id(goal.tag_id)

            # If the tag is not currently detected, publish feedback and rotate in place to search for it.
            if detection is None:
                self.publish_feedback(
                    goal_handle,
                    tag_visible=False,
                    x_error_px=0.0,
                    state="searching_for_tag",
                )

                self.publish_angular_velocity(SEARCH_ANGULAR_SPEED)
                time.sleep(sleep_time)
                continue

            image_center_x = self.image_width / 2.0 # Get the center x-coordinate of the image from camera info
            tag_center_x = float(detection.centre.x) # Get the x-coordinate of the center of the detected tag from the AprilTag detection message 
            x_error = tag_center_x - image_center_x # Calculate the error in pixels between the tag center and the image center

            # If the tag is detected and the x error is within the specified tolerance, stop the robot and succeed the action.
            if abs(x_error) <= goal.x_tolerance_px:
                self.stop_robot()

                self.publish_feedback(
                    goal_handle,
                    tag_visible=True,
                    x_error_px=x_error,
                    state="aligned",
                )

                goal_handle.succeed()

                result.success = True
                result.message = f"Aligned to tag {goal.tag_id}."
                return result

            # Otherwise, if the tag is detected but not yet aligned, publish feedback and continue rotating to reduce the error.
            self.publish_feedback(
                goal_handle,
                tag_visible=True,
                x_error_px=x_error,
                state="aligning",
            )

            angular_z = -ALIGN_KP * x_error
            angular_z = self.clamp(
                angular_z,
                -MAX_ANGULAR_SPEED,
                MAX_ANGULAR_SPEED,
            )

            self.publish_angular_velocity(angular_z)
            time.sleep(sleep_time)

        # rclpy is no longer ok, ROS shut down
        self.stop_robot()
        goal_handle.abort()

        result.success = False
        result.message = "ROS shutdown during alignment."
        return result


# Node entry point
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