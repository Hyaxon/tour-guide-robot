import math
from math import atan2
import time
from typing import Optional, Tuple

#from geometry_msgs import msg
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from tourbot_interfaces.action import DoorTraverse

turn_speed_default = 0.8

class DoorBehaviorServer(Node):
    def __init__(self) -> None:
        super().__init__('door_behavior_server')

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')

        self.declare_parameter('backup_distance_default', 0.9)
        self.declare_parameter('backup_speed_default', 0.15)
        self.declare_parameter('wait_seconds_default', 3.0)
        self.declare_parameter('forward_distance_default', 1.5)
        self.declare_parameter('forward_speed_default', 0.18)
        #self.declare_parameter('turn_speed_default', 0.8)

        self.declare_parameter('control_rate_hz', 20.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.backup_distance_default = float(self.get_parameter('backup_distance_default').value)
        self.backup_speed_default = float(self.get_parameter('backup_speed_default').value)
        self.wait_seconds_default = float(self.get_parameter('wait_seconds_default').value)
        self.forward_distance_default = float(self.get_parameter('forward_distance_default').value)
        self.forward_speed_default = float(self.get_parameter('forward_speed_default').value)

        #self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_cb,
            10,
            callback_group=self.cb_group
        )

        self.current_odom: Optional[Odometry] = None
        self.current_yaw: Optional[float] = None

        self._action_server = ActionServer(
            self,
            DoorTraverse,
            'door_traverse',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Door behavior server ready.')

    def goal_callback(self, goal_request: DoorTraverse.Goal) -> int:
        self.get_logger().info('Received door traversal goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> int:
        self.get_logger().info('Received request to cancel door traversal.')
        return CancelResponse.ACCEPT

    def odom_cb(self, msg: Odometry) -> None:
        self.current_odom = msg

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_feedback(self, goal_handle, state: str, distance: float) -> None:
        feedback = DoorTraverse.Feedback()
        feedback.current_state = state
        feedback.distance_traveled = float(distance)
        goal_handle.publish_feedback(feedback)

    def stop_robot(self) -> None:
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)

    def set_linear_velocity(self, speed: float) -> None:
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(speed)
        self.cmd_pub.publish(msg)

    def get_xy(self) -> Optional[Tuple[float, float]]:
        if self.current_odom is None:
            return None
        pos = self.current_odom.pose.pose.position
        return (float(pos.x), float(pos.y))

    def distance_from(self, start_xy: Tuple[float, float]) -> Optional[float]:
        current_xy = self.get_xy()
        if current_xy is None:
            return None
        dx = current_xy[0] - start_xy[0]
        dy = current_xy[1] - start_xy[1]
        return math.sqrt(dx * dx + dy * dy)

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


    def angle_diff(self, target: float, current: float) -> float:
        return self.normalize_angle(target - current)

    def wait_with_cancel(self, seconds: float, goal_handle, state_name: str) -> Tuple[bool, str]:
        start_time = time.monotonic()
        sleep_dt = 1.0 / self.control_rate_hz

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return False, 'Canceled'

            elapsed = time.monotonic() - start_time
            self.publish_feedback(goal_handle, state_name, 0.0)

            if elapsed >= seconds:
                return True, 'Wait complete'

            time.sleep(sleep_dt)

        self.stop_robot()
        return False, 'ROS shutdown during wait'

    def drive_linear_distance(
        self,
        speed: float,
        target_distance: float,
        goal_handle,
        state_name: str
    ) -> Tuple[bool, str]:
        start_xy = self.get_xy()
        if start_xy is None:
            self.stop_robot()
            return False, 'No odometry available'

        sleep_dt = 1.0 / self.control_rate_hz

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return False, 'Canceled'

            distance = self.distance_from(start_xy)
            if distance is None:
                self.stop_robot()
                return False, 'Lost odometry during motion'

            self.publish_feedback(goal_handle, state_name, distance)

            if distance >= target_distance:
                self.stop_robot()
                return True, 'Motion complete'

            self.set_linear_velocity(speed)
            time.sleep(sleep_dt)

        self.stop_robot()
        return False, 'ROS shutdown during motion'

    def set_angular_velocity(self, speed: float) -> None:
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = float(speed)
        self.cmd_pub.publish(msg)

    def turn_relative_angle(
        self,
        angle_rad: float,
        angular_speed: float,
        goal_handle,
        state_name: str
    ) -> Tuple[bool, str]:
        if self.current_yaw is None:
            self.stop_robot()
            return False, 'No odometry yaw available'

        start_yaw = self.current_yaw
        target_yaw = self.normalize_angle(start_yaw + angle_rad)

        sleep_dt = 1.0 / self.control_rate_hz

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return False, 'Canceled'

            if self.current_yaw is None:
                self.stop_robot()
                return False, 'Lost odometry yaw during turn'

            error = self.angle_diff(target_yaw, self.current_yaw)

            self.publish_feedback(goal_handle, state_name, abs(error))

            if abs(error) < 0.05:
                self.stop_robot()
                return True, 'Turn complete'

            turn_direction = 1.0 if error > 0.0 else -1.0
            self.set_angular_velocity(turn_direction * abs(angular_speed))

            time.sleep(sleep_dt)

        self.stop_robot()
        return False, 'ROS shutdown during turn'

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing door traversal action.')

        goal = goal_handle.request

        backup_distance = goal.backup_distance if goal.backup_distance > 0.0 else self.backup_distance_default
        backup_speed = goal.backup_speed if goal.backup_speed > 0.0 else self.backup_speed_default
        wait_seconds = goal.wait_seconds if goal.wait_seconds > 0.0 else self.wait_seconds_default
        forward_distance = goal.forward_distance if goal.forward_distance > 0.0 else self.forward_distance_default
        forward_speed = goal.forward_speed if goal.forward_speed > 0.0 else self.forward_speed_default

        try:
            # Turn around so the robot can move "backward" using forward motion.
            ok, msg = self.turn_relative_angle(
                angle_rad=math.pi,
                angular_speed=turn_speed_default,
                goal_handle=goal_handle,
                state_name='TURNING_AROUND_TO_BACK_UP'
            )
            if not ok:
                result = DoorTraverse.Result()
                result.success = False
                result.message = msg
                return result

            # Drive forward while facing away from the door.
            ok, msg = self.drive_linear_distance(
                speed=abs(backup_speed),
                target_distance=backup_distance,
                goal_handle=goal_handle,
                state_name='DRIVING_AWAY_FROM_DOOR'
            )
            if not ok:
                result = DoorTraverse.Result()
                result.success = False
                result.message = msg
                return result

            # Turn back to face the original door direction.
            ok, msg = self.turn_relative_angle(
                angle_rad=math.pi,
                angular_speed=turn_speed_default,
                goal_handle=goal_handle,
                state_name='TURNING_BACK_TO_DOOR'
            )
            if not ok:
                result = DoorTraverse.Result()
                result.success = False
                result.message = msg
                return result

            ok, msg = self.wait_with_cancel(wait_seconds, goal_handle, 'PAUSING')
            if not ok:
                result = DoorTraverse.Result()
                result.success = False
                result.message = msg
                return result

            ok, msg = self.drive_linear_distance(
                speed=abs(forward_speed),
                target_distance=forward_distance,
                goal_handle=goal_handle,
                state_name='DRIVING_FORWARD'
            )
            if not ok:
                result = DoorTraverse.Result()
                result.success = False
                result.message = msg
                return result

            self.stop_robot()
            goal_handle.succeed()

            result = DoorTraverse.Result()
            result.success = True
            result.message = 'Door traversal complete'
            return result

        except Exception as exc:
            self.stop_robot()
            self.get_logger().error(f'Exception in door traversal: {exc}')
            result = DoorTraverse.Result()
            result.success = False
            result.message = f'Exception: {exc}'
            return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoorBehaviorServer()
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