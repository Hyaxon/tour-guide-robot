import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class NavToPoseClient(Node):
    def __init__(self) -> None:
        super().__init__('nav_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wait_for_server(self) -> bool:
        self.get_logger().info('Waiting for Nav2 action server...')
        return self._action_client.wait_for_server(timeout_sec=10.0)

    def make_goal(self, x: float, y: float, yaw: float, frame_id: str = 'map') -> NavigateToPose.Goal:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quaternion(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def send_goal(self, x: float, y: float, yaw: float):
        goal_msg = self.make_goal(x, y, yaw)
        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        return self._action_client.send_goal_async(goal_msg)