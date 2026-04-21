import rclpy
from rclpy.node import Node

from .nav_to_pose_client import NavToPoseClient

class TourDeliberationNode(Node):
    def __init__(self) -> None:
        super().__init__('tour_deliberation_node')

        self.landmarks = [
            {"name": "stop_1", "x": 1.0, "y": 0.5, "yaw": 0.0},
            {"name": "stop_2", "x": 20.0, "y": -10.0, "yaw": 1.57},
        ]

        self.nav_client = NavToPoseClient()
        self.goal_handle = None
        self.result_future = None

        self.timer = self.create_timer(1.0, self.start_once)
        self.started = False

    def start_once(self) -> None:
        if self.started:
            return
        self.started = True

        if not self.nav_client.wait_for_server():
            self.get_logger().error('Nav2 action server not available.')
            rclpy.shutdown()
            return

        target = self.landmarks[0]
        self.get_logger().info(f'Selected landmark: {target["name"]}')

        send_goal_future = self.nav_client.send_goal(
            target["x"],
            target["y"],
            target["yaw"],
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted.')
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future) -> None:
        result = future.result()
        status = result.status

        self.get_logger().info(f'Navigation finished with status code: {status}')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TourDeliberationNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.nav_client)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        node.nav_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()