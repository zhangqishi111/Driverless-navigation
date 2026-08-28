#!/usr/bin/env python3
"""Bridge the project-level /goal_pose topic to Nav2 NavigateToPose actions."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._status = self.create_publisher(String, '/navigation_status', 10)
        self._goal_handle = None
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status.publish(message)
        self.get_logger().info(text)

    def _on_goal(self, pose: PoseStamped) -> None:
        if pose.header.frame_id != 'map':
            self._publish_status('rejected: /goal_pose frame_id must be map')
            return
        if self._goal_handle is not None:
            self._publish_status('rejected: a navigation goal is already active')
            return
        if not self._client.server_is_ready():
            self._publish_status('rejected: NavigateToPose action server is not ready')
            return

        request = NavigateToPose.Goal()
        request.pose = pose
        self._publish_status('planning')
        future = self._client.send_goal_async(request, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status('rejected: Nav2 did not accept the goal')
            return
        self._goal_handle = goal_handle
        self._publish_status('navigating')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, _feedback_message) -> None:
        # /plan and /local_plan remain the visual source of truth for D.
        return

    def _on_result(self, future) -> None:
        status = future.result().status
        if status == 4:
            self._publish_status('arrived')
        elif status == 5:
            self._publish_status('cancelled')
        else:
            self._publish_status(f'failed: NavigateToPose status={status}')
        self._goal_handle = None


def main() -> None:
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
