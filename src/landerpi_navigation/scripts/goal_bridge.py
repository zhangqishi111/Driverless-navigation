#!/usr/bin/env python3
"""Bridge project goals to Nav2 and expose navigation-result telemetry."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._status = self.create_publisher(String, '/navigation_status', 10)
        last_value_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # These topics are machine-readable and independent from the
        # human-facing /navigation_status text stream.
        self._arrival_status = self.create_publisher(
            Bool, '/arrival_status', last_value_qos)
        self._position_error = self.create_publisher(
            Float32, '/position_error', last_value_qos)
        self._goal_handle = None
        self._goal_pending = False
        self._goal_pose = None
        self._robot_pose = None
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self._on_robot_pose, 10)
        self.create_timer(0.2, self._publish_position_error)
        self._publish_arrival(False)

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status.publish(message)
        self.get_logger().info(text)

    def _publish_arrival(self, arrived: bool) -> None:
        message = Bool()
        message.data = arrived
        self._arrival_status.publish(message)

    def _on_robot_pose(self, pose: PoseStamped) -> None:
        if pose.header.frame_id != 'map':
            self.get_logger().warning(
                f'ignoring /robot_pose in {pose.header.frame_id!r}; expected map')
            return
        self._robot_pose = pose
        self._publish_position_error()

    def _publish_position_error(self) -> None:
        """Publish the map-frame XY error to the latest valid goal request."""
        if self._goal_pose is None or self._robot_pose is None:
            return

        target = self._goal_pose.pose.position
        current = self._robot_pose.pose.position
        message = Float32()
        message.data = math.hypot(target.x - current.x, target.y - current.y)
        self._position_error.publish(message)

    def _on_goal(self, pose: PoseStamped) -> None:
        if pose.header.frame_id != 'map':
            self._publish_status('rejected: /goal_pose frame_id must be map')
            return
        if self._goal_handle is not None or self._goal_pending:
            self._publish_status('rejected: a navigation goal is already active')
            return

        # Reserve the goal before the asynchronous action response arrives.
        # Without this guard, two UI clicks in the same executor cycle can both
        # be submitted to Nav2, producing the rejection seen in integration.
        self._goal_pending = True
        self._goal_pose = pose
        self._publish_arrival(False)
        self._publish_position_error()
        if not self._client.server_is_ready():
            self._goal_pending = False
            self._publish_status('rejected: NavigateToPose action server is not ready')
            return

        request = NavigateToPose.Goal()
        request.pose = pose
        self._publish_status('planning')
        try:
            future = self._client.send_goal_async(
                request, feedback_callback=self._on_feedback)
        except Exception as error:
            self._goal_pending = False
            self._publish_status(f'failed: could not submit navigation goal: {error}')
            self._publish_arrival(False)
            return
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        self._goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as error:  # Action transport failures are terminal.
            self._publish_status(f'failed: could not submit navigation goal: {error}')
            self._publish_arrival(False)
            return
        if not goal_handle.accepted:
            self._publish_status('rejected: Nav2 did not accept the goal')
            self._publish_arrival(False)
            return
        self._goal_handle = goal_handle
        self._publish_status('navigating')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, _feedback_message) -> None:
        # /plan and /local_plan remain the visual source of truth for D.
        return

    def _on_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception as error:
            self._publish_status(f'failed: navigation result unavailable: {error}')
            self._publish_arrival(False)
            self._goal_handle = None
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status('arrived')
            self._publish_arrival(True)
        elif status == GoalStatus.STATUS_CANCELED:
            self._publish_status('cancelled')
            self._publish_arrival(False)
        else:
            self._publish_status(f'failed: NavigateToPose status={status}')
            self._publish_arrival(False)
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
