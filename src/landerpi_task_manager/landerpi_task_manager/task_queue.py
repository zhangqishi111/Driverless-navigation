#!/usr/bin/env python3
"""Execute a short navigation task without preempting or skipping waypoints."""

import math
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String, UInt8


class NavigationTaskQueue(Node):
    """Own one-to-three ``NavigateToPose`` requests until each stop is proven."""

    def __init__(self) -> None:
        super().__init__('navigation_task_queue')
        self.declare_parameter('max_goals', 3)
        self.declare_parameter('localization_timeout', 1.0)
        self.declare_parameter('max_position_variance', 0.25)
        self.declare_parameter('max_yaw_variance', 0.50)
        # Simulation observes Nav2's output directly. The real-navigation
        # launch overrides this with the adapter's final controller output.
        self.declare_parameter('stop_velocity_topic', '/cmd_vel_nav_output')
        self.declare_parameter('stop_speed_threshold', 0.01)
        self.declare_parameter('stop_settle_duration', 0.50)
        self.declare_parameter('stop_timeout', 3.0)
        self.declare_parameter('stop_velocity_freshness', 0.25)

        self._max_goals = int(self.get_parameter('max_goals').value)
        self._localization_timeout_ns = self._ns('localization_timeout')
        self._max_position_variance = float(
            self.get_parameter('max_position_variance').value)
        self._max_yaw_variance = float(self.get_parameter('max_yaw_variance').value)
        self._stop_speed_threshold = float(
            self.get_parameter('stop_speed_threshold').value)
        self._stop_settle_ns = self._ns('stop_settle_duration')
        self._stop_timeout_ns = self._ns('stop_timeout')
        self._stop_velocity_freshness_ns = self._ns('stop_velocity_freshness')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status = self.create_publisher(String, '/navigation_task/status', latched)
        self._active = self.create_publisher(Bool, '/navigation_task/active', latched)
        self._current = self.create_publisher(
            UInt8, '/navigation_task/current_index', latched)
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(PoseArray, '/navigation_task/goals', self._on_goals, 10)
        self.create_subscription(Empty, '/navigation_task/cancel', self._on_cancel, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        self.create_subscription(
            Twist, self.get_parameter('stop_velocity_topic').value, self._on_velocity, 10)
        self.create_subscription(
            Bool, '/navigation_goal_bridge/active', self._on_goal_bridge_active, latched)
        self.create_timer(0.05, self._monitor)

        self._goals: List[PoseStamped] = []
        self._goal_count = 0
        self._task_id = 0
        self._index = -1
        self._state = 'idle'
        self._claimed = False
        self._bridge_active = False
        self._goal_handle = None
        self._goal_pending = False
        self._last_localization_ns: Optional[int] = None
        self._localization_error = 'no AMCL pose received'
        self._stop_started_ns: Optional[int] = None
        self._last_velocity_ns: Optional[int] = None
        self._zero_since_ns: Optional[int] = None
        self._terminal_state: Optional[str] = None
        self._terminal_reason: Optional[str] = None
        self._terminal_point = 0
        self._publish_active(False)
        self._publish_index(0)
        self._publish_status('idle', 'ready')

    def _ns(self, parameter: str) -> int:
        return int(float(self.get_parameter(parameter).value) * 1_000_000_000)

    def _now(self) -> int:
        return self.get_clock().now().nanoseconds

    def _publish_status(self, state: str, detail: str) -> None:
        point = self._index + 1 if self._index >= 0 else 0
        if state == self._terminal_state and self._terminal_point:
            point = self._terminal_point
        message = String()
        message.data = (
            f'task={self._task_id} state={state} point={point}/{self._goal_count} '
            f'detail={detail}')
        self._status.publish(message)
        self.get_logger().info(message.data)

    def _publish_active(self, active: bool) -> None:
        self._claimed = active
        message = Bool()
        message.data = active
        self._active.publish(message)

    def _publish_index(self, index: int) -> None:
        message = UInt8()
        message.data = index
        self._current.publish(message)

    def _on_goal_bridge_active(self, message: Bool) -> None:
        self._bridge_active = message.data

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        error = self._localization_error_for(message)
        if error is None:
            self._last_localization_ns = self._now()
            self._localization_error = ''
        else:
            self._last_localization_ns = None
            self._localization_error = error

    def _localization_error_for(
            self, message: PoseWithCovarianceStamped) -> Optional[str]:
        if message.header.frame_id != 'map':
            return 'AMCL pose is not in map frame'
        covariance = message.pose.covariance
        position_variance = max(covariance[0], covariance[7])
        yaw_variance = covariance[35]
        if not all(math.isfinite(value) for value in
                   (position_variance, yaw_variance)):
            return 'AMCL covariance is non-finite'
        if position_variance > self._max_position_variance:
            return (f'AMCL position variance {position_variance:.3f} exceeds '
                    f'{self._max_position_variance:.3f}')
        if yaw_variance > self._max_yaw_variance:
            return (f'AMCL yaw variance {yaw_variance:.3f} exceeds '
                    f'{self._max_yaw_variance:.3f}')
        return None

    def _localization_is_healthy(self, now: int) -> bool:
        if self._last_localization_ns is None:
            return False
        if now - self._last_localization_ns > self._localization_timeout_ns:
            self._localization_error = 'AMCL pose timed out'
            return False
        return True

    def _on_goals(self, message: PoseArray) -> None:
        if self._claimed:
            self._publish_status('rejected', 'another task is still cleaning up')
            return
        if self._bridge_active:
            self._publish_status('rejected', 'a single navigation goal is active')
            return
        if message.header.frame_id != 'map':
            self._publish_status('rejected', 'task goals must use map frame')
            return
        count = len(message.poses)
        if count == 0 or count > self._max_goals:
            self._publish_status(
                'rejected', f'goal count must be between 1 and {self._max_goals}')
            return
        for number, pose in enumerate(message.poses, start=1):
            if not self._valid_pose(pose):
                self._publish_status('rejected', f'goal {number} has invalid values')
                return
        if not self._localization_is_healthy(self._now()):
            self._publish_status('rejected', self._localization_error)
            return

        self._task_id += 1
        self._goal_count = count
        self._goals = []
        for pose in message.poses:
            goal = PoseStamped()
            goal.header = message.header
            goal.pose = pose
            self._goals.append(goal)
        self._index = 0
        self._state = 'dispatching'
        self._terminal_state = None
        self._terminal_reason = None
        self._terminal_point = 0
        self._publish_active(True)
        self._publish_index(1)
        self._publish_status('accepted', f'{count} goals queued')
        self._send_current_goal()

    @staticmethod
    def _valid_pose(pose) -> bool:
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w,
        )
        orientation_norm = sum(value * value for value in values[3:])
        return all(math.isfinite(value) for value in values) and orientation_norm > 1e-8

    def _send_current_goal(self) -> None:
        if not self._claimed or self._state != 'dispatching':
            return
        if not self._localization_is_healthy(self._now()):
            self._request_terminal('failed', self._localization_error)
            return
        if not self._client.server_is_ready():
            self._request_terminal('failed', 'NavigateToPose action server is not ready')
            return
        request = NavigateToPose.Goal()
        request.pose = self._goals[self._index]
        task_id = self._task_id
        self._goal_pending = True
        self._publish_status('planning', 'goal submitted to Nav2')
        try:
            future = self._client.send_goal_async(request)
        except Exception as error:
            self._goal_pending = False
            self._request_terminal('failed', f'could not submit goal: {error}')
            return
        future.add_done_callback(
            lambda response, run_id=task_id: self._on_goal_response(run_id, response))

    def _on_goal_response(self, task_id: int, future) -> None:
        if task_id != self._task_id:
            return
        self._goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as error:
            if self._terminal_state is not None:
                self._finish_terminal_cleanup()
            else:
                self._request_terminal('failed', f'goal response unavailable: {error}')
            return
        self._goal_handle = goal_handle
        if self._terminal_state is not None:
            if goal_handle.accepted:
                self._cancel_active_goal()
            else:
                self._finish_terminal_cleanup()
            return
        if not goal_handle.accepted:
            self._goal_handle = None
            self._request_terminal('failed', 'Nav2 rejected the goal')
            return
        self._state = 'navigating'
        self._publish_status('navigating', 'Nav2 accepted goal')
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda response, run_id=task_id: self._on_goal_result(run_id, response))

    def _on_goal_result(self, task_id: int, future) -> None:
        if task_id != self._task_id:
            return
        if self._terminal_state is not None:
            self._goal_handle = None
            self._finish_terminal_cleanup()
            return
        try:
            status = future.result().status
        except Exception as error:
            # Retain the handle and the navigation claim. The action outcome is
            # unknown, so another request must not be allowed to preempt it.
            self._request_terminal('failed', f'navigation result unavailable: {error}')
            return
        # A result status means this action is terminal; do not issue a second
        # cancellation and accidentally wait forever for another result.
        self._goal_handle = None
        if status != GoalStatus.STATUS_SUCCEEDED:
            detail = ('Nav2 cancelled the goal' if status == GoalStatus.STATUS_CANCELED
                      else f'Nav2 failed goal with status={status}')
            self._request_terminal('failed', detail)
            return
        self._state = 'waiting_for_stop'
        self._stop_started_ns = self._now()
        self._last_velocity_ns = None
        self._zero_since_ns = None
        self._publish_status(
            'waiting_for_stop', 'Nav2 arrived; waiting for confirmed zero velocity')

    def _on_velocity(self, message: Twist) -> None:
        if not self._claimed or self._state != 'waiting_for_stop':
            return
        now = self._now()
        self._last_velocity_ns = now
        speed = max(
            abs(message.linear.x), abs(message.linear.y), abs(message.linear.z),
            abs(message.angular.x), abs(message.angular.y), abs(message.angular.z))
        if speed <= self._stop_speed_threshold:
            if self._zero_since_ns is None:
                self._zero_since_ns = now
        else:
            self._zero_since_ns = None

    def _monitor(self) -> None:
        if not self._claimed or self._terminal_state is not None:
            return
        now = self._now()
        if not self._localization_is_healthy(now):
            self._request_terminal('failed', self._localization_error)
            return
        if self._state != 'waiting_for_stop':
            return
        if now - self._stop_started_ns > self._stop_timeout_ns:
            self._request_terminal('failed', 'zero-velocity confirmation timed out')
            return
        if (self._zero_since_ns is None or self._last_velocity_ns is None or
                now - self._last_velocity_ns > self._stop_velocity_freshness_ns):
            return
        if now - self._zero_since_ns >= self._stop_settle_ns:
            self._advance_or_finish()

    def _advance_or_finish(self) -> None:
        if self._index + 1 == len(self._goals):
            self._request_terminal('succeeded', 'all goals reached and stopped')
            return
        self._index += 1
        self._publish_index(self._index + 1)
        self._state = 'dispatching'
        self._publish_status('advancing', 'previous goal reached and stop confirmed')
        self._send_current_goal()

    def _on_cancel(self, _message: Empty) -> None:
        if not self._claimed:
            self._publish_status('ignored', 'no active task to cancel')
            return
        self._request_terminal('cancelled', 'cancel requested')

    def _request_terminal(self, terminal_state: str, reason: str) -> None:
        if self._terminal_state is not None:
            return
        self._terminal_state = terminal_state
        self._terminal_reason = reason
        self._terminal_point = self._index + 1 if self._index >= 0 else 0
        self._goals.clear()  # Never silently move on after a bad waypoint.
        self._publish_status(terminal_state, reason)
        if self._goal_handle is not None:
            self._cancel_active_goal()
        elif not self._goal_pending:
            self._finish_terminal_cleanup()

    def _cancel_active_goal(self) -> None:
        try:
            self._goal_handle.cancel_goal_async()
        except Exception as error:
            # Keep our claim: releasing it could allow a second command to
            # preempt a vehicle whose cancellation state is uncertain.
            self.get_logger().error(f'could not cancel active Nav2 goal: {error}')

    def _finish_terminal_cleanup(self) -> None:
        terminal_state = self._terminal_state or 'failed'
        terminal_reason = self._terminal_reason or 'unknown terminal state'
        self._goal_handle = None
        self._goal_pending = False
        self._index = -1
        self._state = terminal_state
        self._publish_index(0)
        self._publish_active(False)
        self._publish_status(terminal_state, f'{terminal_reason}; queue cleared')


def main() -> None:
    rclpy.init()
    node = NavigationTaskQueue()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
