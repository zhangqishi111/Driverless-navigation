#!/usr/bin/env python3
"""Execute a short navigation task without preempting or skipping waypoints."""

import math
from copy import deepcopy
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from landerpi_msgs.msg import NavigationPointState, NavigationTaskState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String, UInt8


@dataclass
class PointRecord:
    """Persistent execution result for one task waypoint."""

    index: int
    target: object
    state: int = NavigationPointState.PENDING
    detail: str = 'waiting'
    started_ns: Optional[int] = None
    elapsed_time_s: float = 0.0
    has_arrival_error: bool = False
    arrival_error_m: float = 0.0
    retry_count: int = 0

    def start(self, now_ns: int) -> None:
        if self.started_ns is None:
            self.started_ns = now_ns

    def set_state(self, state: int, detail: str) -> None:
        self.state = state
        self.detail = detail

    def finish(
            self,
            state: int,
            detail: str,
            now_ns: int,
            arrival_error_m: Optional[float] = None) -> None:
        self.set_state(state, detail)
        if self.started_ns is not None:
            self.elapsed_time_s = max(0.0, (now_ns - self.started_ns) / 1e9)
        if arrival_error_m is not None:
            self.arrival_error_m = float(arrival_error_m)
            self.has_arrival_error = True

    def elapsed_at(self, now_ns: int) -> float:
        if (self.started_ns is not None and self.state in (
                NavigationPointState.PLANNING,
                NavigationPointState.NAVIGATING,
                NavigationPointState.WAITING_FOR_STOP)):
            return max(0.0, (now_ns - self.started_ns) / 1e9)
        return self.elapsed_time_s

    def to_message(self, now_ns: int) -> NavigationPointState:
        message = NavigationPointState()
        message.index = self.index
        message.target = self.target
        message.state = self.state
        message.has_arrival_error = self.has_arrival_error
        message.arrival_error_m = self.arrival_error_m if self.has_arrival_error else 0.0
        message.elapsed_time_s = self.elapsed_at(now_ns)
        message.retry_count = self.retry_count
        message.detail = self.detail
        return message


def finalize_records(
        records: List[PointRecord],
        failed_index: int,
        failed_state: int,
        reason: str,
        now_ns: int) -> None:
    """Finish one bad point and make every later target explicitly unexecuted."""
    records[failed_index].finish(failed_state, reason, now_ns)
    failed_number = records[failed_index].index
    for record in records[failed_index + 1:]:
        record.set_state(
            NavigationPointState.NOT_EXECUTED,
            f'not executed because point {failed_number} failed',
        )


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
        self._task_state = self.create_publisher(
            NavigationTaskState, '/navigation_task/state', latched)
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
        self._records: List[PointRecord] = []
        self._task_state_code = NavigationTaskState.IDLE
        self._task_detail = 'ready'
        self._task_started_ns: Optional[int] = None
        self._task_elapsed_time_s = 0.0
        self._last_state_publish_ns: Optional[int] = None
        self._latest_amcl_xy = None
        self._publish_active(False)
        self._publish_index(0)
        self._publish_status('idle', 'ready')
        self._publish_task_snapshot()

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

    def _snapshot_message(self, now_ns: int) -> NavigationTaskState:
        message = NavigationTaskState()
        message.header.frame_id = 'map'
        message.header.stamp.sec = now_ns // 1_000_000_000
        message.header.stamp.nanosec = now_ns % 1_000_000_000
        message.task_id = self._task_id
        message.active = self._claimed
        if self._claimed and self._index >= 0:
            message.current_index = self._index + 1
        elif self._terminal_point:
            message.current_index = self._terminal_point
        else:
            message.current_index = 0
        message.total_points = len(self._records)
        message.state = self._task_state_code
        if self._task_started_ns is None:
            message.elapsed_time_s = 0.0
        elif self._claimed:
            message.elapsed_time_s = max(
                0.0, (now_ns - self._task_started_ns) / 1e9)
        else:
            message.elapsed_time_s = self._task_elapsed_time_s
        message.detail = self._task_detail
        message.points = [record.to_message(now_ns) for record in self._records]
        return message

    def _publish_task_snapshot(self, now_ns: Optional[int] = None) -> None:
        if now_ns is None:
            now_ns = self._now()
        self._task_state.publish(self._snapshot_message(now_ns))
        self._last_state_publish_ns = now_ns

    def _set_point_state(self, index: int, state: int, detail: str) -> None:
        record = self._records[index]
        if state == NavigationPointState.PLANNING:
            record.start(self._now())
        record.set_state(state, detail)
        self._task_detail = detail
        self._publish_task_snapshot()

    def _finish_point(
            self,
            index: int,
            state: int,
            detail: str,
            now_ns: int,
            arrival_error_m: Optional[float] = None) -> None:
        self._records[index].finish(
            state, detail, now_ns, arrival_error_m=arrival_error_m)
        self._task_detail = detail
        self._publish_task_snapshot(now_ns)

    def _on_goal_bridge_active(self, message: Bool) -> None:
        self._bridge_active = message.data

    def _reject_submission(self, detail: str) -> None:
        self._publish_status('rejected', detail)
        if self._claimed:
            return
        self._records = []
        self._terminal_point = 0
        self._task_state_code = NavigationTaskState.REJECTED
        self._task_detail = detail
        self._task_started_ns = None
        self._task_elapsed_time_s = 0.0
        self._publish_task_snapshot()

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        error = self._localization_error_for(message)
        if error is None:
            self._last_localization_ns = self._now()
            self._localization_error = ''
            self._latest_amcl_xy = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
            )
        else:
            self._last_localization_ns = None
            self._localization_error = error
            self._latest_amcl_xy = None

    def _localization_error_for(
            self, message: PoseWithCovarianceStamped) -> Optional[str]:
        if message.header.frame_id != 'map':
            return 'AMCL pose is not in map frame'
        pose = message.pose.pose
        pose_values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in pose_values):
            return 'AMCL pose contains non-finite values'
        if sum(value * value for value in pose_values[3:]) <= 1e-8:
            return 'AMCL pose has an invalid orientation'
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
            self._reject_submission('another task is still cleaning up')
            return
        if self._bridge_active:
            self._reject_submission('a single navigation goal is active')
            return
        if message.header.frame_id != 'map':
            self._reject_submission('task goals must use map frame')
            return
        count = len(message.poses)
        if count == 0 or count > self._max_goals:
            self._reject_submission(
                f'goal count must be between 1 and {self._max_goals}')
            return
        for number, pose in enumerate(message.poses, start=1):
            if not self._valid_pose(pose):
                self._reject_submission(f'goal {number} has invalid values')
                return
        now = self._now()
        if not self._localization_is_healthy(now):
            self._reject_submission(self._localization_error)
            return

        self._task_id += 1
        self._goal_count = count
        self._goals = []
        self._records = []
        for number, pose in enumerate(message.poses, start=1):
            goal = PoseStamped()
            goal.header = message.header
            goal.pose = deepcopy(pose)
            self._goals.append(goal)
            self._records.append(PointRecord(
                index=number,
                target=deepcopy(pose),
            ))
        self._index = 0
        self._state = 'dispatching'
        self._terminal_state = None
        self._terminal_reason = None
        self._terminal_point = 0
        self._task_state_code = NavigationTaskState.ACTIVE
        self._task_detail = f'{count} goals queued'
        self._task_started_ns = now
        self._task_elapsed_time_s = 0.0
        self._publish_active(True)
        self._publish_index(1)
        self._publish_status('accepted', f'{count} goals queued')
        self._publish_task_snapshot(now)
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
        self._set_point_state(
            self._index,
            NavigationPointState.PLANNING,
            'goal submitted to Nav2',
        )
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
                try:
                    result = goal_handle.get_result_async()
                    result.add_done_callback(
                        lambda response, run_id=task_id:
                        self._on_goal_result(run_id, response))
                except Exception as error:
                    self.get_logger().error(
                        f'could not observe cancelled Nav2 goal: {error}')
                self._cancel_active_goal()
            else:
                self._finish_terminal_cleanup()
            return
        if not goal_handle.accepted:
            self._goal_handle = None
            self._request_terminal('failed', 'Nav2 rejected the goal')
            return
        self._state = 'navigating'
        self._set_point_state(
            self._index,
            NavigationPointState.NAVIGATING,
            'Nav2 accepted goal',
        )
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
        self._set_point_state(
            self._index,
            NavigationPointState.WAITING_FOR_STOP,
            'Nav2 arrived; waiting for confirmed zero velocity',
        )
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
        state_snapshot_due = self._last_state_publish_ns is None
        if not state_snapshot_due:
            state_snapshot_due = (
                now - self._last_state_publish_ns >= 200_000_000)
        if state_snapshot_due:
            self._publish_task_snapshot(now)
        if not self._localization_is_healthy(now):
            self._request_terminal('failed', self._localization_error)
            return
        if self._state != 'waiting_for_stop':
            return
        if now - self._stop_started_ns > self._stop_timeout_ns:
            self._request_terminal('failed', 'zero-velocity confirmation timed out')
            return
        missing_velocity = (
            self._zero_since_ns is None or self._last_velocity_ns is None)
        stale_velocity = False
        if self._last_velocity_ns is not None:
            stale_velocity = (
                now - self._last_velocity_ns > self._stop_velocity_freshness_ns)
        if missing_velocity or stale_velocity:
            return
        if now - self._zero_since_ns >= self._stop_settle_ns:
            self._advance_or_finish()

    def _advance_or_finish(self) -> None:
        now = self._now()
        if self._latest_amcl_xy is None:
            self._request_terminal('failed', 'no valid AMCL pose for arrival error')
            return
        target = self._records[self._index].target.position
        arrival_error = math.hypot(
            target.x - self._latest_amcl_xy[0],
            target.y - self._latest_amcl_xy[1],
        )
        self._finish_point(
            self._index,
            NavigationPointState.SUCCEEDED,
            'goal reached and stop confirmed',
            now,
            arrival_error_m=arrival_error,
        )
        if self._index + 1 == len(self._goals):
            self._request_terminal('succeeded', 'all goals reached and stopped')
            return
        self._index += 1
        self._publish_index(self._index + 1)
        self._state = 'dispatching'
        self._task_detail = 'previous goal reached and stop confirmed'
        self._publish_status('advancing', 'previous goal reached and stop confirmed')
        self._publish_task_snapshot(now)
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
        now = self._now()
        state_codes = {
            'succeeded': NavigationTaskState.SUCCEEDED,
            'failed': NavigationTaskState.FAILED,
            'cancelled': NavigationTaskState.CANCELLED,
        }
        self._task_state_code = state_codes.get(
            terminal_state, NavigationTaskState.FAILED)
        self._task_detail = reason
        if self._task_started_ns is not None:
            self._task_elapsed_time_s = max(
                0.0, (now - self._task_started_ns) / 1e9)
        if terminal_state != 'succeeded' and self._records and self._index >= 0:
            point_state = (
                NavigationPointState.CANCELLED
                if terminal_state == 'cancelled'
                else NavigationPointState.FAILED
            )
            finalize_records(
                self._records,
                failed_index=self._index,
                failed_state=point_state,
                reason=reason,
                now_ns=now,
            )
        self._goals.clear()  # Never silently move on after a bad waypoint.
        self._publish_status(terminal_state, reason)
        self._publish_task_snapshot(now)
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
        self._task_detail = terminal_reason
        self._publish_task_snapshot()


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
