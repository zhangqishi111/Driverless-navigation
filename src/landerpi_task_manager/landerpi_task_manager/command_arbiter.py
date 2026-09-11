"""Sole owner of Nav2's public NavigateToPose action endpoint."""

import threading

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from rclpy.signals import SignalHandlerOptions


class CommandGate:
    """Thread-safe fail-closed ownership state for the action proxy."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ready = False
        self._reserved = False
        self._uncertain = False
        self._shutting_down = False

    @property
    def has_unresolved_command(self) -> bool:
        with self._lock:
            return self._reserved

    def mark_startup_cleanup_complete(self) -> None:
        with self._lock:
            if not self._shutting_down:
                self._ready = True

    def try_reserve(self) -> bool:
        with self._lock:
            if not self._ready or self._reserved or self._shutting_down:
                return False
            self._reserved = True
            self._uncertain = False
            return True

    def mark_outcome_unknown(self) -> None:
        with self._lock:
            self._reserved = True
            self._uncertain = True

    def mark_outcome_known(self) -> None:
        with self._lock:
            self._reserved = False
            self._uncertain = False

    def begin_shutdown(self) -> bool:
        with self._lock:
            self._ready = False
            self._shutting_down = True
            return self._reserved


class NavigationCommandArbiter(Node):
    """Serialize internal navigation clients onto one Nav2 action client."""

    def __init__(self) -> None:
        super().__init__('navigation_command_arbiter')
        callback_group = ReentrantCallbackGroup()
        self._gate = CommandGate()
        self._state_lock = threading.Lock()
        self._startup_lock = threading.Lock()
        self._shutting_down = False
        self._downstream_handle = None
        self._downstream_result_future = None
        self._cancel_future = None
        self._startup_cleanup_future = None
        self._startup_cancel_accepted = False
        self._startup_pending_goal_ids = set()
        self._latest_nav2_statuses = {}
        self._startup_finished = False
        self._action_server = None

        self._client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=callback_group)
        self._cancel_all_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
            callback_group=callback_group,
        )
        self._status_subscription = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._on_nav2_status,
            qos_profile_action_status_default,
            callback_group=callback_group,
        )
        self._startup_timer = self.create_timer(
            0.10, self._ensure_clean_start, callback_group=callback_group)
        self._callback_group = callback_group

    def _ensure_clean_start(self) -> None:
        """Cancel orphaned Nav2 goals before exposing the internal action."""
        if self._startup_cleanup_future is not None:
            return
        if not self._cancel_all_client.service_is_ready():
            return
        request = CancelGoal.Request()  # Zero UUID and stamp means cancel all.
        self._startup_cleanup_future = self._cancel_all_client.call_async(
            request)
        self._startup_cleanup_future.add_done_callback(
            self._on_startup_cleanup_response)

    def _on_startup_cleanup_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                'Nav2 startup cancellation failed; still rejecting goals: '
                f'{error}')
            self._startup_cleanup_future = None
            return
        if response.return_code != CancelGoal.Response.ERROR_NONE:
            self.get_logger().error(
                'Nav2 rejected startup cancel-all request; still rejecting '
                'goals '
                f'(code={response.return_code})')
            self._startup_cleanup_future = None
            return

        pending_ids = {
            bytes(goal.goal_id.uuid) for goal in response.goals_canceling
        }
        with self._startup_lock:
            self._startup_cancel_accepted = True
            self._startup_pending_goal_ids = pending_ids
        if pending_ids:
            self.get_logger().info(
                f'waiting for {len(pending_ids)} canceled Nav2 goal(s) to '
                'become terminal')
        self._check_startup_cleanup_complete()

    def _on_nav2_status(self, message: GoalStatusArray) -> None:
        with self._startup_lock:
            self._latest_nav2_statuses = {
                bytes(status.goal_info.goal_id.uuid): status.status
                for status in message.status_list
            }
        self._check_startup_cleanup_complete()

    def _check_startup_cleanup_complete(self) -> None:
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        }
        with self._startup_lock:
            if not self._startup_cancel_accepted or self._startup_finished:
                return
            self._startup_pending_goal_ids = {
                goal_id for goal_id in self._startup_pending_goal_ids
                if self._latest_nav2_statuses.get(goal_id)
                not in terminal_statuses
            }
            if self._startup_pending_goal_ids:
                return
            self._startup_finished = True
        self._finish_startup_cleanup()

    def _finish_startup_cleanup(self) -> None:
        if self._shutting_down:
            return
        self._gate.mark_startup_cleanup_complete()
        self._action_server = ActionServer(
            self, NavigateToPose, '/navigation_command',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._startup_timer.cancel()
        self.get_logger().info(
            'navigation command arbiter ready; startup Nav2 cleanup complete')

    def _on_goal(self, _request) -> GoalResponse:
        if self._gate.try_reserve():
            return GoalResponse.ACCEPT
        self.get_logger().warning(
            'rejected navigation command because another outcome is '
            'unresolved')
        return GoalResponse.REJECT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self._cancel_downstream()
        return CancelResponse.ACCEPT

    def _cancel_downstream(self) -> None:
        with self._state_lock:
            handle = self._downstream_handle
            if handle is None or self._cancel_future is not None:
                return
            try:
                self._cancel_future = handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().error(
                    f'could not request Nav2 cancellation: {error}')

    async def _execute(self, goal_handle):
        known_outcome = False
        downstream_handle = None
        try:
            if not self._client.server_is_ready():
                goal_handle.abort()
                known_outcome = True
                return NavigateToPose.Result()

            response_future = self._client.send_goal_async(
                goal_handle.request,
                feedback_callback=lambda feedback:
                self._forward_feedback(goal_handle, feedback),
            )
            downstream_handle = await response_future
            if not downstream_handle.accepted:
                goal_handle.abort()
                known_outcome = True
                return NavigateToPose.Result()

            with self._state_lock:
                self._downstream_handle = downstream_handle
                self._downstream_result_future = (
                    downstream_handle.get_result_async())
                result_future = self._downstream_result_future

            if goal_handle.is_cancel_requested or self._shutting_down:
                self._cancel_downstream()

            response = await result_future
            status = response.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
            elif status == GoalStatus.STATUS_CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            known_outcome = True
            return response.result
        except Exception as error:
            if goal_handle.is_active:
                goal_handle.abort()
            self._gate.mark_outcome_unknown()
            self.get_logger().error(
                'navigation outcome is unknown; arbiter remains locked: '
                f'{error}')
            return NavigateToPose.Result()
        finally:
            if known_outcome:
                with self._state_lock:
                    if self._downstream_handle is downstream_handle:
                        self._downstream_handle = None
                        self._downstream_result_future = None
                        self._cancel_future = None
                self._gate.mark_outcome_known()

    @staticmethod
    def _forward_feedback(goal_handle, feedback_message) -> None:
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback_message.feedback)

    def begin_shutdown(self) -> bool:
        self._shutting_down = True
        unresolved = self._gate.begin_shutdown()
        if unresolved:
            self._cancel_downstream()
        return unresolved

    @property
    def has_unresolved_command(self) -> bool:
        return self._gate.has_unresolved_command

    def destroy_node(self):
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        return super().destroy_node()


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationCommandArbiter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.begin_shutdown()
        try:
            while rclpy.ok() and node.has_unresolved_command:
                executor.spin_once(timeout_sec=0.10)
        except KeyboardInterrupt:
            node.get_logger().critical(
                'forced shutdown before Nav2 reported a terminal result')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
