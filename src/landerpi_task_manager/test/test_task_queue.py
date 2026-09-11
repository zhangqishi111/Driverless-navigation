import math
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Twist
from action_msgs.msg import GoalStatus
from landerpi_msgs.msg import NavigationPointState, NavigationTaskState
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from landerpi_task_manager.task_queue import (
    NavigationTaskQueue,
    PointRecord,
    finalize_records,
)


class PublisherCapture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class CompletedFuture:
    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value


class DeferredFuture:
    def __init__(self):
        self.callback = None
        self._value = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return self._value

    def resolve(self, value):
        self._value = value
        assert self.callback is not None
        self.callback(self)


class AcceptedGoalHandle:
    accepted = True

    def __init__(self):
        self.result_future = DeferredFuture()
        self.cancel_requested = False

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_requested = True


class FakeActionClient:
    def __init__(self):
        self.requests = []
        self.response_futures = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, request):
        self.requests.append(request)
        future = DeferredFuture()
        self.response_futures.append(future)
        return future


class UnavailableActionClient:
    def server_is_ready(self):
        return False


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    started_here = not rclpy.ok()
    if started_here:
        rclpy.init()
    yield
    if started_here and rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def queue(ros_context):
    if not rclpy.ok():
        rclpy.init()
    node = NavigationTaskQueue()
    node._task_state_qos = node._task_state.qos_profile
    node._task_state = PublisherCapture()
    node._send_current_goal = lambda: None
    node._now = lambda: 10_000_000_000
    node._last_localization_ns = 10_000_000_000
    node._localization_error = ''
    yield node
    node.destroy_node()


def make_pose(x=1.0, y=2.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    return pose


def make_goal_array(count):
    goals = PoseArray()
    goals.header.frame_id = 'map'
    goals.poses = [make_pose(float(number), float(number + 1))
                   for number in range(1, count + 1)]
    return goals


def test_success_records_arrival_error_elapsed_time_and_zero_retries():
    record = PointRecord(index=1, target=make_pose())

    record.start(1_000_000_000)
    record.finish(
        state=NavigationPointState.SUCCEEDED,
        detail='stopped',
        now_ns=3_500_000_000,
        arrival_error_m=0.08,
    )

    assert record.elapsed_time_s == 2.5
    assert math.isclose(record.arrival_error_m, 0.08)
    assert record.has_arrival_error is True
    assert record.retry_count == 0
    assert record.state == NavigationPointState.SUCCEEDED
    assert record.detail == 'stopped'


def test_failure_preserves_completed_points_and_marks_later_points_not_executed():
    records = [
        PointRecord(index=number, target=make_pose(x=float(number)))
        for number in range(1, 4)
    ]
    records[0].start(0)
    records[0].finish(
        NavigationPointState.SUCCEEDED,
        'stopped',
        now_ns=1_000_000_000,
        arrival_error_m=0.03,
    )
    records[1].start(1_000_000_000)

    finalize_records(
        records,
        failed_index=1,
        failed_state=NavigationPointState.FAILED,
        reason='AMCL pose timed out',
        now_ns=2_000_000_000,
    )

    assert [record.state for record in records] == [
        NavigationPointState.SUCCEEDED,
        NavigationPointState.FAILED,
        NavigationPointState.NOT_EXECUTED,
    ]
    assert records[0].detail == 'stopped'
    assert records[1].elapsed_time_s == 1.0
    assert records[1].detail == 'AMCL pose timed out'
    assert records[2].detail == 'not executed because point 2 failed'


def test_point_message_reports_live_elapsed_time_without_fabricating_error():
    record = PointRecord(index=2, target=make_pose())
    record.start(4_000_000_000)
    record.set_state(NavigationPointState.NAVIGATING, 'Nav2 accepted goal')

    message = record.to_message(now_ns=5_250_000_000)

    assert message.index == 2
    assert message.state == NavigationPointState.NAVIGATING
    assert message.elapsed_time_s == 1.25
    assert message.has_arrival_error is False
    assert message.arrival_error_m == 0.0
    assert message.retry_count == 0
    assert message.target.position.x == 1.0


def test_accepting_goals_creates_an_ordered_authoritative_snapshot(queue):
    queue._on_goals(make_goal_array(3))

    snapshot = queue._task_state.messages[-1]
    assert snapshot.task_id == 1
    assert snapshot.active is True
    assert snapshot.current_index == 1
    assert snapshot.total_points == 3
    assert snapshot.state == NavigationTaskState.ACTIVE
    assert [point.index for point in snapshot.points] == [1, 2, 3]
    assert [point.target.position.x for point in snapshot.points] == [1.0, 2.0, 3.0]
    assert [point.state for point in snapshot.points] == [
        NavigationPointState.PENDING,
        NavigationPointState.PENDING,
        NavigationPointState.PENDING,
    ]


def test_terminal_failure_keeps_results_and_blocks_every_later_point(queue):
    queue._on_goals(make_goal_array(3))
    queue._records[0].start(8_000_000_000)
    queue._records[0].finish(
        NavigationPointState.SUCCEEDED,
        'stopped',
        9_000_000_000,
        arrival_error_m=0.02,
    )
    queue._index = 1
    queue._records[1].start(9_000_000_000)

    queue._request_terminal('failed', 'AMCL pose timed out')

    snapshot = queue._task_state.messages[-1]
    assert snapshot.active is False
    assert snapshot.state == NavigationTaskState.FAILED
    assert snapshot.current_index == 2
    assert [point.state for point in snapshot.points] == [
        NavigationPointState.SUCCEEDED,
        NavigationPointState.FAILED,
        NavigationPointState.NOT_EXECUTED,
    ]
    assert snapshot.points[2].detail == 'not executed because point 2 failed'


def test_stop_confirmation_records_error_before_advancing(queue):
    queue._on_goals(make_goal_array(2))
    queue._records[0].start(8_000_000_000)
    queue._records[0].set_state(
        NavigationPointState.WAITING_FOR_STOP,
        'waiting for confirmed zero velocity',
    )
    queue._latest_amcl_xy = (1.03, 2.04)

    queue._advance_or_finish()

    completed = queue._records[0]
    assert completed.state == NavigationPointState.SUCCEEDED
    assert math.isclose(completed.arrival_error_m, 0.05)
    assert completed.elapsed_time_s == 2.0
    assert queue._index == 1


def test_invalid_submission_publishes_a_structured_rejection(queue):
    goals = make_goal_array(2)
    goals.header.frame_id = 'odom'

    queue._on_goals(goals)

    snapshot = queue._task_state.messages[-1]
    assert snapshot.active is False
    assert snapshot.state == NavigationTaskState.REJECTED
    assert snapshot.total_points == 0
    assert snapshot.current_index == 0
    assert snapshot.detail == 'task goals must use map frame'


def test_each_idle_rejected_submission_gets_a_new_task_id(queue):
    goals = make_goal_array(1)
    goals.header.frame_id = 'odom'

    queue._on_goals(goals)
    first_id = queue._task_state.messages[-1].task_id
    queue._on_goals(goals)
    second_id = queue._task_state.messages[-1].task_id

    assert first_id == 1
    assert second_id == 2


def test_cancel_while_goal_is_pending_still_waits_for_nav2_terminal_result(queue):
    queue._on_goals(make_goal_array(2))
    queue._goal_pending = True

    queue._on_cancel(None)
    goal_handle = AcceptedGoalHandle()
    queue._on_goal_response(queue._task_id, CompletedFuture(goal_handle))

    assert goal_handle.cancel_requested is True
    assert goal_handle.result_future.callback is not None
    assert queue._claimed is True


def prepare_waiting_for_stop(queue):
    queue._on_goals(make_goal_array(2))
    queue._records[0].start(8_000_000_000)
    queue._records[0].set_state(
        NavigationPointState.WAITING_FOR_STOP,
        'waiting for confirmed zero velocity',
    )
    queue._state = 'waiting_for_stop'
    queue._stop_started_ns = 9_000_000_000
    queue._latest_amcl_xy = (1.0, 2.0)


def test_monitor_advances_only_after_fresh_continuous_zero_velocity(queue):
    prepare_waiting_for_stop(queue)
    queue._zero_since_ns = 9_500_000_000
    queue._last_velocity_ns = 10_000_000_000

    queue._monitor()

    assert queue._records[0].state == NavigationPointState.SUCCEEDED
    assert queue._index == 1


def test_monitor_does_not_advance_with_stale_velocity(queue):
    prepare_waiting_for_stop(queue)
    queue._zero_since_ns = 9_000_000_000
    queue._last_velocity_ns = 9_700_000_000

    queue._monitor()

    assert queue._records[0].state == NavigationPointState.WAITING_FOR_STOP
    assert queue._index == 0


def test_localization_loss_fails_current_point_and_stops_later_goals(queue):
    queue._on_goals(make_goal_array(3))
    queue._records[0].start(9_000_000_000)
    queue._records[0].set_state(
        NavigationPointState.NAVIGATING,
        'Nav2 accepted goal',
    )
    queue._last_localization_ns = None
    queue._localization_error = 'AMCL pose contains non-finite values'

    queue._monitor()

    assert queue._claimed is False
    assert [point.state for point in queue._records] == [
        NavigationPointState.FAILED,
        NavigationPointState.NOT_EXECUTED,
        NavigationPointState.NOT_EXECUTED,
    ]
    assert queue._task_state.messages[-1].state == NavigationTaskState.FAILED


def test_three_point_sequence_waits_for_stop_and_never_skips_failed_point(queue):
    clock = SimpleNamespace(now_ns=10_000_000_000)
    queue._now = lambda: clock.now_ns
    queue._last_localization_ns = clock.now_ns
    queue._latest_amcl_xy = (1.0, 2.0)
    queue._client = FakeActionClient()
    queue._send_current_goal = NavigationTaskQueue._send_current_goal.__get__(
        queue, NavigationTaskQueue)

    queue._on_goals(make_goal_array(3))
    assert [request.pose.pose.position.x
            for request in queue._client.requests] == [1.0]

    first_handle = AcceptedGoalHandle()
    queue._client.response_futures[0].resolve(first_handle)
    clock.now_ns = 10_500_000_000
    queue._last_localization_ns = clock.now_ns
    first_handle.result_future.resolve(
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    zero_velocity = Twist()
    queue._on_velocity(zero_velocity)

    clock.now_ns = 10_990_000_000
    queue._last_localization_ns = clock.now_ns
    queue._on_velocity(zero_velocity)
    queue._monitor()
    assert len(queue._client.requests) == 1

    clock.now_ns = 11_000_000_000
    queue._last_localization_ns = clock.now_ns
    queue._on_velocity(zero_velocity)
    queue._monitor()
    assert [request.pose.pose.position.x
            for request in queue._client.requests] == [1.0, 2.0]

    second_handle = AcceptedGoalHandle()
    queue._client.response_futures[1].resolve(second_handle)
    clock.now_ns = 11_400_000_000
    queue._last_localization_ns = clock.now_ns
    second_handle.result_future.resolve(
        SimpleNamespace(status=GoalStatus.STATUS_ABORTED))

    assert len(queue._client.requests) == 2
    final_snapshot = queue._task_state.messages[-1]
    assert final_snapshot.active is False
    assert final_snapshot.current_index == 2
    assert [point.state for point in final_snapshot.points] == [
        NavigationPointState.SUCCEEDED,
        NavigationPointState.FAILED,
        NavigationPointState.NOT_EXECUTED,
    ]


def test_task_state_publisher_is_reliable_and_transient_local(queue):
    assert queue._task_state_qos.reliability == ReliabilityPolicy.RELIABLE
    assert queue._task_state_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_excessive_localization_covariance_rejects_submission(queue):
    localization = PoseWithCovarianceStamped()
    localization.header.frame_id = 'map'
    localization.pose.pose.orientation.w = 1.0
    localization.pose.covariance[0] = 0.30

    queue._on_amcl_pose(localization)
    queue._on_goals(make_goal_array(2))

    snapshot = queue._task_state.messages[-1]
    assert snapshot.state == NavigationTaskState.REJECTED
    assert 'position variance' in snapshot.detail


@pytest.mark.parametrize('index, value', [
    (7, math.nan),
    (35, math.inf),
    (0, -0.01),
    (7, -0.01),
    (35, -0.01),
])
def test_invalid_localization_covariance_rejects_submission(queue, index, value):
    localization = PoseWithCovarianceStamped()
    localization.header.frame_id = 'map'
    localization.pose.pose.orientation.w = 1.0
    localization.pose.covariance[index] = value

    queue._on_amcl_pose(localization)
    queue._on_goals(make_goal_array(1))

    snapshot = queue._task_state.messages[-1]
    assert snapshot.state == NavigationTaskState.REJECTED
    assert 'covariance' in snapshot.detail


def test_unavailable_nav2_server_fails_first_point_without_dispatch(queue):
    queue._client = UnavailableActionClient()
    queue._send_current_goal = NavigationTaskQueue._send_current_goal.__get__(
        queue, NavigationTaskQueue)

    queue._on_goals(make_goal_array(2))

    snapshot = queue._task_state.messages[-1]
    assert snapshot.active is False
    assert [point.state for point in snapshot.points] == [
        NavigationPointState.FAILED,
        NavigationPointState.NOT_EXECUTED,
    ]
    assert 'not ready' in snapshot.detail


def test_stop_confirmation_timeout_fails_and_blocks_later_points(queue):
    prepare_waiting_for_stop(queue)
    queue._now = lambda: 13_000_000_001
    queue._last_localization_ns = 13_000_000_001

    queue._monitor()

    snapshot = queue._task_state.messages[-1]
    assert snapshot.active is False
    assert [point.state for point in snapshot.points] == [
        NavigationPointState.FAILED,
        NavigationPointState.NOT_EXECUTED,
    ]
    assert snapshot.detail == 'zero-velocity confirmation timed out'
