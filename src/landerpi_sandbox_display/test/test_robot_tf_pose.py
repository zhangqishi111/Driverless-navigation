import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from landerpi_msgs.msg import NavigationTaskState
from types import SimpleNamespace
from std_msgs.msg import Bool, Float32, String

from landerpi_sandbox_display.sandbox_display_node import (
    SandboxDisplayNode,
    downsample_poses,
    transform_to_pose_stamped,
)
from landerpi_sandbox_display.task_draft import DraftGoal


class PublisherCapture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNow:
    def to_msg(self):
        return Time(sec=12, nanosec=345)


class FakeClock:
    def now(self):
        return FakeNow()

def test_transform_to_pose_stamped_uses_map_frame():
    transform = TransformStamped()

    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'

    transform.transform.translation.x = 1.2
    transform.transform.translation.y = -0.8
    transform.transform.translation.z = 0.1

    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.3
    transform.transform.rotation.w = 0.954

    pose = transform_to_pose_stamped(
        transform
    )

    assert pose.header.frame_id == 'map'

    assert pose.pose.position.x == 1.2
    assert pose.pose.position.y == -0.8
    assert pose.pose.position.z == 0.1

    assert pose.pose.orientation.z == 0.3
    assert pose.pose.orientation.w == 0.954

class FakeDisplayState:
    def __init__(self):
        self.robot_pose = None

    def update_robot_pose(self, pose):
        self.robot_pose = pose


class FakeTfBuffer:
    def __init__(self, transform):
        self.transform = transform

    def lookup_transform(self, *_args, **_kwargs):
        return self.transform


def test_tf_pose_also_updates_actual_path():
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.rotation.w = 1.0

    state = FakeDisplayState()
    actual_path_calls = []

    fake_node = SimpleNamespace(
        tf_buffer=FakeTfBuffer(transform),
        display_state=state,
        update_actual_path=actual_path_calls.append,
    )

    SandboxDisplayNode.update_robot_pose_from_tf(
        fake_node
    )

    assert state.robot_pose is not None
    assert len(actual_path_calls) == 1

    assert (
        actual_path_calls[0].pose.position.x
        == 1.0
    )
    assert (
        actual_path_calls[0].pose.position.y
        == 2.0
    )

def test_downsample_poses_limits_size_and_preserves_ends():
    poses = list(range(2000))

    result = downsample_poses(
        poses,
        max_points=800,
    )

    assert len(result) == 800
    assert result[0] == 0
    assert result[-1] == 1999

def test_position_error_callback_updates_display_state():
        class FakePositionErrorState:
            def __init__(self):
                self.position_error = None

            def update_position_error(self, error):
                self.position_error = error

        state = FakePositionErrorState()

        fake_node = SimpleNamespace(
            display_state=state,
        )

        msg = Float32()
        msg.data = 0.123

        SandboxDisplayNode.position_error_callback(
            fake_node,
            msg,
        )

        assert state.position_error == 0.123

def test_navigation_status_callback_updates_display_state():
    class FakeNavigationState:
        def __init__(self):
            self.navigation_status = None

        def update_navigation_status(self, status):
            self.navigation_status = status

    state = FakeNavigationState()

    fake_node = SimpleNamespace(
        display_state=state,
    )

    msg = String()
    msg.data = 'navigating'

    SandboxDisplayNode.navigation_status_callback(
        fake_node,
        msg,
    )

    assert state.navigation_status == 'navigating'


def test_arrival_status_callback_updates_display_state():
    class FakeArrivalState:
        def __init__(self):
            self.arrival_status = None

        def update_arrival_status(self, arrived):
            self.arrival_status = arrived

    state = FakeArrivalState()

    fake_node = SimpleNamespace(
        display_state=state,
    )

    msg = Bool()
    msg.data = True

    SandboxDisplayNode.arrival_status_callback(
        fake_node,
        msg,
    )

    assert state.arrival_status is True

def test_navigation_status_starts_task_timer():
    class FakeState:
        def update_navigation_status(self, status):
            self.navigation_status = status

    state = FakeState()

    fake_node = SimpleNamespace(
        display_state=state,
        task_running=False,
        task_start_time=None,
    )

    msg = String()
    msg.data = 'navigating'

    SandboxDisplayNode.navigation_status_callback(
        fake_node,
        msg,
    )

    assert fake_node.task_running is True
    assert fake_node.task_start_time is not None


def test_arrival_status_stops_task_timer():
    class FakeState:
        def update_arrival_status(self, arrived):
            self.arrival_status = arrived

        def update_task_time(self, task_time):
            self.task_time = task_time

    state = FakeState()

    fake_node = SimpleNamespace(
        display_state=state,
        task_running=True,
        task_start_time=10.0,
    )

    msg = Bool()
    msg.data = True

    SandboxDisplayNode.arrival_status_callback(
        fake_node,
        msg,
    )

    assert fake_node.task_running is False


def test_task_timer_updates_display_state():
    class FakeState:
        def __init__(self):
            self.task_time = None

        def update_task_time(self, task_time):
            self.task_time = task_time

    state = FakeState()

    fake_node = SimpleNamespace(
        display_state=state,
        task_running=True,
        task_start_time=10.0,
    )

    SandboxDisplayNode.update_task_time(
        fake_node,
        current_time=22.5,
    )

    assert state.task_time == 12.5


def test_publish_navigation_task_sends_one_ordered_map_pose_array():
    publisher = PublisherCapture()
    fake_node = SimpleNamespace(
        navigation_task_publisher=publisher,
        get_clock=lambda: FakeClock(),
    )
    goals = (
        DraftGoal(1.0, 4.0, 0.0),
        DraftGoal(2.0, 5.0, math.pi / 2.0),
        DraftGoal(3.0, 6.0, math.pi),
    )

    accepted = SandboxDisplayNode.publish_navigation_task(fake_node, goals)

    assert accepted is True
    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    assert message.header.frame_id == 'map'
    assert message.header.stamp.sec == 12
    assert message.header.stamp.nanosec == 345
    assert [pose.position.x for pose in message.poses] == [1.0, 2.0, 3.0]
    assert [pose.position.y for pose in message.poses] == [4.0, 5.0, 6.0]
    assert message.poses[1].orientation.z == pytest.approx(math.sqrt(0.5))
    assert message.poses[1].orientation.w == pytest.approx(math.sqrt(0.5))


@pytest.mark.parametrize('goals', [(), tuple(DraftGoal(i, i, 0.0) for i in range(4))])
def test_publish_navigation_task_rejects_invalid_goal_count(goals):
    publisher = PublisherCapture()
    fake_node = SimpleNamespace(
        navigation_task_publisher=publisher,
        get_clock=lambda: FakeClock(),
    )

    accepted = SandboxDisplayNode.publish_navigation_task(fake_node, goals)

    assert accepted is False
    assert publisher.messages == []


def test_cancel_navigation_task_publishes_one_empty_message():
    publisher = PublisherCapture()
    fake_node = SimpleNamespace(navigation_task_cancel_publisher=publisher)

    SandboxDisplayNode.cancel_navigation_task(fake_node)

    assert len(publisher.messages) == 1
    assert publisher.messages[0].__class__.__name__ == 'Empty'


def test_navigation_task_state_callback_updates_display_state():
    class FakeTaskState:
        def update_navigation_task_state(self, task_state):
            self.task_state = task_state

    state = FakeTaskState()
    fake_node = SimpleNamespace(display_state=state)
    message = NavigationTaskState()
    message.task_id = 7

    SandboxDisplayNode.navigation_task_state_callback(fake_node, message)

    assert state.task_state is message
