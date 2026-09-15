from types import SimpleNamespace

import pytest

from landerpi_msgs.msg import NavigationPointState
from landerpi_task_manager.nav_grasp_coordinator import NavGraspCoordinator


class PublisherCapture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class ThreadCapture:
    created = []

    def __init__(self, target=None, daemon=None):
        self.target = target
        self.daemon = daemon
        ThreadCapture.created.append(self)

    def start(self):
        pass


def point(state):
    return SimpleNamespace(state=state)


def message(
        task_id=1,
        active=True,
        current_index=1,
        states=None,
):
    if states is None:
        states = [
            NavigationPointState.PENDING,
            NavigationPointState.PENDING,
            NavigationPointState.PENDING,
        ]

    return SimpleNamespace(
        task_id=task_id,
        active=active,
        current_index=current_index,
        total_points=3,
        points=[point(state) for state in states],
    )


def make_coordinator():
    node = object.__new__(NavGraspCoordinator)

    node.enabled = True
    node.task_id = 1

    node.p1_pause_requested = False
    node.p2_pause_requested = False

    node.grasp_started = False
    node.grasp_finished = False

    node.place_started = False
    node.place_finished = False

    node.pause_pub = PublisherCapture()
    node.resume_pub = PublisherCapture()

    node.grasp_then_resume = lambda: None
    node.place_then_resume = lambda: None

    node.get_logger = lambda: SimpleNamespace(
        info=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
        warn=lambda *args, **kwargs: None,
    )

    return node


def test_p1_requests_first_pause(monkeypatch):
    node = make_coordinator()

    monkeypatch.setattr(
        'landerpi_task_manager.nav_grasp_coordinator.threading.Thread',
        ThreadCapture,
    )

    node.state_callback(
        message(
            current_index=1,
            states=[
                NavigationPointState.NAVIGATING,
                NavigationPointState.PENDING,
                NavigationPointState.PENDING,
            ],
        )
    )

    assert node.p1_pause_requested is True
    assert len(node.pause_pub.messages) == 1


def test_p2_requests_second_pause_after_grasp_finished(monkeypatch):
    node = make_coordinator()
    node.p1_pause_requested = True
    node.grasp_started = True
    node.grasp_finished = True

    monkeypatch.setattr(
        'landerpi_task_manager.nav_grasp_coordinator.threading.Thread',
        ThreadCapture,
    )

    node.state_callback(
        message(
            current_index=2,
            states=[
                NavigationPointState.SUCCEEDED,
                NavigationPointState.NAVIGATING,
                NavigationPointState.PENDING,
            ],
        )
    )

    assert node.p2_pause_requested is True
    assert len(node.pause_pub.messages) == 1


def test_p2_success_starts_place_after_grasp(monkeypatch):
    node = make_coordinator()
    node.p1_pause_requested = True
    node.p2_pause_requested = True
    node.grasp_started = True
    node.grasp_finished = True

    ThreadCapture.created = []

    monkeypatch.setattr(
        'landerpi_task_manager.nav_grasp_coordinator.threading.Thread',
        ThreadCapture,
    )

    node.state_callback(
        message(
            current_index=2,
            states=[
                NavigationPointState.SUCCEEDED,
                NavigationPointState.SUCCEEDED,
                NavigationPointState.PENDING,
            ],
        )
    )

    assert node.place_started is True
    assert len(ThreadCapture.created) == 1


def test_p3_success_finishes_pick_and_place_mode(monkeypatch):
    node = make_coordinator()

    node.p1_pause_requested = True
    node.p2_pause_requested = True

    node.grasp_started = True
    node.grasp_finished = True

    node.place_started = True
    node.place_finished = True

    monkeypatch.setattr(
        'landerpi_task_manager.nav_grasp_coordinator.threading.Thread',
        ThreadCapture,
    )

    node.state_callback(
        message(
            active=False,
            current_index=3,
            states=[
                NavigationPointState.SUCCEEDED,
                NavigationPointState.SUCCEEDED,
                NavigationPointState.SUCCEEDED,
            ],
        )
    )

    assert node.enabled is False
