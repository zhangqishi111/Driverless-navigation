import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from landerpi_task_manager.command_arbiter import (
    CommandGate,
    NavigationCommandArbiter,
)


def test_gate_rejects_commands_until_startup_cleanup_finishes():
    gate = CommandGate()

    assert gate.try_reserve() is False
    gate.mark_startup_cleanup_complete()
    assert gate.try_reserve() is True


def test_simultaneous_clients_cannot_both_reserve_nav2():
    gate = CommandGate()
    gate.mark_startup_cleanup_complete()
    barrier = threading.Barrier(2)

    def reserve():
        barrier.wait()
        return gate.try_reserve()

    with ThreadPoolExecutor(max_workers=2) as executor:
        results = list(executor.map(lambda _index: reserve(), range(2)))

    assert sorted(results) == [False, True]


def test_unknown_result_keeps_gate_reserved():
    gate = CommandGate()
    gate.mark_startup_cleanup_complete()
    assert gate.try_reserve() is True

    gate.mark_outcome_unknown()

    assert gate.try_reserve() is False
    assert gate.has_unresolved_command is True


def test_shutdown_never_releases_an_unresolved_command():
    gate = CommandGate()
    gate.mark_startup_cleanup_complete()
    assert gate.try_reserve() is True

    assert gate.begin_shutdown() is True
    assert gate.try_reserve() is False
    assert gate.has_unresolved_command is True

    gate.mark_outcome_known()
    assert gate.has_unresolved_command is False
    assert gate.try_reserve() is False


def test_result_transport_failure_keeps_real_arbiter_locked_through_shutdown():
    class Logger:
        def error(self, _message):
            pass

    class DownstreamHandle:
        accepted = True

        def __init__(self, result_future):
            self._result_future = result_future
            self.cancel_requested = False

        def get_result_async(self):
            return self._result_future

        def cancel_goal_async(self):
            self.cancel_requested = True
            return object()

    class Client:
        def __init__(self, response_future):
            self._response_future = response_future

        def server_is_ready(self):
            return True

        def send_goal_async(self, _request, **_kwargs):
            return self._response_future

    class UpstreamHandle:
        request = object()
        is_active = True
        is_cancel_requested = False

        def abort(self):
            self.is_active = False

    class Harness:
        def __init__(self, client, gate):
            self._client = client
            self._gate = gate
            self._state_lock = threading.Lock()
            self._shutting_down = False
            self._downstream_handle = None
            self._downstream_result_future = None
            self._cancel_future = None

        def get_logger(self):
            return Logger()

        def _forward_feedback(self, *_args):
            pass

        def _cancel_downstream(self):
            NavigationCommandArbiter._cancel_downstream(self)

    async def exercise_failure():
        loop = asyncio.get_running_loop()
        result_future = loop.create_future()
        result_future.set_exception(RuntimeError('result transport failed'))
        downstream = DownstreamHandle(result_future)
        response_future = loop.create_future()
        response_future.set_result(downstream)
        gate = CommandGate()
        gate.mark_startup_cleanup_complete()
        assert gate.try_reserve() is True
        harness = Harness(Client(response_future), gate)

        await NavigationCommandArbiter._execute(harness, UpstreamHandle())

        assert gate.has_unresolved_command is True
        assert gate.try_reserve() is False
        assert NavigationCommandArbiter.begin_shutdown(harness) is True
        assert downstream.cancel_requested is True
        assert gate.has_unresolved_command is True

    asyncio.run(exercise_failure())


def test_startup_waits_for_canceling_orphan_to_become_terminal():
    class Logger:
        def info(self, _message):
            pass

    class Harness:
        def __init__(self):
            self._startup_lock = threading.Lock()
            self._startup_cancel_accepted = False
            self._startup_pending_goal_ids = set()
            self._latest_nav2_statuses = {}
            self._startup_finished = False
            self.finished = False

        def get_logger(self):
            return Logger()

        def _check_startup_cleanup_complete(self):
            NavigationCommandArbiter._check_startup_cleanup_complete(self)

        def _finish_startup_cleanup(self):
            self.finished = True

    goal_info = GoalInfo()
    goal_info.goal_id.uuid[0] = 42
    response = SimpleNamespace(
        return_code=0,
        goals_canceling=[goal_info],
    )
    future = SimpleNamespace(result=lambda: response)
    harness = Harness()

    NavigationCommandArbiter._on_startup_cleanup_response(harness, future)
    assert harness.finished is False

    canceling = GoalStatusArray()
    canceling_status = GoalStatus()
    canceling_status.goal_info = goal_info
    canceling_status.status = GoalStatus.STATUS_CANCELING
    canceling.status_list = [canceling_status]
    NavigationCommandArbiter._on_nav2_status(harness, canceling)
    assert harness.finished is False

    terminal = GoalStatusArray()
    terminal_status = GoalStatus()
    terminal_status.goal_info = goal_info
    terminal_status.status = GoalStatus.STATUS_CANCELED
    terminal.status_list = [terminal_status]
    NavigationCommandArbiter._on_nav2_status(harness, terminal)
    assert harness.finished is True
