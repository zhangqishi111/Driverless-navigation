from dataclasses import dataclass
from threading import Lock


@dataclass
class StateSnapshot:
    values: dict
    revisions: dict


class DisplayState:
    def __init__(self):
        self._lock = Lock()
        self._values = {}
        self._revisions = {}

    def _update(self, name, value):
        with self._lock:
            self._values[name] = value
            self._revisions[name] = (
                self._revisions.get(name, 0) + 1
            )

    def update_map(self, msg):
        self._update('map', msg)

    def update_amcl_pose(self, pose):
        self._update('amcl_pose', pose)

    def update_robot_pose(self, pose):
        self._update('robot_pose', pose)

    def update_global_plan(self, path):
        self._update('global_plan', path)

    def update_local_plan(self, path):
        self._update('local_plan', path)

    def update_actual_path(self, path):
        self._update('actual_path', path)

    def update_position_error(self, error):
        self._update(
            'position_error',
            error,
        )

    def update_navigation_status(self, status):
        self._update(
            'navigation_status',
            status,
        )

    def update_arrival_status(self, arrived):
        self._update(
            'arrival_status',
            arrived,
        )

    def update_task_time(self, task_time):
        self._update(
            'task_time',
            task_time,
        )

    def snapshot(self):
        with self._lock:
            return StateSnapshot(
                values=dict(self._values),
                revisions=dict(
                    self._revisions
                ),
            )