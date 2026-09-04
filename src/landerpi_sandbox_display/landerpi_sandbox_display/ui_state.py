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

    def update_global_plan(self, plan):
        self._update('global_plan', plan)

    def update_local_plan(self, plan):
        self._update('local_plan', plan)

    def update_actual_path(self, path):
        self._update('actual_path', path)

    def snapshot(self):
        with self._lock:
            return StateSnapshot(
                values=self._values.copy(),
                revisions=self._revisions.copy(),
            )