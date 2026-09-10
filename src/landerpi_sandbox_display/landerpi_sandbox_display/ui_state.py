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
        """
        统一更新状态值，并增加对应 revision。

        revision 用于通知 GUI 某项数据已经发生变化，
        避免每次刷新都重复更新所有控件。
        """
        with self._lock:
            self._values[name] = value
            self._revisions[name] = (
                self._revisions.get(name, 0) + 1
            )

    def update_map(self, msg):
        """更新地图数据。"""
        self._update('map', msg)

    def update_amcl_pose(self, pose):
        """更新 AMCL 定位结果。"""
        self._update('amcl_pose', pose)

    def update_robot_pose(self, pose):
        """更新机器人当前位姿。"""
        self._update('robot_pose', pose)

    def update_global_plan(self, plan):
        """更新全局规划路径。"""
        self._update('global_plan', plan)

    def update_local_plan(self, plan):
        """更新局部规划路径。"""
        self._update('local_plan', plan)

    def update_actual_path(self, path):
        """更新机器人实际运动轨迹。"""
        self._update('actual_path', path)

    def update_position_error(self, error):
        """
        更新机器人当前位置与目标位置之间的 XY 距离误差。

        参数:
            error: 位置误差，单位为米。
        """
        self._update('position_error', error)

    def snapshot(self):
        """
        获取当前 Display 状态快照。

        使用浅拷贝避免 GUI 线程直接访问内部状态容器。
        """
        with self._lock:
            return StateSnapshot(
                values=self._values.copy(),
                revisions=self._revisions.copy(),
            )