"""Pure model for drafting a short ordered navigation task."""

import math
from dataclasses import dataclass, replace
from typing import Optional, Tuple


@dataclass(frozen=True)
class DraftGoal:
    x: float
    y: float
    yaw: float
    yaw_manual: bool = False
    fallback_yaw: float = 0.0


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return a normalized planar quaternion for a finite yaw angle."""
    if not math.isfinite(yaw):
        raise ValueError('yaw must be finite')
    half_yaw = yaw / 2.0
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class TaskDraft:
    """Maintain one-to-three goals in the order selected on the map."""

    def __init__(self, max_goals: int = 3):
        if max_goals < 1:
            raise ValueError('max_goals must be positive')
        self._max_goals = int(max_goals)
        self._goals = []

    def add(self, x: float, y: float, robot_yaw: Optional[float]) -> bool:
        values = (x, y)
        if len(self._goals) >= self._max_goals:
            return False
        if not all(math.isfinite(value) for value in values):
            return False
        initial_yaw = robot_yaw
        if initial_yaw is None or not math.isfinite(initial_yaw):
            initial_yaw = 0.0
        self._goals.append(DraftGoal(
            x=float(x),
            y=float(y),
            yaw=float(initial_yaw),
            fallback_yaw=float(initial_yaw),
        ))
        self._refresh_automatic_yaws()
        return True

    def set_yaw(self, index: int, yaw: float) -> None:
        if not math.isfinite(yaw):
            raise ValueError('yaw must be finite')
        if index < 0 or index >= len(self._goals):
            raise IndexError('goal index out of range')
        self._goals[index] = replace(
            self._goals[index],
            yaw=float(yaw),
            yaw_manual=True,
        )

    def undo(self) -> None:
        if not self._goals:
            return
        self._goals.pop()
        self._refresh_automatic_yaws()

    def clear(self) -> None:
        self._goals.clear()

    def goals(self) -> Tuple[DraftGoal, ...]:
        return tuple(self._goals)

    def _refresh_automatic_yaws(self) -> None:
        if not self._goals:
            return
        if len(self._goals) == 1:
            goal = self._goals[0]
            if not goal.yaw_manual:
                self._goals[0] = replace(goal, yaw=goal.fallback_yaw)
            return
        for index in range(len(self._goals) - 1):
            goal = self._goals[index]
            if goal.yaw_manual:
                continue
            following = self._goals[index + 1]
            self._goals[index] = replace(
                goal,
                yaw=math.atan2(following.y - goal.y, following.x - goal.x),
            )
        last = self._goals[-1]
        if not last.yaw_manual:
            previous = self._goals[-2]
            self._goals[-1] = replace(
                last,
                yaw=math.atan2(last.y - previous.y, last.x - previous.x),
            )
