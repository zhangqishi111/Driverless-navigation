import math

import pytest

from landerpi_sandbox_display.task_draft import TaskDraft, yaw_to_quaternion


def test_draft_keeps_click_order_and_rejects_fourth_goal():
    draft = TaskDraft(max_goals=3)

    assert draft.add(0.0, 0.0, robot_yaw=0.2)
    assert draft.add(1.0, 0.0, robot_yaw=0.2)
    assert draft.add(1.0, 1.0, robot_yaw=0.2)
    assert not draft.add(2.0, 2.0, robot_yaw=0.2)

    assert [(goal.x, goal.y) for goal in draft.goals()] == [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
    ]


def test_automatic_yaw_points_each_goal_toward_the_next_goal():
    draft = TaskDraft()
    draft.add(0.0, 0.0, robot_yaw=-0.3)
    draft.add(1.0, 0.0, robot_yaw=-0.3)
    draft.add(1.0, 1.0, robot_yaw=-0.3)

    goals = draft.goals()

    assert goals[0].yaw == pytest.approx(0.0)
    assert goals[1].yaw == pytest.approx(math.pi / 2.0)
    assert goals[2].yaw == pytest.approx(math.pi / 2.0)


def test_single_goal_uses_robot_yaw_and_defaults_to_zero_when_unknown():
    known = TaskDraft()
    unknown = TaskDraft()

    known.add(2.0, 3.0, robot_yaw=0.7)
    unknown.add(2.0, 3.0, robot_yaw=None)

    assert known.goals()[0].yaw == pytest.approx(0.7)
    assert unknown.goals()[0].yaw == pytest.approx(0.0)


def test_manual_yaw_is_not_overwritten_by_later_click():
    draft = TaskDraft()
    draft.add(0.0, 0.0, robot_yaw=0.0)
    draft.set_yaw(0, 1.25)

    draft.add(1.0, 1.0, robot_yaw=0.0)

    assert draft.goals()[0].yaw == pytest.approx(1.25)
    assert draft.goals()[0].yaw_manual is True


def test_undo_recomputes_the_new_last_automatic_yaw():
    draft = TaskDraft()
    draft.add(0.0, 0.0, robot_yaw=0.0)
    draft.add(1.0, 0.0, robot_yaw=0.0)
    draft.add(1.0, 1.0, robot_yaw=0.0)

    draft.undo()

    assert [(goal.x, goal.y) for goal in draft.goals()] == [
        (0.0, 0.0),
        (1.0, 0.0),
    ]
    assert draft.goals()[-1].yaw == pytest.approx(0.0)


def test_clear_removes_all_goals():
    draft = TaskDraft()
    draft.add(0.0, 0.0, robot_yaw=0.0)

    draft.clear()

    assert draft.goals() == ()


@pytest.mark.parametrize('bad_value', [math.nan, math.inf, -math.inf])
def test_non_finite_coordinates_are_rejected(bad_value):
    draft = TaskDraft()

    assert not draft.add(bad_value, 0.0, robot_yaw=0.0)
    assert not draft.add(0.0, bad_value, robot_yaw=0.0)
    assert draft.goals() == ()


def test_non_finite_manual_yaw_is_rejected_without_mutating_goal():
    draft = TaskDraft()
    draft.add(0.0, 0.0, robot_yaw=0.4)

    with pytest.raises(ValueError):
        draft.set_yaw(0, math.nan)

    assert draft.goals()[0].yaw == pytest.approx(0.4)
    assert draft.goals()[0].yaw_manual is False


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2.0, -2.7, 10.0])
def test_yaw_quaternion_is_planar_and_normalized(yaw):
    x, y, z, w = yaw_to_quaternion(yaw)

    assert x == 0.0
    assert y == 0.0
    assert math.sqrt(z * z + w * w) == pytest.approx(1.0)
