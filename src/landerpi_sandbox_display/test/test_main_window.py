import os
import math

import pytest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from landerpi_msgs.msg import NavigationPointState, NavigationTaskState
from nav_msgs.msg import OccupancyGrid
from landerpi_sandbox_display.ui_state import DisplayState

os.environ.setdefault(
    'QT_QPA_PLATFORM',
    'offscreen',
)

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QLabel
from landerpi_sandbox_display.main_window import MainWindow
from landerpi_sandbox_display.map_widget import MapWidget


app = QApplication.instance()

if app is None:
    app = QApplication([])


def test_main_window_has_console_layout():
    window = MainWindow()

    assert window.windowTitle() == (
        'LanderPi Navigation Console'
    )

    assert isinstance(
        window.map_widget,
        MapWidget,
    )

    assert window.robot_state_panel is not None
    assert window.navigation_task_panel is not None
    assert window.map_info_panel is not None
    assert window.event_log_panel is not None

    window.close()

def test_main_window_updates_map():
    window = MainWindow()

    msg = OccupancyGrid()

    msg.info.width = 2
    msg.info.height = 1
    msg.info.resolution = 0.5
    msg.info.origin.position.x = -1.0
    msg.info.origin.position.y = -2.0

    msg.data = [
        0,
        100,
    ]

    window.update_map(msg)

    assert window.map_widget.map_image is not None

    assert window.coordinate_transform.map_width == 2
    assert window.coordinate_transform.map_height == 1
    assert window.coordinate_transform.resolution == 0.5
    assert window.coordinate_transform.origin_x == -1.0
    assert window.coordinate_transform.origin_y == -2.0

    window.close()

def test_main_window_updates_robot_pose():
    window = MainWindow()

    msg = PoseStamped()

    msg.pose.position.x = 1.2
    msg.pose.position.y = -0.8

    # yaw = 90°
    msg.pose.orientation.z = math.sin(
        math.pi / 4.0
    )

    msg.pose.orientation.w = math.cos(
        math.pi / 4.0
    )

    window.update_robot_pose(msg)

    x, y, yaw = window.map_widget.robot_pose

    assert x == 1.2
    assert y == -0.8
    assert abs(yaw - math.pi / 2.0) < 1e-6

    window.close()

def test_main_window_updates_global_plan():
    window = MainWindow()

    msg = Path()

    pose_a = PoseStamped()
    pose_a.pose.position.x = 1.0
    pose_a.pose.position.y = 2.0

    pose_b = PoseStamped()
    pose_b.pose.position.x = 3.0
    pose_b.pose.position.y = 4.0

    msg.poses = [
        pose_a,
        pose_b,
    ]

    window.update_global_plan(msg)

    assert window.map_widget.global_path == [
        (1.0, 2.0),
        (3.0, 4.0),
    ]

    window.close()

def test_main_window_updates_local_plan():
    window = MainWindow()

    msg = Path()

    pose = PoseStamped()
    pose.pose.position.x = 0.5
    pose.pose.position.y = -0.5

    msg.poses = [
        pose,
    ]

    window.update_local_plan(msg)

    assert window.map_widget.local_path == [
        (0.5, -0.5),
    ]

    window.close()

def test_main_window_updates_actual_path():
    window = MainWindow()

    msg = Path()

    pose_a = PoseStamped()
    pose_a.pose.position.x = -1.0
    pose_a.pose.position.y = 0.2

    pose_b = PoseStamped()
    pose_b.pose.position.x = -0.5
    pose_b.pose.position.y = 0.8

    msg.poses = [
        pose_a,
        pose_b,
    ]

    window.update_actual_path(msg)

    assert window.map_widget.actual_path == [
        (-1.0, 0.2),
        (-0.5, 0.8),
    ]

    window.close()

def test_main_window_handles_map_click_and_calls_goal_callback():
    window = MainWindow()

    window.coordinate_transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    published = []

    window.set_goal_publish_callback(
        lambda x, y: published.append((x, y))
    )

    viewport = (
        window.coordinate_transform.widget_viewport(
            window.map_widget.width(),
            window.map_widget.height(),
        )
    )

    assert viewport is not None

    viewport_x, viewport_y, viewport_width, viewport_height = viewport

    click_x = viewport_x + viewport_width / 2.0
    click_y = viewport_y + viewport_height / 2.0

    expected = (
        window.coordinate_transform.widget_to_world(
            click_x,
            click_y,
            window.map_widget.width(),
            window.map_widget.height(),
        )
    )

    assert expected is not None

    window.handle_map_click(
        click_x,
        click_y,
    )

    assert window.map_widget.goal_pose == expected
    assert published == [expected]

    window.close()

def test_main_window_connects_map_widget_click_signal():
    window = MainWindow()

    window.coordinate_transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    published = []

    window.set_goal_publish_callback(
        lambda x, y: published.append((x, y))
    )

    viewport = (
        window.coordinate_transform.widget_viewport(
            window.map_widget.width(),
            window.map_widget.height(),
        )
    )

    assert viewport is not None

    viewport_x, viewport_y, viewport_width, viewport_height = viewport

    click_x = viewport_x + viewport_width / 2.0
    click_y = viewport_y + viewport_height / 2.0

    window.map_widget.clicked.emit(
        click_x,
        click_y,
    )

    assert len(published) == 1
    assert window.map_widget.goal_pose == published[0]

    window.close()

def test_main_window_robot_state_starts_empty():
    window = MainWindow()

    assert window.robot_x_value.text() == '--'
    assert window.robot_y_value.text() == '--'
    assert window.robot_yaw_value.text() == '--'

    window.close()

def test_main_window_robot_state_displays_pose():
    window = MainWindow()

    msg = PoseStamped()

    msg.pose.position.x = 1.2
    msg.pose.position.y = -0.8

    msg.pose.orientation.z = math.sin(
        math.pi / 4.0
    )

    msg.pose.orientation.w = math.cos(
        math.pi / 4.0
    )

    window.update_robot_pose(msg)

    assert window.robot_x_value.text() == '1.200 m'
    assert window.robot_y_value.text() == '-0.800 m'
    assert window.robot_yaw_value.text() == '90.0°'

    window.close()

def test_main_window_navigation_task_starts_empty():
    window = MainWindow()

    assert window.goal_x_value.text() == '--'
    assert window.goal_y_value.text() == '--'

    assert window.global_path_value.text() == '0 points'
    assert window.local_path_value.text() == '0 points'
    assert window.actual_path_value.text() == '0 points'

    window.close()

def test_main_window_navigation_task_displays_goal():
    window = MainWindow()

    window.coordinate_transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    viewport = (
        window.coordinate_transform.widget_viewport(
            window.map_widget.width(),
            window.map_widget.height(),
        )
    )

    assert viewport is not None

    viewport_x, viewport_y, viewport_width, viewport_height = viewport

    click_x = viewport_x + viewport_width / 2.0
    click_y = viewport_y + viewport_height / 2.0

    expected = (
        window.coordinate_transform.widget_to_world(
            click_x,
            click_y,
            window.map_widget.width(),
            window.map_widget.height(),
        )
    )

    assert expected is not None

    window.handle_map_click(
        click_x,
        click_y,
    )

    x, y = expected

    assert window.goal_x_value.text() == f'{x:.3f} m'
    assert window.goal_y_value.text() == f'{y:.3f} m'

    window.close()

def test_main_window_navigation_task_displays_path_counts():
    window = MainWindow()

    msg = Path()

    pose_a = PoseStamped()
    pose_a.pose.position.x = 1.0
    pose_a.pose.position.y = 2.0

    pose_b = PoseStamped()
    pose_b.pose.position.x = 3.0
    pose_b.pose.position.y = 4.0

    pose_c = PoseStamped()
    pose_c.pose.position.x = 5.0
    pose_c.pose.position.y = 6.0

    msg.poses = [
        pose_a,
        pose_b,
        pose_c,
    ]

    window.update_global_plan(msg)
    window.update_local_plan(msg)
    window.update_actual_path(msg)

    assert window.global_path_value.text() == '3 points'
    assert window.local_path_value.text() == '3 points'
    assert window.actual_path_value.text() == '3 points'

    window.close()

def test_main_window_localization_starts_empty():
    window = MainWindow()

    assert window.map_frame_value.text() == '--'
    assert window.map_resolution_value.text() == '--'
    assert window.map_size_value.text() == '--'
    assert window.map_origin_value.text() == '--'

    window.close()

def test_main_window_localization_displays_map_info():
    window = MainWindow()

    msg = OccupancyGrid()

    msg.header.frame_id = 'map'

    msg.info.width = 80
    msg.info.height = 102
    msg.info.resolution = 0.05

    msg.info.origin.position.x = -2.92
    msg.info.origin.position.y = -2.56

    msg.data = [0] * (
        msg.info.width
        * msg.info.height
    )

    window.update_map(msg)

    assert window.map_frame_value.text() == 'map'
    assert window.map_resolution_value.text() == '0.050 m/cell'
    assert window.map_size_value.text() == '80 × 102'
    assert window.map_origin_value.text() == '(-2.920, -2.560)'

    window.close()

def test_main_window_uses_map_info_panel_title():
    window = MainWindow()

    labels = window.map_info_panel.findChildren(QLabel)

    assert any(
        label.text() == 'Map Info'
        for label in labels
    )

    window.close()

def test_refresh_from_state_displays_position_error():
    app = QApplication.instance()

    if app is None:
        app = QApplication([])

    window = MainWindow()

    state = DisplayState()
    state.update_position_error(0.123)

    window.refresh_from_state(state)

    assert window.position_error_value.text() == '0.123 m'

    window.close()

def test_refresh_from_state_displays_navigation_status():
    app = QApplication.instance()

    if app is None:
        app = QApplication([])

    window = MainWindow()
    state = DisplayState()

    state.update_navigation_status(
        'navigating'
    )

    window.refresh_from_state(
        state
    )

    assert (
        window.navigation_status_value.text()
        == 'Navigating'
    )

    window.close()


def test_refresh_from_state_displays_arrival_status():
    app = QApplication.instance()

    if app is None:
        app = QApplication([])

    window = MainWindow()
    state = DisplayState()

    state.update_arrival_status(
        True
    )

    window.refresh_from_state(
        state
    )

    assert (
        window.arrival_status_value.text()
        == 'Arrived'
    )

    window.close()


def configure_test_map(window):
    window.coordinate_transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )
    window.map_widget.resize(500, 500)


def click_world(window, x, y):
    point = window.coordinate_transform.world_to_widget(
        x,
        y,
        window.map_widget.width(),
        window.map_widget.height(),
    )
    assert point is not None
    window.handle_map_click(*point)


def make_task_snapshot(active=True, task_state=NavigationTaskState.ACTIVE):
    snapshot = NavigationTaskState()
    snapshot.task_id = 12
    snapshot.active = active
    snapshot.current_index = 2
    snapshot.total_points = 3
    snapshot.state = task_state
    snapshot.elapsed_time_s = 7.5
    snapshot.detail = 'executing point 2'
    point_states = (
        NavigationPointState.SUCCEEDED,
        NavigationPointState.NAVIGATING,
        NavigationPointState.PENDING,
    )
    for index, state in enumerate(point_states, start=1):
        point = NavigationPointState()
        point.index = index
        point.target.position.x = float(index)
        point.target.position.y = float(index + 10)
        point.target.orientation.w = 1.0
        point.state = state
        point.has_arrival_error = index == 1
        point.arrival_error_m = 0.04 if index == 1 else 0.0
        point.elapsed_time_s = float(index)
        point.retry_count = 0
        point.detail = f'point {index}'
        snapshot.points.append(point)
    return snapshot


def test_multi_goal_controls_start_in_single_goal_mode():
    window = MainWindow()

    assert window.single_goal_mode_button.isChecked()
    assert not window.multi_goal_mode_button.isChecked()
    assert window.task_table.isHidden()
    assert not window.goal_x_value.parentWidget().isHidden()
    assert not window.submit_task_button.isEnabled()
    assert not window.cancel_task_button.isEnabled()
    assert window.task_table.rowCount() == 3

    window.close()


def test_switching_mode_reuses_the_navigation_panel_space():
    window = MainWindow()

    window.multi_goal_mode_button.setChecked(True)

    assert not window.task_table.isHidden()
    assert window.goal_x_value.parentWidget().isHidden()
    assert window.navigation_status_value.parentWidget().isHidden()

    window.close()


def test_multi_goal_mode_collects_three_ordered_clicks_without_single_publish():
    window = MainWindow()
    configure_test_map(window)
    single_published = []
    window.set_goal_publish_callback(
        lambda x, y: single_published.append((x, y)))
    window.multi_goal_mode_button.setChecked(True)

    click_world(window, 1.0, 1.0)
    click_world(window, 2.0, 1.0)
    click_world(window, 2.0, 3.0)
    click_world(window, 4.0, 4.0)

    assert [(goal.x, goal.y) for goal in window.task_draft.goals()] == [
        (1.0, 1.0),
        (2.0, 1.0),
        (2.0, 3.0),
    ]
    assert single_published == []
    assert [marker.index for marker in window.map_widget.task_markers] == [1, 2, 3]
    assert window.submit_task_button.isEnabled()

    window.close()


def test_task_undo_clear_and_yaw_edit_update_the_draft():
    window = MainWindow()
    configure_test_map(window)
    window.multi_goal_mode_button.setChecked(True)
    click_world(window, 1.0, 1.0)
    click_world(window, 2.0, 1.0)

    window.task_table.item(0, 3).setText('90.0')
    assert window.task_draft.goals()[0].yaw == pytest.approx(math.pi / 2.0)
    assert window.task_draft.goals()[0].yaw_manual is True

    window.undo_task_button.click()
    assert len(window.task_draft.goals()) == 1
    window.clear_task_button.click()
    assert window.task_draft.goals() == ()
    assert window.map_widget.task_markers == []

    window.close()


def test_submit_publishes_the_whole_draft_once_and_cancel_is_one_shot():
    window = MainWindow()
    configure_test_map(window)
    window.multi_goal_mode_button.setChecked(True)
    click_world(window, 1.0, 1.0)
    click_world(window, 2.0, 2.0)
    published = []
    cancelled = []
    window.set_task_publish_callback(
        lambda goals: published.append(goals) or True)
    window.set_task_cancel_callback(lambda: cancelled.append(True))

    window.submit_task_button.click()
    assert len(published) == 1
    assert len(published[0]) == 2

    window.update_navigation_task_state(make_task_snapshot(active=True))
    window.cancel_task_button.click()
    assert cancelled == [True]
    assert not window.cancel_task_button.isEnabled()
    window.cancel_task_button.click()
    assert cancelled == [True]
    window.update_navigation_task_state(make_task_snapshot(active=True))
    assert window.cancel_task_button.isEnabled()

    window.close()


def test_initial_idle_snapshot_keeps_default_single_goal_workflow():
    window = MainWindow()
    idle = NavigationTaskState()
    idle.state = NavigationTaskState.IDLE

    window.update_navigation_task_state(idle)

    assert window.single_goal_mode_button.isChecked()
    assert not window.multi_goal_mode_button.isChecked()
    window.close()


def test_active_task_snapshot_locks_editing_and_displays_point_results():
    window = MainWindow()
    state = DisplayState()
    state.update_navigation_task_state(make_task_snapshot(active=True))

    window.refresh_from_state(state)

    assert not window.submit_task_button.isEnabled()
    assert not window.single_goal_mode_button.isEnabled()
    assert not window.undo_task_button.isEnabled()
    assert not window.clear_task_button.isEnabled()
    assert window.cancel_task_button.isEnabled()
    assert window.task_table.item(0, 4).text() == 'Succeeded'
    assert window.task_table.item(0, 5).text() == '0.040 m'
    assert window.task_table.item(1, 4).text() == 'Navigating'
    assert [marker.state for marker in window.map_widget.task_markers] == [
        'succeeded', 'navigating', 'pending',
    ]

    window.close()


def test_terminal_snapshot_unlocks_controls_and_keeps_results_visible():
    window = MainWindow()
    active = make_task_snapshot(active=True)
    window.update_navigation_task_state(active)
    terminal = make_task_snapshot(
        active=False,
        task_state=NavigationTaskState.FAILED,
    )
    terminal.detail = 'point 2 failed'

    window.update_navigation_task_state(terminal)

    assert window.single_goal_mode_button.isEnabled()
    assert window.undo_task_button.isEnabled()
    assert window.clear_task_button.isEnabled()
    assert not window.cancel_task_button.isEnabled()
    assert window.task_table.item(1, 4).text() == 'Navigating'
    assert 'Failed' in window.task_summary_value.text()

    window.close()


def test_rejected_submission_without_point_records_keeps_draft_visible():
    window = MainWindow()
    configure_test_map(window)
    window.multi_goal_mode_button.setChecked(True)
    click_world(window, 1.0, 1.0)
    click_world(window, 2.0, 1.0)
    click_world(window, 2.0, 2.0)

    rejected = NavigationTaskState()
    rejected.task_id = 13
    rejected.state = NavigationTaskState.REJECTED
    rejected.detail = 'AMCL position variance exceeds threshold'
    window.update_navigation_task_state(rejected)

    assert len(window.task_draft.goals()) == 3
    assert len(window.map_widget.task_markers) == 3
    assert window.task_table.item(0, 4).text() == 'Draft'
    assert 'Rejected' in window.task_summary_value.text()
    assert window.submit_task_button.isEnabled()

    window.close()
