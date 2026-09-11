import os
import math

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
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