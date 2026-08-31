import math
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


from PyQt5.QtCore import QSize
from PyQt5.QtGui import QResizeEvent
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from PyQt5.QtWidgets import QApplication

from landerpi_sandbox_display.map_widget import MapWidget
from landerpi_sandbox_display.sandbox_display_node import SandboxWindow
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from landerpi_sandbox_display.map_widget import MapWidget
from landerpi_sandbox_display.sandbox_display_node import SandboxWindow


app = QApplication.instance()

if app is None:
    app = QApplication([])


def test_sandbox_window_uses_map_widget_as_central_widget():
    window = SandboxWindow()

    assert isinstance(
        window.centralWidget(),
        MapWidget,
    )

    assert window.centralWidget() is window.map_widget

    assert (
        window.map_widget.coordinate_transform
        is window.coordinate_transform
    )

    window.close()

def test_update_map_sends_image_to_map_widget():
    window = SandboxWindow()

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

    window.close()

def test_update_global_plan_sends_points_to_map_widget():
    window = SandboxWindow()

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

def test_update_local_plan_sends_points_to_map_widget():
    window = SandboxWindow()

    msg = Path()

    pose = PoseStamped()
    pose.pose.position.x = 0.5
    pose.pose.position.y = -0.5

    msg.poses = [pose]

    window.update_local_plan(msg)

    assert window.map_widget.local_path == [
        (0.5, -0.5),
    ]

    window.close()

def test_update_actual_path_sends_points_to_map_widget():
    window = SandboxWindow()

    msg = Path()

    pose = PoseStamped()
    pose.pose.position.x = -1.0
    pose.pose.position.y = 1.5

    msg.poses = [pose]

    window.update_actual_path(msg)

    assert window.map_widget.actual_path == [
        (-1.0, 1.5),
    ]

    window.close()

def test_update_robot_pose_sends_pose_to_map_widget():
    window = SandboxWindow()

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

def test_handle_map_click_updates_goal_and_calls_publisher():
    window = SandboxWindow()

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

def test_window_can_resize_after_map_update():
    window = SandboxWindow()

    msg = OccupancyGrid()

    msg.info.width = 2
    msg.info.height = 2
    msg.info.resolution = 0.5
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0

    msg.data = [
        0, 0,
        0, 100,
    ]

    window.update_map(msg)

    event = QResizeEvent(
        QSize(800, 600),
        QSize(900, 600),
    )

    window.resizeEvent(event)

    assert window.map_widget.map_image is not None

    window.close()