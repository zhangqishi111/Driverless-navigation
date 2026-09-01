import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QColor, QImage
from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtTest import QTest

from landerpi_sandbox_display.map_widget import MapWidget
from landerpi_sandbox_display.coordinate_transform import CoordinateTransform
from landerpi_sandbox_display.sandbox_display_node import (
    SandboxDisplayNode,
    downsample_poses,
    transform_to_pose_stamped,
)


app = QApplication.instance()

if app is None:
    app = QApplication([])


def test_map_widget_initial_state():
    widget = MapWidget()

    assert widget.map_image is None

    assert widget.global_path == []
    assert widget.local_path == []
    assert widget.actual_path == []

    assert widget.robot_pose is None
    assert widget.goal_pose is None

def test_set_map_image():
    widget = MapWidget()

    fake_image = object()

    widget.set_map_image(fake_image)

    assert widget.map_image is fake_image


def test_set_global_path():
    widget = MapWidget()

    path = [
        (0.0, 0.0),
        (1.0, 1.0),
    ]

    widget.set_global_path(path)

    assert widget.global_path == path


def test_set_local_path():
    widget = MapWidget()

    path = [
        (0.0, 0.0),
        (0.5, 0.2),
    ]

    widget.set_local_path(path)

    assert widget.local_path == path


def test_set_actual_path():
    widget = MapWidget()

    path = [
        (0.0, 0.0),
        (0.1, 0.0),
        (0.2, 0.1),
    ]

    widget.set_actual_path(path)

    assert widget.actual_path == path


def test_set_robot_pose():
    widget = MapWidget()

    pose = (1.2, -0.5, 1.57)

    widget.set_robot_pose(pose)

    assert widget.robot_pose == pose


def test_set_goal_pose():
    widget = MapWidget()

    goal = (2.0, 3.0)

    widget.set_goal_pose(goal)

    assert widget.goal_pose == goal

def test_data_setters_request_repaint():
    class TrackingMapWidget(MapWidget):
        def __init__(self):
            self.update_count = 0
            super().__init__()

        def update(self):
            self.update_count += 1

    widget = TrackingMapWidget()
    initial_count = widget.update_count

    widget.set_map_image(object())
    widget.set_global_path([(0.0, 0.0)])
    widget.set_local_path([(0.0, 0.0)])
    widget.set_actual_path([(0.0, 0.0)])
    widget.set_robot_pose((0.0, 0.0, 0.0))
    widget.set_goal_pose((1.0, 1.0))

    assert widget.update_count == initial_count + 6

def test_map_widget_renders_map_without_smoothing():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=2,
        height=1,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    widget = MapWidget(
        coordinate_transform=transform,
    )

    widget.resize(4, 2)

    map_image = QImage(
        2,
        1,
        QImage.Format_RGB32,
    )

    map_image.setPixelColor(
        0,
        0,
        QColor(0, 0, 0),
    )

    map_image.setPixelColor(
        1,
        0,
        QColor(255, 255, 255),
    )

    widget.set_map_image(map_image)

    rendered = QImage(
        widget.size(),
        QImage.Format_RGB32,
    )

    rendered.fill(
        QColor(128, 128, 128)
    )

    widget.render(rendered)

    # 2x1 地图放大成 4x2。
    # 不做平滑插值时：
    # 左半部分应该仍然纯黑，
    # 右半部分应该仍然纯白。
    assert rendered.pixelColor(
        1,
        0,
    ) == QColor(0, 0, 0)

    assert rendered.pixelColor(
        2,
        0,
    ) == QColor(255, 255, 255)

def test_map_widget_renders_global_path():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    widget = MapWidget(
        coordinate_transform=transform,
    )

    widget.resize(100, 100)

    map_image = QImage(
        10,
        10,
        QImage.Format_RGB32,
    )

    map_image.fill(
        QColor(255, 255, 255)
    )

    widget.set_map_image(map_image)

    widget.set_global_path([
        (1.0, 1.0),
        (8.0, 8.0),
    ])

    rendered = QImage(
        widget.size(),
        QImage.Format_RGB32,
    )

    rendered.fill(
        QColor(255, 255, 255)
    )

    widget.render(rendered)

    # 路径中心附近应该出现明显的蓝色分量。
    color = rendered.pixelColor(
        45,
        45,
    )

    assert color.blue() > color.red()
    assert color.blue() > color.green()

def test_map_widget_renders_local_path():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    widget = MapWidget(
        coordinate_transform=transform,
    )

    widget.resize(100, 100)

    map_image = QImage(
        10,
        10,
        QImage.Format_RGB32,
    )
    map_image.fill(QColor(255, 255, 255))

    widget.set_map_image(map_image)

    widget.set_local_path([
        (1.0, 2.0),
        (8.0, 2.0),
    ])

    rendered = QImage(
        widget.size(),
        QImage.Format_RGB32,
    )
    rendered.fill(QColor(255, 255, 255))

    widget.render(rendered)

    color = rendered.pixelColor(
        50,
        70,
    )

    assert color.green() > color.red()
    assert color.green() > color.blue()


def test_map_widget_renders_actual_path():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    widget = MapWidget(
        coordinate_transform=transform,
    )

    widget.resize(100, 100)

    map_image = QImage(
        10,
        10,
        QImage.Format_RGB32,
    )
    map_image.fill(QColor(255, 255, 255))

    widget.set_map_image(map_image)

    widget.set_actual_path([
        (1.0, 5.0),
        (8.0, 5.0),
    ])

    rendered = QImage(
        widget.size(),
        QImage.Format_RGB32,
    )
    rendered.fill(QColor(255, 255, 255))

    widget.render(rendered)

    color = rendered.pixelColor(
        50,
        40,
    )

    assert color.red() > color.green()
    assert color.red() > color.blue()

def test_map_widget_renders_robot_pose():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    widget = MapWidget(
        coordinate_transform=transform,
    )

    widget.resize(100, 100)

    map_image = QImage(
        10,
        10,
        QImage.Format_RGB32,
    )
    map_image.fill(QColor(255, 255, 255))

    widget.set_map_image(map_image)

    widget.set_robot_pose(
        (5.0, 5.0, 0.0)
    )

    rendered = QImage(
        widget.size(),
        QImage.Format_RGB32,
    )
    rendered.fill(QColor(255, 255, 255))

    widget.render(rendered)

    # (5, 5) 映射到窗口中心附近，
    # 机器人主体应具有明显的紫色特征。
    color = rendered.pixelColor(
        50,
        40,
    )

    assert color.red() > color.green()
    assert color.blue() > color.green()


def test_map_widget_renders_goal_pose():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    widget = MapWidget(
        coordinate_transform=transform,
    )

    widget.resize(100, 100)

    map_image = QImage(
        10,
        10,
        QImage.Format_RGB32,
    )
    map_image.fill(QColor(255, 255, 255))

    widget.set_map_image(map_image)

    widget.set_goal_pose(
        (5.0, 5.0)
    )

    rendered = QImage(
        widget.size(),
        QImage.Format_RGB32,
    )
    rendered.fill(QColor(255, 255, 255))

    widget.render(rendered)

    color = rendered.pixelColor(
        50,
        40,
    )

    # Goal 使用橙色：
    # 红色分量高，蓝色分量低。
    assert color.red() > color.green()
    assert color.green() > color.blue()

def test_map_widget_emits_clicked_signal_on_left_click():
    widget = MapWidget()
    widget.resize(100, 100)

    received = []

    widget.clicked.connect(
        lambda x, y: received.append((x, y))
    )

    QTest.mouseClick(
        widget,
        Qt.LeftButton,
        Qt.NoModifier,
        QPoint(23, 37),
    )

    assert received == [
        (23.0, 37.0)
    ]

def test_downsample_poses_limits_size_and_preserves_ends():
    poses = list(range(2000))

    result = downsample_poses(
        poses,
        max_points=800,
    )

    assert len(result) == 800
    assert result[0] == 0
    assert result[-1] == 1999