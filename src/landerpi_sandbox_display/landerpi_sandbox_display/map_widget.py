import math

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import QWidget

from landerpi_sandbox_display.coordinate_transform import CoordinateTransform

class MapWidget(QWidget):

    clicked = pyqtSignal(float, float)

    def __init__(
            self,
            parent=None,
            coordinate_transform=None,
    ):
        super().__init__(parent)

        if coordinate_transform is None:
            coordinate_transform = CoordinateTransform()

        self.coordinate_transform = coordinate_transform
        self.map_image = None

        self.global_path = []
        self.local_path = []
        self.actual_path = []

        self.robot_pose = None
        self.goal_pose = None

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(
                float(event.x()),
                float(event.y()),
            )

        super().mousePressEvent(event)

    def set_map_image(self, image):
        self.map_image = image
        self.update()

    def set_global_path(self, path):
        self.global_path = path
        self.update()

    def set_local_path(self, path):
        self.local_path = path
        self.update()

    def set_actual_path(self, path):
        self.actual_path = path
        self.update()

    def set_robot_pose(self, pose):
        self.robot_pose = pose
        self.update()

    def set_goal_pose(self, pose):
        self.goal_pose = pose
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)

        painter = QPainter(self)

        try:
            painter.fillRect(
                self.rect(),
                QColor(35, 35, 35),
            )

            if self.map_image is None:
                return

            viewport = (
                self.coordinate_transform.widget_viewport(
                    self.width(),
                    self.height(),
                )
            )

            if viewport is None:
                return

            x, y, width, height = viewport

            # OccupancyGrid 是离散栅格地图。
            # 禁用平滑插值，避免放大后变糊。
            painter.setRenderHint(
                QPainter.SmoothPixmapTransform,
                False,
            )

            target_rect = QRectF(
                x,
                y,
                width,
                height,
            )

            painter.drawImage(
                target_rect,
                self.map_image,
            )

            self._draw_path(
                painter,
                self.global_path,
                QColor(0, 120, 255),
                2.0,
            )

            self._draw_path(
                painter,
                self.local_path,
                QColor(0, 200, 100),
                2.0,
            )

            self._draw_path(
                painter,
                self.actual_path,
                QColor(255, 70, 70),
                2.0,
            )

            self._draw_robot(painter)

            self._draw_goal(painter)

        finally:
            painter.end()

    def _draw_path(
            self,
            painter,
            path,
            color,
            width=2.0,
    ):
        if len(path) < 2:
            return

        painter.setRenderHint(
            QPainter.Antialiasing,
            True,
        )

        pen = QPen(
            color,
            width,
        )

        pen.setCosmetic(True)

        painter.setPen(pen)

        first = self.coordinate_transform.world_to_widget(
            path[0][0],
            path[0][1],
            self.width(),
            self.height(),
        )

        if first is None:
            return

        painter_path = QPainterPath()

        painter_path.moveTo(
            QPointF(
                first[0],
                first[1],
            )
        )

        for x, y in path[1:]:
            point = (
                self.coordinate_transform.world_to_widget(
                    x,
                    y,
                    self.width(),
                    self.height(),
                )
            )

            if point is None:
                continue

            painter_path.lineTo(
                QPointF(
                    point[0],
                    point[1],
                )
            )

        painter.drawPath(
            painter_path
        )

    def _draw_robot(
            self,
            painter,
    ):
        if self.robot_pose is None:
            return

        x, y, yaw = self.robot_pose

        center = self.coordinate_transform.world_to_widget(
            x,
            y,
            self.width(),
            self.height(),
        )

        if center is None:
            return

        center_x, center_y = center

        painter.setRenderHint(
            QPainter.Antialiasing,
            True,
        )

        # 屏幕坐标中 y 轴向下，
        # 因此 sin(yaw) 需要取反。
        forward_x = math.cos(yaw)
        forward_y = -math.sin(yaw)

        left_x = -forward_y
        left_y = forward_x

        tip_distance = 9.0
        rear_distance = 6.0
        half_width = 6.0

        tip = QPointF(
            center_x + forward_x * tip_distance,
            center_y + forward_y * tip_distance,
        )

        rear_center_x = (
                center_x - forward_x * rear_distance
        )
        rear_center_y = (
                center_y - forward_y * rear_distance
        )

        rear_left = QPointF(
            rear_center_x + left_x * half_width,
            rear_center_y + left_y * half_width,
        )

        rear_right = QPointF(
            rear_center_x - left_x * half_width,
            rear_center_y - left_y * half_width,
        )

        robot_path = QPainterPath()

        robot_path.moveTo(tip)
        robot_path.lineTo(rear_left)
        robot_path.lineTo(rear_right)
        robot_path.closeSubpath()

        painter.setPen(
            QPen(
                QColor(120, 0, 180),
                1.5,
            )
        )

        painter.setBrush(
            QColor(180, 0, 255)
        )

        painter.drawPath(robot_path)

    def _draw_goal(
            self,
            painter,
    ):
        if self.goal_pose is None:
            return

        x, y = self.goal_pose

        center = self.coordinate_transform.world_to_widget(
            x,
            y,
            self.width(),
            self.height(),
        )

        if center is None:
            return

        center_x, center_y = center

        painter.setRenderHint(
            QPainter.Antialiasing,
            True,
        )

        pen = QPen(
            QColor(255, 165, 0),
            2.5,
        )

        pen.setCosmetic(True)

        painter.setPen(pen)
        painter.setBrush(
            QColor(255, 165, 0)
        )

        # 中心小圆点
        painter.drawEllipse(
            QPointF(center_x, center_y),
            3.0,
            3.0,
        )

        # 十字标记
        painter.drawLine(
            QPointF(center_x - 8.0, center_y),
            QPointF(center_x + 8.0, center_y),
        )

        painter.drawLine(
            QPointF(center_x, center_y - 8.0),
            QPointF(center_x, center_y + 8.0),
        )