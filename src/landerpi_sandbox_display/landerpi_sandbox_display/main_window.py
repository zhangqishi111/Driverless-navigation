import math

from PyQt5.QtGui import QColor, QImage
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from landerpi_sandbox_display.coordinate_transform import (
    CoordinateTransform,
)
from landerpi_sandbox_display.map_widget import MapWidget


class MainWindow(QMainWindow):
    def __init__(
            self,
            parent=None,
            coordinate_transform=None,
    ):
        super().__init__(parent)

        self.setWindowTitle(
            'LanderPi Navigation Console'
        )

        self.resize(
            1200,
            760,
        )

        if coordinate_transform is None:
            coordinate_transform = (
                CoordinateTransform()
            )

        self.coordinate_transform = (
            coordinate_transform
        )

        self.goal_publish_callback = None
        self.task_publish_callback = None
        self.task_cancel_callback = None

        self.goal_x = None
        self.goal_y = None

        self._state_revisions = {}

        self.map_widget = MapWidget(
            coordinate_transform=(
                self.coordinate_transform
            ),
        )

        self.map_widget.clicked.connect(
            self.handle_map_click
        )

        # =========================
        # Robot State
        # =========================

        self.robot_state_panel = (
            self._create_panel(
                'Robot State'
            )
        )

        robot_layout = (
            self.robot_state_panel.layout()
        )

        self.robot_x_value = (
            self._add_value_row(
                robot_layout,
                'X',
            )
        )

        self.robot_y_value = (
            self._add_value_row(
                robot_layout,
                'Y',
            )
        )

        self.robot_yaw_value = (
            self._add_value_row(
                robot_layout,
                'Yaw',
            )
        )

        # =========================
        # Navigation Task
        # =========================

        self.navigation_task_panel = (
            self._create_panel(
                'Navigation Task'
            )
        )

        navigation_layout = (
            self.navigation_task_panel.layout()
        )

        self.goal_x_value = (
            self._add_value_row(
                navigation_layout,
                'Goal X',
            )
        )

        self.goal_y_value = (
            self._add_value_row(
                navigation_layout,
                'Goal Y',
            )
        )

        # 第四周：导航状态
        self.navigation_status_value = (
            self._add_value_row(
                navigation_layout,
                'Navigation Status',
            )
        )

        # 第四周：到达状态
        self.arrival_status_value = (
            self._add_value_row(
                navigation_layout,
                'Arrival Status',
            )
        )

        # 第四周：任务耗时
        self.task_time_value = (
            self._add_value_row(
                navigation_layout,
                'Task Time',
            )
        )

        # 第三周：位置误差
        self.position_error_value = (
            self._add_value_row(
                navigation_layout,
                'Position Error',
            )
        )

        self.global_path_value = (
            self._add_value_row(
                navigation_layout,
                'Global Path',
                '0 points',
            )
        )

        self.local_path_value = (
            self._add_value_row(
                navigation_layout,
                'Local Path',
                '0 points',
            )
        )

        self.actual_path_value = (
            self._add_value_row(
                navigation_layout,
                'Actual Path',
                '0 points',
            )
        )

        # =========================
        # Map Info
        # =========================

        self.map_info_panel = (
            self._create_panel(
                'Map Info'
            )
        )

        map_info_layout = (
            self.map_info_panel.layout()
        )

        self.map_frame_value = (
            self._add_value_row(
                map_info_layout,
                'Frame',
            )
        )

        self.map_resolution_value = (
            self._add_value_row(
                map_info_layout,
                'Resolution',
            )
        )

        self.map_size_value = (
            self._add_value_row(
                map_info_layout,
                'Map Size',
            )
        )

        self.map_origin_value = (
            self._add_value_row(
                map_info_layout,
                'Origin',
            )
        )

        # =========================
        # Event Log
        # =========================

        self.event_log_panel = (
            self._create_panel(
                'Event Log'
            )
        )

        self._build_ui()

    def _create_panel(self, title):
        panel = QFrame()

        panel.setFrameShape(
            QFrame.StyledPanel
        )

        layout = QVBoxLayout(panel)

        title_label = QLabel(title)

        title_label.setAlignment(
            Qt.AlignLeft
            | Qt.AlignVCenter
        )

        layout.addWidget(
            title_label
        )

        return panel

    def _build_ui(self):
        central_widget = QWidget()

        root_layout = QVBoxLayout(
            central_widget
        )

        root_layout.setContentsMargins(
            12,
            12,
            12,
            12,
        )

        root_layout.setSpacing(10)

        # =========================
        # Header
        # =========================

        header = QFrame()

        header_layout = QHBoxLayout(
            header
        )

        title_label = QLabel(
            'LanderPi Navigation Console'
        )

        connection_label = QLabel(
            '● Connected'
        )

        header_layout.addWidget(
            title_label
        )

        header_layout.addStretch()

        header_layout.addWidget(
            connection_label
        )

        root_layout.addWidget(
            header
        )

        # =========================
        # Main content
        # =========================

        content_widget = QWidget()

        content_layout = QHBoxLayout(
            content_widget
        )

        content_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        content_layout.setSpacing(10)

        # 左侧地图
        self.map_widget.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )

        content_layout.addWidget(
            self.map_widget,
            4,
        )

        # 右侧信息区
        side_panel = QWidget()

        side_layout = QVBoxLayout(
            side_panel
        )

        side_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        side_layout.setSpacing(10)

        side_layout.addWidget(
            self.robot_state_panel
        )

        side_layout.addWidget(
            self.navigation_task_panel
        )

        side_layout.addWidget(
            self.map_info_panel
        )

        content_layout.addWidget(
            side_panel,
            1,
        )

        root_layout.addWidget(
            content_widget,
            1,
        )

        # =========================
        # Event Log
        # =========================

        self.event_log_panel.setMinimumHeight(
            120
        )

        self.event_log_panel.setMaximumHeight(
            160
        )

        root_layout.addWidget(
            self.event_log_panel
        )

        self.setCentralWidget(
            central_widget
        )

    # =============================
    # Map
    # =============================

    def update_map(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        origin_x = (
            msg.info.origin.position.x
        )

        origin_y = (
            msg.info.origin.position.y
        )

        self.coordinate_transform.update_map_info(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )

        image = QImage(
            width,
            height,
            QImage.Format_Grayscale8,
        )

        for y in range(height):
            for x in range(width):
                value = msg.data[
                    y * width + x
                ]

                if value < 0:
                    gray = 128

                elif value >= 50:
                    gray = 0

                else:
                    gray = 255

                image.setPixelColor(
                    x,
                    y,
                    QColor(
                        gray,
                        gray,
                        gray,
                    ),
                )

        # ROS OccupancyGrid 左下角为原点；
        # Qt QImage 左上角为原点。
        image = image.mirrored(
            False,
            True,
        )

        self.map_widget.set_map_image(
            image
        )

        frame_id = msg.header.frame_id

        self.map_frame_value.setText(
            frame_id
            if frame_id
            else '--'
        )

        self.map_resolution_value.setText(
            f'{resolution:.3f} m/cell'
        )

        self.map_size_value.setText(
            f'{width} × {height}'
        )

        self.map_origin_value.setText(
            f'({origin_x:.3f}, '
            f'{origin_y:.3f})'
        )

    # =============================
    # Robot pose
    # =============================

    def update_robot_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        q = msg.pose.orientation

        siny_cosp = 2.0 * (
                q.w * q.z
                + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
                q.y * q.y
                + q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        self.map_widget.set_robot_pose(
            (
                x,
                y,
                yaw,
            )
        )

        self.robot_x_value.setText(
            f'{x:.3f} m'
        )

        self.robot_y_value.setText(
            f'{y:.3f} m'
        )

        self.robot_yaw_value.setText(
            f'{math.degrees(yaw):.1f}°'
        )

    # =============================
    # Paths
    # =============================

    def update_global_plan(self, msg):
        points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.map_widget.set_global_path(
            points
        )

        self.global_path_value.setText(
            f'{len(points)} points'
        )

    def update_local_plan(self, msg):
        points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.map_widget.set_local_path(
            points
        )

        self.local_path_value.setText(
            f'{len(points)} points'
        )

    def update_actual_path(self, msg):
        points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.map_widget.set_actual_path(
            points
        )

        self.actual_path_value.setText(
            f'{len(points)} points'
        )

    # =============================
    # Navigation status
    # =============================

    def update_navigation_status(
            self,
            status,
    ):
        status_text = str(status).strip()

        if not status_text:
            self.navigation_status_value.setText(
                '--'
            )
            return

        # planning -> Planning
        # navigating -> Navigating
        # goal_rejected -> Goal Rejected
        display_text = (
            status_text
            .replace('_', ' ')
            .title()
        )

        self.navigation_status_value.setText(
            display_text
        )

    def update_arrival_status(
            self,
            arrived,
    ):
        if bool(arrived):
            text = 'Arrived'
        else:
            text = 'Not Arrived'

        self.arrival_status_value.setText(
            text
        )

    def update_task_time(
            self,
            task_time,
    ):
        self.task_time_value.setText(
            f'{float(task_time):.1f} s'
        )

    def update_position_error(
            self,
            error,
    ):
        self.position_error_value.setText(
            f'{float(error):.3f} m'
        )

    # =============================
    # DisplayState -> UI
    # =============================

    def refresh_from_state(self, state):
        snapshot = state.snapshot()

        handlers = {
            'map':
                self.update_map,

            'robot_pose':
                self.update_robot_pose,

            'global_plan':
                self.update_global_plan,

            'local_plan':
                self.update_local_plan,

            'actual_path':
                self.update_actual_path,

            'navigation_status':
                self.update_navigation_status,

            'arrival_status':
                self.update_arrival_status,

            'task_time':
                self.update_task_time,

            'position_error':
                self.update_position_error,
        }

        for name, handler in handlers.items():
            revision = (
                snapshot.revisions.get(
                    name,
                    0,
                )
            )

            previous_revision = (
                self._state_revisions.get(
                    name,
                    0,
                )
            )

            if revision == previous_revision:
                continue

            value = (
                snapshot.values.get(name)
            )

            if value is not None:
                handler(value)

            self._state_revisions[name] = (
                revision
            )

    # =============================
    # Goal
    # =============================

    def set_goal_publish_callback(
            self,
            callback,
    ):
        self.goal_publish_callback = (
            callback
        )

    def set_task_publish_callback(self, callback):
        self.task_publish_callback = callback

    def set_task_cancel_callback(self, callback):
        self.task_cancel_callback = callback

    def handle_map_click(
            self,
            widget_x,
            widget_y,
    ):
        world = (
            self.coordinate_transform
            .widget_to_world(
                widget_x=widget_x,
                widget_y=widget_y,
                widget_width=(
                    self.map_widget.width()
                ),
                widget_height=(
                    self.map_widget.height()
                ),
            )
        )

        if world is None:
            return

        x, y = world

        self.goal_x = x
        self.goal_y = y

        self.map_widget.set_goal_pose(
            (
                x,
                y,
            )
        )

        self.goal_x_value.setText(
            f'{x:.3f} m'
        )

        self.goal_y_value.setText(
            f'{y:.3f} m'
        )

        if (
                self.goal_publish_callback
                is not None
        ):
            self.goal_publish_callback(
                x,
                y,
            )

    # =============================
    # UI helper
    # =============================

    def _add_value_row(
            self,
            layout,
            name,
            initial_value='--',
    ):
        row = QWidget()

        row_layout = QHBoxLayout(
            row
        )

        row_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        name_label = QLabel(name)

        value_label = QLabel(
            initial_value
        )

        value_label.setAlignment(
            Qt.AlignRight
            | Qt.AlignVCenter
        )

        row_layout.addWidget(
            name_label
        )

        row_layout.addStretch()

        row_layout.addWidget(
            value_label
        )

        layout.addWidget(
            row
        )

        return value_label
