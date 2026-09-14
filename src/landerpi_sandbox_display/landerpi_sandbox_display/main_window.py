import math

from landerpi_msgs.msg import NavigationPointState, NavigationTaskState
from PyQt5.QtGui import QColor, QImage
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QButtonGroup,
    QFrame,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from landerpi_sandbox_display.coordinate_transform import (
    CoordinateTransform,
)
from landerpi_sandbox_display.map_widget import MapWidget, TaskMarker
from landerpi_sandbox_display.task_draft import TaskDraft


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
        self.task_draft = TaskDraft(max_goals=3)
        self._task_active = False
        self._cancel_pending = False
        self._updating_task_table = False

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

        self._create_task_builder(navigation_layout)

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

        self._single_goal_widgets = [
            value.parentWidget()
            for value in (
                self.goal_x_value,
                self.goal_y_value,
                self.navigation_status_value,
                self.arrival_status_value,
                self.task_time_value,
                self.position_error_value,
                self.global_path_value,
                self.local_path_value,
                self.actual_path_value,
            )
        ]
        self._task_builder_widgets = [
            self.task_summary_value.parentWidget(),
            self.task_table,
            self.task_buttons_widget,
        ]
        self._apply_mode_visibility()

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
        self._refresh_task_draft_view()
        self._update_task_controls()

    def _create_task_builder(self, navigation_layout):
        mode_row = QWidget()
        mode_layout = QHBoxLayout(mode_row)
        mode_layout.setContentsMargins(0, 0, 0, 0)
        self.single_goal_mode_button = QRadioButton('Single goal')
        self.multi_goal_mode_button = QRadioButton('Multi-goal')
        self.mode_button_group = QButtonGroup(self)
        self.mode_button_group.addButton(self.single_goal_mode_button)
        self.mode_button_group.addButton(self.multi_goal_mode_button)
        self.single_goal_mode_button.setChecked(True)
        mode_layout.addWidget(self.single_goal_mode_button)
        mode_layout.addWidget(self.multi_goal_mode_button)
        navigation_layout.addWidget(mode_row)

        self.task_summary_value = self._add_value_row(
            navigation_layout,
            'Multi-goal Task',
        )

        self.task_table = QTableWidget(3, 8)
        self.task_table.setHorizontalHeaderLabels([
            '#', 'X', 'Y', 'Yaw°', 'State', 'Error', 'Time', 'Retry',
        ])
        self.task_table.verticalHeader().setVisible(False)
        self.task_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.task_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.task_table.setEditTriggers(
            QAbstractItemView.DoubleClicked |
            QAbstractItemView.SelectedClicked)
        self.task_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Fixed)
        for column, width in enumerate((30, 70, 70, 52, 112, 82, 62, 50)):
            self.task_table.setColumnWidth(column, width)
        self.task_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.task_table.setMinimumHeight(128)
        navigation_layout.addWidget(self.task_table)

        buttons = QWidget()
        self.task_buttons_widget = buttons
        buttons_layout = QHBoxLayout(buttons)
        buttons_layout.setContentsMargins(0, 0, 0, 0)
        self.undo_task_button = QPushButton('Undo last')
        self.clear_task_button = QPushButton('Clear')
        self.submit_task_button = QPushButton('Submit task')
        self.cancel_task_button = QPushButton('Cancel task')
        for button in (
                self.undo_task_button,
                self.clear_task_button,
                self.submit_task_button,
                self.cancel_task_button):
            buttons_layout.addWidget(button)
        navigation_layout.addWidget(buttons)

        self.single_goal_mode_button.toggled.connect(self._on_mode_changed)
        self.multi_goal_mode_button.toggled.connect(self._on_mode_changed)
        self.undo_task_button.clicked.connect(self.undo_task_goal)
        self.clear_task_button.clicked.connect(self.clear_task_goals)
        self.submit_task_button.clicked.connect(self.submit_navigation_task)
        self.cancel_task_button.clicked.connect(self.cancel_navigation_task)
        self.task_table.itemChanged.connect(self._on_task_table_item_changed)

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
            '● ROS display'
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
        side_panel.setMinimumWidth(580)

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
    # Multi-goal task
    # =============================

    def _on_mode_changed(self, _checked):
        self._apply_mode_visibility()
        if self.multi_goal_mode_button.isChecked():
            self.map_widget.set_goal_pose(None)
            self._refresh_task_draft_view()
        else:
            self.map_widget.set_task_markers([])
        self._update_task_controls()

    def _apply_mode_visibility(self):
        multi_mode = self.multi_goal_mode_button.isChecked()
        for widget in getattr(self, '_single_goal_widgets', []):
            widget.setVisible(not multi_mode)
        for widget in getattr(self, '_task_builder_widgets', []):
            widget.setVisible(multi_mode)

    def _update_task_controls(self):
        multi_mode = self.multi_goal_mode_button.isChecked()
        editable = multi_mode and not self._task_active
        self.single_goal_mode_button.setEnabled(not self._task_active)
        self.multi_goal_mode_button.setEnabled(not self._task_active)
        self.undo_task_button.setEnabled(editable)
        self.clear_task_button.setEnabled(editable)
        self.submit_task_button.setEnabled(
            editable and bool(self.task_draft.goals()))
        self.cancel_task_button.setEnabled(
            self._task_active and not self._cancel_pending)
        if self._task_active:
            self.task_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        else:
            self.task_table.setEditTriggers(
                QAbstractItemView.DoubleClicked |
                QAbstractItemView.SelectedClicked)

    def _set_table_item(self, row, column, text, editable=False):
        item = self.task_table.item(row, column)
        if item is None:
            item = QTableWidgetItem()
            self.task_table.setItem(row, column, item)
        item.setText(str(text))
        flags = item.flags()
        if editable:
            item.setFlags(flags | Qt.ItemIsEditable)
        else:
            item.setFlags(flags & ~Qt.ItemIsEditable)

    def _clear_task_table(self):
        for row in range(self.task_table.rowCount()):
            for column in range(self.task_table.columnCount()):
                self._set_table_item(row, column, '', editable=False)

    def _refresh_task_draft_view(self):
        goals = self.task_draft.goals()
        self._updating_task_table = True
        try:
            self._clear_task_table()
            for row, goal in enumerate(goals):
                values = (
                    row + 1,
                    f'{goal.x:.3f}',
                    f'{goal.y:.3f}',
                    f'{math.degrees(goal.yaw):.1f}',
                    'Draft',
                    '--',
                    '--',
                    '0',
                )
                for column, value in enumerate(values):
                    self._set_table_item(
                        row,
                        column,
                        value,
                        editable=(column == 3),
                    )
        finally:
            self._updating_task_table = False

        if self.multi_goal_mode_button.isChecked():
            self.map_widget.set_task_markers([
                TaskMarker(
                    index=index,
                    x=goal.x,
                    y=goal.y,
                    yaw=goal.yaw,
                    state='pending',
                )
                for index, goal in enumerate(goals, start=1)
            ])
        self.task_summary_value.setText(
            f'Draft · {len(goals)}/3 points' if goals else '--')
        self._update_task_controls()

    def _on_task_table_item_changed(self, item):
        if self._updating_task_table or self._task_active:
            return
        if not self.multi_goal_mode_button.isChecked() or item.column() != 3:
            return
        if item.row() >= len(self.task_draft.goals()):
            return
        try:
            yaw_degrees = float(item.text())
            self.task_draft.set_yaw(item.row(), math.radians(yaw_degrees))
        except (TypeError, ValueError):
            pass
        self._refresh_task_draft_view()

    def undo_task_goal(self):
        if self._task_active:
            return
        self.task_draft.undo()
        self._refresh_task_draft_view()

    def clear_task_goals(self):
        if self._task_active:
            return
        self.task_draft.clear()
        self._refresh_task_draft_view()

    def submit_navigation_task(self):
        goals = self.task_draft.goals()
        if self._task_active or not goals or self.task_publish_callback is None:
            return
        if self.task_publish_callback(goals) is False:
            self.task_summary_value.setText('Not submitted · task receiver unavailable')
            self.task_summary_value.setToolTip(
                'No task receiver is connected or the draft is invalid. '
                'Start navigation_task_queue, then submit again. Draft retained.')
            return
        self.task_summary_value.setText('Submitted · waiting for task state')
        self.task_summary_value.setToolTip(
            'Waiting for /navigation_task/state from the task queue. '
            'If this persists, check that display and navigation use matching versions.')

    def cancel_navigation_task(self):
        if (not self._task_active or self._cancel_pending or
                self.task_cancel_callback is None):
            return
        self._cancel_pending = True
        self._update_task_controls()
        self.task_cancel_callback()

    @staticmethod
    def _yaw_from_pose(pose):
        quaternion = pose.orientation
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z +
                   quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y +
                         quaternion.z * quaternion.z),
        )

    def update_navigation_task_state(self, message):
        self._task_active = bool(message.active)
        self._cancel_pending = False
        should_show_task = (
            message.active or
            (message.state != NavigationTaskState.IDLE and bool(message.points)))
        if should_show_task and not self.multi_goal_mode_button.isChecked():
            self.multi_goal_mode_button.setChecked(True)

        task_states = {
            NavigationTaskState.IDLE: 'Idle',
            NavigationTaskState.ACTIVE: 'Active',
            NavigationTaskState.SUCCEEDED: 'Succeeded',
            NavigationTaskState.FAILED: 'Failed',
            NavigationTaskState.CANCELLED: 'Cancelled',
            NavigationTaskState.REJECTED: 'Rejected',
        }
        task_state = task_states.get(message.state, 'Unknown')
        summary_text = (
            f'Task {message.task_id} · {task_state} · '
            f'{message.current_index}/{message.total_points} · '
            f'{message.elapsed_time_s:.1f} s')
        self.task_summary_value.setText(summary_text)
        self.task_summary_value.setToolTip(message.detail)

        if (message.state == NavigationTaskState.REJECTED and
                not message.points and self.task_draft.goals()):
            self._refresh_task_draft_view()
            self.task_summary_value.setText(summary_text)
            self.task_summary_value.setToolTip(message.detail)
            return

        point_states = {
            NavigationPointState.PENDING: 'Pending',
            NavigationPointState.PLANNING: 'Planning',
            NavigationPointState.NAVIGATING: 'Navigating',
            NavigationPointState.WAITING_FOR_STOP: 'Waiting For Stop',
            NavigationPointState.SUCCEEDED: 'Succeeded',
            NavigationPointState.FAILED: 'Failed',
            NavigationPointState.CANCELLED: 'Cancelled',
            NavigationPointState.NOT_EXECUTED: 'Not Executed',
        }

        markers = []
        self._updating_task_table = True
        try:
            self._clear_task_table()
            for row, point in enumerate(message.points[:3]):
                yaw = self._yaw_from_pose(point.target)
                state_text = point_states.get(point.state, 'Unknown')
                error_text = (
                    f'{point.arrival_error_m:.3f} m'
                    if point.has_arrival_error else '--')
                values = (
                    point.index,
                    f'{point.target.position.x:.3f}',
                    f'{point.target.position.y:.3f}',
                    f'{math.degrees(yaw):.1f}',
                    state_text,
                    error_text,
                    f'{point.elapsed_time_s:.1f} s',
                    point.retry_count,
                )
                for column, value in enumerate(values):
                    self._set_table_item(
                        row,
                        column,
                        value,
                        editable=(column == 3 and not self._task_active),
                    )
                self.task_table.item(row, 4).setToolTip(point.detail)
                markers.append(TaskMarker(
                    index=point.index,
                    x=point.target.position.x,
                    y=point.target.position.y,
                    yaw=yaw,
                    state=state_text.lower().replace(' ', '_'),
                ))
        finally:
            self._updating_task_table = False

        self.map_widget.set_task_markers(markers)
        self._update_task_controls()

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

            'navigation_task_state':
                self.update_navigation_task_state,
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

            single_goal_fields = {
                'navigation_status',
                'arrival_status',
                'task_time',
                'position_error',
            }
            if (self.multi_goal_mode_button.isChecked()
                    and name in single_goal_fields):
                self._state_revisions[name] = revision
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
        if self._task_active:
            return
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

        if self.multi_goal_mode_button.isChecked():
            robot_yaw = None
            if self.map_widget.robot_pose is not None:
                robot_yaw = self.map_widget.robot_pose[2]
            if self.task_draft.add(x, y, robot_yaw=robot_yaw):
                self._refresh_task_draft_view()
            return

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
