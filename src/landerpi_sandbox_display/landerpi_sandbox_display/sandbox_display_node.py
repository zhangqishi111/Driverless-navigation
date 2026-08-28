import math
import sys
from landerpi_sandbox_display.coordinate_transform import CoordinateTransform


import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid, Path
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtCore import QTimer, Qt, QPointF, pyqtSignal
from PyQt5.QtGui import (
    QImage,
    QPixmap,
    QColor,
    QPainter,
    QPen,
    QBrush,
    QPolygonF,
)
from PyQt5.QtGui import QImage, QPixmap, QColor, QPainter, QPen, QBrush
from geometry_msgs.msg import PoseStamped

class MapLabel(QLabel):
    clicked = pyqtSignal(float, float)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(
                float(event.x()),
                float(event.y())
            )

        super().mousePressEvent(event)

class SandboxDisplayNode(Node):
    def __init__(
            self,
            map_update_callback=None,
            robot_pose_update_callback=None,
            plan_update_callback=None,
            local_plan_update_callback=None,
            actual_path_update_callback=None,
    ):
        super().__init__('sandbox_display_node')

        self.map_update_callback = map_update_callback
        self.robot_pose_update_callback = robot_pose_update_callback
        self.plan_update_callback = plan_update_callback
        self.local_plan_update_callback = local_plan_update_callback
        self.actual_path_update_callback = actual_path_update_callback

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 订阅地图
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )

        # 订阅机器人实时位姿
        self.robot_pose_subscription = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            10,
        )

        self.plan_subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10,
        )

        self.local_plan_subscription = self.create_subscription(
            Path,
            '/local_plan',
            self.local_plan_callback,
            10,
        )

        self.actual_path_subscription = self.create_subscription(
            Path,
            '/actual_path',
            self.actual_path_callback,
            10,
        )

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10,
        )

        self._last_map_signature = None

        self.get_logger().info(
            'Sandbox display node started. Waiting for /map and /robot_pose...'
        )

    def publish_goal_pose(self, x, y):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        # V0.8 暂时默认目标朝向 yaw = 0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pose_publisher.publish(msg)

        self.get_logger().info(
            f'Published /goal_pose: x={x:.3f}, y={y:.3f}, yaw=0.000'
        )

    def robot_pose_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'/robot_pose frame is "{msg.header.frame_id}", expected "map".'
            )
            return

        if self.robot_pose_update_callback is not None:
            self.robot_pose_update_callback(msg)

    def plan_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'/plan frame is "{msg.header.frame_id}", expected "map".'
            )
            return

        if self.plan_update_callback is not None:
            self.plan_update_callback(msg)

    def local_plan_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'/local_plan frame is "{msg.header.frame_id}", expected "map".'
            )
            return

        if self.local_plan_update_callback is not None:
            self.local_plan_update_callback(msg)

    def actual_path_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'/actual_path frame is "{msg.header.frame_id}", expected "map".'
            )
            return

        if self.actual_path_update_callback is not None:
            self.actual_path_update_callback(msg)

    def map_callback(self, msg):
        info = msg.info

        map_signature = (
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
        )

        if map_signature != self._last_map_signature:
            self._last_map_signature = map_signature

            self.get_logger().info(
                f'Received /map: '
                f'frame={msg.header.frame_id}, '
                f'size={info.width}x{info.height}, '
                f'resolution={info.resolution:.3f} m/cell, '
                f'origin=({info.origin.position.x:.3f}, '
                f'{info.origin.position.y:.3f})'
            )

        if self.map_update_callback is not None:
            self.map_update_callback(msg)


class SandboxWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('LanderPi Sandbox Display')
        self.resize(900, 600)

        self.map_label = MapLabel('Waiting for /map...')
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.clicked.connect(
            self.handle_map_click
        )

        self.goal_publish_callback = None
        self.coordinate_transform = CoordinateTransform()

        self.setCentralWidget(self.map_label)

        self.map_image = None

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.global_plan_points = []
        self.local_plan_points = []
        self.actual_path_points = []

        self.goal_x = None
        self.goal_y = None



    def update_map(self, msg):
        width = msg.info.width
        height = msg.info.height

        self.map_width = width
        self.map_height = height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.coordinate_transform.update_map_info(
            width=width,
            height=height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )



        image = QImage(
            width,
            height,
            QImage.Format_Grayscale8
        )

        for y in range(height):
            for x in range(width):
                value = msg.data[y * width + x]

                if value < 0:
                    # ROS OccupancyGrid: -1 = 未知区域
                    gray = 128
                elif value >= 50:
                    # 障碍物
                    gray = 0
                else:
                    # 可通行区域
                    gray = 255

                image.setPixelColor(
                    x,
                    y,
                    QColor(gray, gray, gray)
                )

        # ROS地图原点在左下，而Qt图像原点在左上
        image = image.mirrored(False, True)

        self.map_image = image

        self.refresh_map()

    def refresh_map(self):
        if self.map_image is None:
            return

        display_image = self.map_image.convertToFormat(
            QImage.Format_RGB32
        )

        painter = QPainter(display_image)

        painter.setRenderHint(
            QPainter.Antialiasing,
            True
        )

        # -------------------------
        # 1. 绘制三条路径
        # -------------------------

        # 全局规划：蓝色
        self.draw_path(
            painter,
            self.global_plan_points,
            QColor(0, 80, 255),
            1,
        )

        # 局部规划：绿色
        self.draw_path(
            painter,
            self.local_plan_points,
            QColor(0, 180, 0),
            1,
        )

        # 实际轨迹：红色
        self.draw_path(
            painter,
            self.actual_path_points,
            QColor(220, 0, 0),
            1,
        )

        # -------------------------
        # 2. 绘制目标点 /goal_pose
        # -------------------------
        if self.goal_x is not None and self.goal_y is not None:
            pixel = self.coordinate_transform.world_to_pixel(
                self.goal_x,
                self.goal_y
            )

            if pixel is not None:
                px, py = pixel

                if (
                        0 <= px < self.map_width
                        and 0 <= py < self.map_height
                ):
                    goal_color = QColor(255, 165, 0)

                    painter.setPen(
                        QPen(goal_color, 2)
                    )
                    painter.setBrush(
                        QBrush(Qt.NoBrush)
                    )

                    radius = 2

                    # 圆圈
                    painter.drawEllipse(
                        QPointF(px, py),
                        radius,
                        radius
                    )

                    # 横线
                    painter.drawLine(
                        QPointF(px - radius - 3, py),
                        QPointF(px + radius + 3, py)
                    )

                    # 竖线
                    painter.drawLine(
                        QPointF(px, py - radius - 3),
                        QPointF(px, py + radius + 3)
                    )

        # -------------------------
        # 3. 绘制机器人
        # -------------------------
        if (
                self.robot_x is not None
                and self.robot_y is not None
        ):
            pixel = self.coordinate_transform.world_to_pixel(
                self.robot_x,
                self.robot_y
            )

            if pixel is not None:
                px, py = pixel

                robot_color = QColor(170, 0, 255)

                painter.setPen(
                    QPen(robot_color, 1)
                )
                painter.setBrush(
                    QBrush(robot_color)
                )

                yaw = (
                    self.robot_yaw
                    if self.robot_yaw is not None
                    else 0.0
                )

                length = 6.0
                half_width = 3.5

                forward_x = math.cos(yaw)
                forward_y = -math.sin(yaw)

                left_x = math.cos(
                    yaw + math.pi / 2.0
                )
                left_y = -math.sin(
                    yaw + math.pi / 2.0
                )

                tip = QPointF(
                    px + length * forward_x,
                    py + length * forward_y,
                )

                rear_left = QPointF(
                    px - length * 0.6 * forward_x
                    + half_width * left_x,
                    py - length * 0.6 * forward_y
                    + half_width * left_y,
                )

                rear_right = QPointF(
                    px - length * 0.6 * forward_x
                    - half_width * left_x,
                    py - length * 0.6 * forward_y
                    - half_width * left_y,
                )

                robot_triangle = QPolygonF([
                    tip,
                    rear_left,
                    rear_right,
                ])

                painter.drawPolygon(robot_triangle)

        painter.end()

        pixmap = QPixmap.fromImage(display_image)

        scaled_pixmap = pixmap.scaled(
            self.map_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )

        self.map_label.setPixmap(scaled_pixmap)


    def set_goal_publish_callback(self, callback):
        self.goal_publish_callback = callback

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.refresh_map()

    def update_robot_pose(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y

        q = msg.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.refresh_map()

    def update_global_plan(self, msg):
        self.global_plan_points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.refresh_map()

    def update_local_plan(self, msg):
        self.local_plan_points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.refresh_map()

    def update_actual_path(self, msg):
        self.actual_path_points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.refresh_map()

    def handle_map_click(self, label_x, label_y):
        pixel = self.coordinate_transform.label_to_map_pixel(
            label_x=label_x,
            label_y=label_y,
            label_width=self.map_label.width(),
            label_height=self.map_label.height(),
        )

        if pixel is None:
            return

        px, py = pixel

        world = self.coordinate_transform.pixel_to_world(
            px,
            py
        )

        if world is None:
            return

        x, y = world

        self.goal_x = x
        self.goal_y = y

        self.refresh_map()

        print(
            f'Map clicked: '
            f'pixel=({px:.1f}, {py:.1f}), '
            f'world=({x:.3f}, {y:.3f})'
        )

        if self.goal_publish_callback is not None:
            self.goal_publish_callback(x, y)

    def draw_path(self, painter, points, color, width=1):
        if len(points) < 2:
            return

        painter.setPen(QPen(color, width))

        previous_point = None

        for x, y in points:
            pixel = self.coordinate_transform.world_to_pixel(x, y)

            if pixel is None:
                continue

            px, py = pixel

            if not (
                    0 <= px < self.map_width
                    and 0 <= py < self.map_height
            ):
                previous_point = None
                continue

            current_point = QPointF(px, py)

            if previous_point is not None:
                painter.drawLine(
                    previous_point,
                    current_point
                )

            previous_point = current_point



def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    window = SandboxWindow()

    node = SandboxDisplayNode(
        map_update_callback=window.update_map,
        robot_pose_update_callback=window.update_robot_pose,
        plan_update_callback=window.update_global_plan,
        local_plan_update_callback=window.update_local_plan,
        actual_path_update_callback=window.update_actual_path,
    )

    window.set_goal_publish_callback(
        node.publish_goal_pose
    )

    window.show()

    timer = QTimer()
    timer.timeout.connect(
        lambda: rclpy.spin_once(node, timeout_sec=0.0)
    )
    timer.start(20)

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()