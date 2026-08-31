import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor, QImage
from PyQt5.QtWidgets import QApplication, QMainWindow
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from landerpi_sandbox_display.coordinate_transform import CoordinateTransform
from landerpi_sandbox_display.map_widget import MapWidget

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

        # 实际轨迹发布器
        self.actual_path_publisher = self.create_publisher(
            Path,
            '/actual_path',
            10,
        )

        # 保存实际轨迹
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'

        self.last_actual_x = None
        self.last_actual_y = None

        # 机器人移动超过 3cm 才记录一个轨迹点
        self.actual_path_min_distance = 0.03

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

        # 更新界面中的机器人
        if self.robot_pose_update_callback is not None:
            self.robot_pose_update_callback(msg)

        # 记录实际轨迹
        self.update_actual_path(msg)

    def update_actual_path(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        should_record = False

        if self.last_actual_x is None:
            should_record = True
        else:
            dx = x - self.last_actual_x
            dy = y - self.last_actual_y

            distance = math.hypot(dx, dy)

            if distance >= self.actual_path_min_distance:
                should_record = True

        if not should_record:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose

        self.actual_path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self.actual_path.poses.append(pose)

        self.actual_path_publisher.publish(
            self.actual_path
        )

        self.last_actual_x = x
        self.last_actual_y = y

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

        self.goal_publish_callback = None

        # 坐标转换器必须先创建
        self.coordinate_transform = CoordinateTransform()

        # 新的高清地图控件
        self.map_widget = MapWidget(
            coordinate_transform=self.coordinate_transform,
        )

        self.map_widget.setMinimumSize(
            600,
            500,
        )

        # 保留地图点击 → Goal 的功能
        self.map_widget.clicked.connect(
            self.handle_map_click
        )

        # MapWidget 正式成为主窗口中央控件
        self.setCentralWidget(
            self.map_widget
        )

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
            QImage.Format_Grayscale8,
        )

        for y in range(height):
            for x in range(width):
                value = msg.data[y * width + x]

                if value < 0:
                    gray = 128
                elif value >= 50:
                    gray = 0
                else:
                    gray = 255

                image.setPixelColor(
                    x,
                    y,
                    QColor(gray, gray, gray),
                )

        # ROS 地图原点左下，Qt 图像原点左上
        image = image.mirrored(
            False,
            True,
        )

        self.map_image = image

        # 交给新的高清 MapWidget
        self.map_widget.set_map_image(
            image
        )




    def set_goal_publish_callback(self, callback):
        self.goal_publish_callback = callback


    def update_robot_pose(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y

        q = msg.pose.orientation

        siny_cosp = 2.0 * (
                q.w * q.z
                + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
                q.y * q.y
                + q.z * q.z
        )

        self.robot_yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        self.map_widget.set_robot_pose(
            (
                self.robot_x,
                self.robot_y,
                self.robot_yaw,
            )
        )

    def update_global_plan(self, msg):
        self.global_plan_points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.map_widget.set_global_path(
            self.global_plan_points
        )


    def update_local_plan(self, msg):
        self.local_plan_points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.map_widget.set_local_path(
            self.local_plan_points
        )


    def update_actual_path(self, msg):
        self.actual_path_points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in msg.poses
        ]

        self.map_widget.set_actual_path(
            self.actual_path_points
        )


    def handle_map_click(
            self,
            widget_x,
            widget_y,
    ):
        world = (
            self.coordinate_transform.widget_to_world(
                widget_x=widget_x,
                widget_y=widget_y,
                widget_width=self.map_widget.width(),
                widget_height=self.map_widget.height(),
            )
        )

        if world is None:
            return

        x, y = world

        self.goal_x = x
        self.goal_y = y

        self.map_widget.set_goal_pose(
            (x, y)
        )

        print(
            f'Map clicked: '
            f'widget=({widget_x:.1f}, {widget_y:.1f}), '
            f'world=({x:.3f}, {y:.3f})'
        )

        if self.goal_publish_callback is not None:
            self.goal_publish_callback(
                x,
                y,
            )


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