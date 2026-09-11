import math
import sys
import time

import rclpy

from nav_msgs.msg import Path
from std_msgs.msg import Bool, Empty, Float32, String

from threading import Thread

from rclpy.executors import SingleThreadedExecutor

from landerpi_sandbox_display.ui_state import DisplayState

from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)

from tf2_geometry_msgs import (
    do_transform_pose_stamped,
)

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from landerpi_msgs.msg import NavigationTaskState
from nav_msgs.msg import OccupancyGrid, Path

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from landerpi_sandbox_display.main_window import MainWindow
from landerpi_sandbox_display.coordinate_transform import CoordinateTransform
from landerpi_sandbox_display.map_widget import MapWidget
from landerpi_sandbox_display.task_draft import yaw_to_quaternion


class SandboxDisplayNode(Node):
    def __init__(
            self,
            map_update_callback=None,
            robot_pose_update_callback=None,
            plan_update_callback=None,
            local_plan_update_callback=None,
            actual_path_update_callback=None,
            display_state=None,
    ):
        super().__init__('sandbox_display_node')

        if display_state is None:
            display_state = DisplayState()

        self.display_state = display_state

        self.map_update_callback = map_update_callback
        self.robot_pose_update_callback = robot_pose_update_callback
        self.plan_update_callback = plan_update_callback
        self.local_plan_update_callback = local_plan_update_callback
        self.actual_path_update_callback = actual_path_update_callback

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        actual_path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.actual_path_publisher = self.create_publisher(
            Path,
            '/actual_path',
            actual_path_qos,
        )

        # =========================
        # Actual Path
        # =========================

        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'

        self.last_actual_x = None
        self.last_actual_y = None

        # 机器人移动超过 3cm 才记录一个轨迹点
        self.actual_path_min_distance = 0.03

        # UI 最多绘制 800 个 Actual Path 点
        self.actual_path_display_max_points = 800

        self.actual_path_ui_dirty = False
        self.actual_path_publish_dirty = False

        # 红线 UI 最高 5 Hz 刷新
        self.actual_path_ui_timer = self.create_timer(
            0.2,
            self.refresh_actual_path_display,
        )

        # /actual_path 对外最高 1 Hz 发布
        self.actual_path_publish_timer = self.create_timer(
            1.0,
            self.publish_actual_path_if_dirty,
        )

        # =========================
        # Task Time
        # =========================

        self.task_running = False
        self.task_start_time = None

        # 每 0.1 秒刷新一次任务时间
        self.task_time_timer = self.create_timer(
            0.1,
            self.refresh_task_time,
        )

        # =========================
        # QoS
        # =========================

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # =========================
        # TF
        # =========================

        # 20 Hz 从 TF 获取机器人实时位置
        self.robot_tf_timer = self.create_timer(
            0.05,
            self.update_robot_pose_from_tf,
        )

        # =========================
        # Subscriptions
        # =========================

        # 地图
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )

        # AMCL 机器人定位位姿
        self.robot_pose_subscription = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            10,
        )

        # 全局规划路径
        self.plan_subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10,
        )

        # 局部规划路径
        self.local_plan_subscription = self.create_subscription(
            Path,
            '/local_plan',
            self.local_plan_callback,
            10,
        )

        # 位置误差
        self.position_error_subscription = self.create_subscription(
            Float32,
            '/position_error',
            self.position_error_callback,
            10,
        )

        # 第四周：导航状态
        self.navigation_status_subscription = self.create_subscription(
            String,
            '/navigation_status',
            self.navigation_status_callback,
            10,
        )

        # 第四周：到达状态
        self.arrival_status_subscription = self.create_subscription(
            Bool,
            '/arrival_status',
            self.arrival_status_callback,
            10,
        )

        self.navigation_task_state_subscription = self.create_subscription(
            NavigationTaskState,
            '/navigation_task/state',
            self.navigation_task_state_callback,
            map_qos,
        )

        # =========================
        # Publishers
        # =========================

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10,
        )

        self.navigation_task_publisher = self.create_publisher(
            PoseArray,
            '/navigation_task/goals',
            10,
        )

        self.navigation_task_cancel_publisher = self.create_publisher(
            Empty,
            '/navigation_task/cancel',
            10,
        )

        self._last_map_signature = None

        self.get_logger().info(
            'Sandbox display node started. '
            'Waiting for /map and navigation data...'
        )

    # =========================================================
    # Goal
    # =========================================================

    def publish_goal_pose(self, x, y):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        # 暂时默认目标朝向 yaw = 0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pose_publisher.publish(msg)

        self.get_logger().info(
            f'Published /goal_pose: '
            f'x={x:.3f}, y={y:.3f}, yaw=0.000'
        )

    def publish_navigation_task(self, goals):
        goals = tuple(goals)
        if len(goals) < 1 or len(goals) > 3:
            return False
        if not all(
                math.isfinite(value)
                for goal in goals
                for value in (goal.x, goal.y, goal.yaw)):
            return False

        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        for goal in goals:
            pose = Pose()
            pose.position.x = float(goal.x)
            pose.position.y = float(goal.y)
            pose.position.z = 0.0
            quaternion = yaw_to_quaternion(float(goal.yaw))
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            message.poses.append(pose)

        self.navigation_task_publisher.publish(message)
        return True

    def cancel_navigation_task(self):
        self.navigation_task_cancel_publisher.publish(Empty())

    def navigation_task_state_callback(self, message):
        self.display_state.update_navigation_task_state(message)

    # =========================================================
    # Position Error
    # =========================================================

    def position_error_callback(self, msg):
        """
        接收 /position_error。

        std_msgs/msg/Float32
        当前机器人到目标点的 XY 欧氏距离，单位 m。
        """
        self.display_state.update_position_error(
            float(msg.data)
        )

    # =========================================================
    # Navigation Status
    # =========================================================

    def navigation_status_callback(self, msg):
        """
        接收 /navigation_status。

        planning / navigating 时启动任务计时。
        终止状态出现时停止计时。
        """
        status = str(msg.data).strip()

        self.display_state.update_navigation_status(
            status
        )

        normalized_status = status.lower()

        if normalized_status in (
                'planning',
                'navigating',
        ):
            if not getattr(
                    self,
                    'task_running',
                    False,
            ):
                self.task_running = True
                self.task_start_time = (
                    time.monotonic()
                )

        terminal_keywords = (
            'arrived',
            'failed',
            'failure',
            'rejected',
            'aborted',
            'canceled',
            'cancelled',
        )

        if any(
                keyword in normalized_status
                for keyword in terminal_keywords
        ):
            if getattr(
                    self,
                    'task_running',
                    False,
            ):
                self.update_task_time()
                self.task_running = False

    # =========================================================
    # Arrival Status
    # =========================================================

    def arrival_status_callback(self, msg):
        """
        接收 /arrival_status。

        True 表示成功到达目标，
        同时停止任务计时。
        """
        arrived = bool(msg.data)

        self.display_state.update_arrival_status(
            arrived
        )

        if (
                arrived
                and getattr(
            self,
            'task_running',
            False,
        )
                and getattr(
            self,
            'task_start_time',
            None,
        ) is not None
        ):
            current_time = time.monotonic()

            elapsed = max(
                0.0,
                current_time
                - self.task_start_time,
            )

            self.display_state.update_task_time(
                elapsed
            )

            self.task_running = False

    # =========================================================
    # Task Time
    # =========================================================

    def update_task_time(
            self,
            current_time=None,
    ):
        """
        根据任务开始时间计算已经运行的秒数。

        current_time 参数主要用于测试；
        正常运行时使用 time.monotonic()。
        """
        if not self.task_running:
            return

        if self.task_start_time is None:
            return

        if current_time is None:
            current_time = time.monotonic()

        elapsed = max(
            0.0,
            float(current_time)
            - float(self.task_start_time),
        )

        self.display_state.update_task_time(
            elapsed
        )

    def refresh_task_time(self):
        """
        ROS Timer 周期调用。

        导航任务运行期间持续刷新 Task Time。
        """
        if not self.task_running:
            return

        self.update_task_time()

    # =========================================================
    # Robot Pose
    # =========================================================

    def robot_pose_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'/robot_pose frame is "{msg.header.frame_id}", '
                'expected "map".'
            )
            return

        # /robot_pose 保留作为 AMCL 统一接口
        self.display_state.update_amcl_pose(
            msg
        )

    # =========================================================
    # Actual Path
    # =========================================================

    def update_actual_path(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        should_record = False

        if self.last_actual_x is None:
            should_record = True

        else:
            dx = x - self.last_actual_x
            dy = y - self.last_actual_y

            distance = math.hypot(
                dx,
                dy,
            )

            if (
                    distance
                    >= self.actual_path_min_distance
            ):
                should_record = True

        if not should_record:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose

        self.actual_path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self.actual_path.poses.append(
            pose
        )

        self.last_actual_x = x
        self.last_actual_y = y

        self.actual_path_ui_dirty = True
        self.actual_path_publish_dirty = True

    def refresh_actual_path_display(self):
        if not self.actual_path_ui_dirty:
            return

        display_path = Path()

        display_path.header = (
            self.actual_path.header
        )

        display_path.poses = downsample_poses(
            self.actual_path.poses,
            self.actual_path_display_max_points,
        )

        self.display_state.update_actual_path(
            display_path
        )

        self.actual_path_ui_dirty = False

    def publish_actual_path_if_dirty(self):
        if not self.actual_path_publish_dirty:
            return

        self.actual_path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # 没有外部订阅者时不进行整条 Path DDS 序列化
        if (
                self.actual_path_publisher
                .get_subscription_count()
                > 0
        ):
            self.actual_path_publisher.publish(
                self.actual_path
            )

        self.actual_path_publish_dirty = False

    # =========================================================
    # Global Plan
    # =========================================================

    def plan_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'/plan frame is "{msg.header.frame_id}", '
                'expected "map".'
            )
            return

        self.display_state.update_global_plan(
            msg
        )

    # =========================================================
    # Local Plan
    # =========================================================

    def local_plan_callback(self, msg):
        if msg.header.frame_id == 'map':
            map_path = msg

        elif msg.header.frame_id == 'odom':
            try:
                transform = (
                    self.tf_buffer.lookup_transform(
                        'map',
                        'odom',
                        rclpy.time.Time.from_msg(
                            msg.header.stamp
                        ),
                    )
                )

                map_path = transform_path(
                    msg,
                    transform,
                )

            except TransformException as exc:
                self.get_logger().warning(
                    'Unable to transform '
                    f'/local_plan from odom to map: {exc}'
                )
                return

        else:
            self.get_logger().warning(
                'Unsupported /local_plan frame: '
                f'"{msg.header.frame_id}"'
            )
            return

        self.display_state.update_local_plan(
            map_path
        )

    # =========================================================
    # Map
    # =========================================================

    def map_callback(self, msg):
        info = msg.info

        map_signature = (
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
        )

        if (
                map_signature
                != self._last_map_signature
        ):
            self._last_map_signature = (
                map_signature
            )

            self.get_logger().info(
                f'Received /map: '
                f'frame={msg.header.frame_id}, '
                f'size={info.width}x{info.height}, '
                f'resolution={info.resolution:.3f} m/cell, '
                f'origin=('
                f'{info.origin.position.x:.3f}, '
                f'{info.origin.position.y:.3f})'
            )

        self.display_state.update_map(
            msg
        )

    # =========================================================
    # TF Robot Pose
    # =========================================================

    def update_robot_pose_from_tf(self):
        try:
            transform = (
                self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(),
                )
            )

        except TransformException:
            return

        pose = transform_to_pose_stamped(
            transform
        )

        self.display_state.update_robot_pose(
            pose
        )

        self.update_actual_path(
            pose
        )


# =============================================================
# Helpers
# =============================================================

def downsample_poses(
        poses,
        max_points=800,
):
    if len(poses) <= max_points:
        return list(poses)

    if max_points < 2:
        return [poses[-1]]

    step = (
        (len(poses) - 1)
        / (max_points - 1)
    )

    indices = [
        round(i * step)
        for i in range(max_points)
    ]

    return [
        poses[index]
        for index in indices
    ]


def transform_path(
        path_msg,
        transform,
):
    transformed_path = Path()

    transformed_path.header = (
        path_msg.header
    )

    transformed_path.header.frame_id = (
        transform.header.frame_id
    )

    transformed_path.poses = [
        do_transform_pose_stamped(
            pose,
            transform,
        )
        for pose in path_msg.poses
    ]

    return transformed_path


def transform_to_pose_stamped(
        transform,
):
    pose = PoseStamped()

    pose.header = transform.header

    pose.pose.position.x = (
        transform.transform.translation.x
    )

    pose.pose.position.y = (
        transform.transform.translation.y
    )

    pose.pose.position.z = (
        transform.transform.translation.z
    )

    pose.pose.orientation = (
        transform.transform.rotation
    )

    return pose


def create_main_window():
    return MainWindow()


# =============================================================
# Main
# =============================================================

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(
        sys.argv
    )

    window = create_main_window()

    display_state = DisplayState()

    node = SandboxDisplayNode(
        display_state=display_state,
    )

    window.set_goal_publish_callback(
        node.publish_goal_pose
    )

    window.set_task_publish_callback(
        node.publish_navigation_task
    )

    window.set_task_cancel_callback(
        node.cancel_navigation_task
    )

    # ROS 使用独立 Executor
    executor = (
        SingleThreadedExecutor()
    )

    executor.add_node(
        node
    )

    ros_thread = Thread(
        target=executor.spin,
        daemon=True,
    )

    ros_thread.start()

    # Qt 每 50ms 获取最新状态
    ui_timer = QTimer()

    ui_timer.timeout.connect(
        lambda:
        window.refresh_from_state(
            display_state
        )
    )

    ui_timer.start(
        50
    )

    window.show()

    exit_code = app.exec_()

    ui_timer.stop()

    executor.shutdown()

    ros_thread.join(
        timeout=2.0
    )

    node.destroy_node()

    rclpy.shutdown()

    sys.exit(
        exit_code
    )


if __name__ == '__main__':
    main()
