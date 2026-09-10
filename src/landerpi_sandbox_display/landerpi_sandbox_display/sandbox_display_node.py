import math
import sys

import rclpy

from nav_msgs.msg import Path
from std_msgs.msg import Float32

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

from geometry_msgs.msg import PoseStamped
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

        # 保存实际轨迹
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'

        self.last_actual_x = None
        self.last_actual_y = None

        # 机器人移动超过 3cm 才记录一个轨迹点
        self.actual_path_min_distance = 0.03

        # UI 最多绘制 800 个 Actual Path 点
        self.actual_path_display_max_points = 800

        # 新轨迹点产生后，只标记 dirty，
        # 不在高频 TF callback 中直接发布/绘制整条轨迹。
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

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 20 Hz 从 TF 获取机器人实时位置
        self.robot_tf_timer = self.create_timer(
            0.05,
            self.update_robot_pose_from_tf,
        )

        # 订阅地图
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )

        # 订阅机器人定位位姿
        self.robot_pose_subscription = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            10,
        )

        # 订阅全局规划路径
        self.plan_subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10,
        )

        # 订阅局部规划路径
        self.local_plan_subscription = self.create_subscription(
            Path,
            '/local_plan',
            self.local_plan_callback,
            10,
        )

        # 订阅机器人当前位置与目标之间的 XY 位置误差
        self.position_error_subscription = self.create_subscription(
            Float32,
            '/position_error',
            self.position_error_callback,
            10,
        )

        # 发布导航目标
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10,
        )

        self._last_map_signature = None

        self.get_logger().info(
            'Sandbox display node started. '
            'Waiting for /map and /robot_pose...'
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
            f'Published /goal_pose: '
            f'x={x:.3f}, y={y:.3f}, yaw=0.000'
        )

    def position_error_callback(self, msg):
        """
        接收导航模块发布的 /position_error。

        /position_error 类型为 std_msgs/msg/Float32，
        表示机器人当前位置到最新有效目标点之间的
        map 坐标系 XY 欧氏距离，单位为米。
        """
        self.display_state.update_position_error(
            float(msg.data)
        )

    def robot_pose_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'/robot_pose frame is "{msg.header.frame_id}", '
                'expected "map".'
            )
            return

        # /robot_pose 保留作为 AMCL 统一接口。
        # 不再直接驱动实时机器人绘制。
        self.display_state.update_amcl_pose(
            msg
        )

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

        self.actual_path.poses.append(
            pose
        )

        self.last_actual_x = x
        self.last_actual_y = y

        # 只做标记。
        # 真正的 UI 刷新和 DDS 发布由低频 timer 完成。
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

        # 没有外部订阅者时，不做整条 Path 的 DDS 序列化。
        if (
                self.actual_path_publisher
                .get_subscription_count()
                > 0
        ):
            self.actual_path_publisher.publish(
                self.actual_path
            )

        self.actual_path_publish_dirty = False

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

        self.display_state.update_map(
            msg
        )

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


def transform_path(path_msg, transform):
    transformed_path = Path()

    transformed_path.header = path_msg.header
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


def transform_to_pose_stamped(transform):
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


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    window = create_main_window()

    display_state = DisplayState()

    node = SandboxDisplayNode(
        display_state=display_state,
    )

    window.set_goal_publish_callback(
        node.publish_goal_pose
    )

    # ROS 使用独立 Executor。
    # ROS callback 不再依赖 Qt 主线程处理。
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_thread = Thread(
        target=executor.spin,
        daemon=True,
    )

    ros_thread.start()

    # Qt 只负责每 50ms 读取一次“最新状态”。
    ui_timer = QTimer()

    ui_timer.timeout.connect(
        lambda: window.refresh_from_state(
            display_state
        )
    )

    ui_timer.start(50)

    window.show()

    exit_code = app.exec_()

    ui_timer.stop()

    executor.shutdown()

    ros_thread.join(
        timeout=2.0
    )

    node.destroy_node()

    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()