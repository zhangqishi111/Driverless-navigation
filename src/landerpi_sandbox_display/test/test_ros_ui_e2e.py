import math
import os
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
# Keep this graph isolated from a developer's running robot or display node.
os.environ.setdefault('ROS_DOMAIN_ID', '42')

import rclpy

from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from landerpi_msgs.msg import NavigationPointState, NavigationTaskState
from nav_msgs.msg import OccupancyGrid, Path
from PyQt5.QtWidgets import QApplication
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float32, String
from tf2_ros import TransformBroadcaster

from landerpi_sandbox_display.main_window import MainWindow
from landerpi_sandbox_display.sandbox_display_node import SandboxDisplayNode
from landerpi_sandbox_display.ui_state import DisplayState


app = QApplication.instance()

if app is None:
    app = QApplication([])


def _spin_until(executor, window, state, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        window.refresh_from_state(state)
        app.processEvents()
        if predicate():
            return
    raise AssertionError('timed out waiting for ROS/UI end-to-end condition')


def _map_message():
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width = 10
    message.info.height = 10
    message.info.resolution = 1.0
    message.info.origin.position.x = 0.0
    message.info.origin.position.y = 0.0
    message.data = [0] * 100
    return message


def _task_state(active):
    message = NavigationTaskState()
    message.task_id = 8
    message.active = active
    message.current_index = 2
    message.total_points = 3
    message.state = (
        NavigationTaskState.ACTIVE
        if active else NavigationTaskState.IDLE)
    message.elapsed_time_s = 24.0
    message.detail = 'task state supplied by the navigation backend'

    for index, point_state in enumerate((
            NavigationPointState.SUCCEEDED,
            NavigationPointState.NAVIGATING,
            NavigationPointState.PENDING), start=1):
        point = NavigationPointState()
        point.index = index
        point.target.position.x = float(index)
        point.target.position.y = float(index + 1)
        point.target.orientation.w = 1.0
        point.state = point_state
        point.has_arrival_error = index == 1
        point.arrival_error_m = 0.04 if index == 1 else 0.0
        point.elapsed_time_s = float(index)
        point.retry_count = 0
        point.detail = f'goal {index}'
        message.points.append(point)

    return message


def _path_message(points):
    message = Path()
    message.header.frame_id = 'map'
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def _click_world(window, x, y):
    point = window.coordinate_transform.world_to_widget(
        x,
        y,
        window.map_widget.width(),
        window.map_widget.height(),
    )
    assert point is not None
    window.handle_map_click(*point)


def test_ros_topics_and_ui_actions_round_trip_end_to_end():
    """Exercise the real ROS graph through the same UI wiring used by main()."""
    rclpy.init()
    state = DisplayState()
    display_node = SandboxDisplayNode(display_state=state)
    driver = Node('sandbox_display_e2e_driver')
    window = MainWindow()
    window.map_widget.resize(640, 480)
    window.set_goal_publish_callback(display_node.publish_goal_pose)
    window.set_task_publish_callback(display_node.publish_navigation_task)
    window.set_task_cancel_callback(display_node.cancel_navigation_task)

    map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    map_publisher = driver.create_publisher(OccupancyGrid, '/map', map_qos)
    robot_pose_publisher = driver.create_publisher(
        PoseStamped, '/robot_pose', 10)
    plan_publisher = driver.create_publisher(Path, '/plan', 10)
    local_plan_publisher = driver.create_publisher(Path, '/local_plan', 10)
    position_error_publisher = driver.create_publisher(
        Float32, '/position_error', 10)
    status_publisher = driver.create_publisher(
        String, '/navigation_status', 10)
    arrival_publisher = driver.create_publisher(Bool, '/arrival_status', 10)
    task_state_publisher = driver.create_publisher(
        NavigationTaskState, '/navigation_task/state', map_qos)
    transform_broadcaster = TransformBroadcaster(driver)

    goal_messages = []
    task_messages = []
    cancel_messages = []
    actual_path_messages = []
    driver.create_subscription(
        PoseStamped, '/goal_pose', goal_messages.append, 10)
    driver.create_subscription(
        PoseArray, '/navigation_task/goals', task_messages.append, 10)
    driver.create_subscription(
        Empty, '/navigation_task/cancel', cancel_messages.append, 10)
    driver.create_subscription(
        Path, '/actual_path', actual_path_messages.append, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(display_node)
    executor.add_node(driver)

    try:
        _spin_until(
            executor, window, state,
            lambda: (
                map_publisher.get_subscription_count() == 1 and
                robot_pose_publisher.get_subscription_count() == 1 and
                plan_publisher.get_subscription_count() == 1 and
                local_plan_publisher.get_subscription_count() == 1 and
                position_error_publisher.get_subscription_count() == 1 and
                status_publisher.get_subscription_count() == 1 and
                arrival_publisher.get_subscription_count() == 1 and
                task_state_publisher.get_subscription_count() == 1 and
                display_node.goal_pose_publisher.get_subscription_count() == 1 and
                display_node.navigation_task_publisher.get_subscription_count() == 1 and
                display_node.navigation_task_cancel_publisher.get_subscription_count() == 1 and
                display_node.actual_path_publisher.get_subscription_count() == 1),
        )

        map_publisher.publish(_map_message())
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 2.5
        pose.pose.position.y = 3.5
        pose.pose.orientation.z = math.sin(math.pi / 4.0)
        pose.pose.orientation.w = math.cos(math.pi / 4.0)
        robot_pose_publisher.publish(pose)
        plan_publisher.publish(_path_message([(2.5, 3.5), (4.0, 4.0)]))
        local_plan_publisher.publish(_path_message([(2.5, 3.5), (3.0, 3.8)]))
        position_error = Float32()
        position_error.data = 0.125
        position_error_publisher.publish(position_error)
        status = String()
        status.data = 'navigating'
        status_publisher.publish(status)
        arrived = Bool()
        arrived.data = False
        arrival_publisher.publish(arrived)

        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = 2.5
        transform.transform.translation.y = 3.5
        transform.transform.rotation.z = math.sin(math.pi / 4.0)
        transform.transform.rotation.w = math.cos(math.pi / 4.0)
        transform_broadcaster.sendTransform(transform)

        _spin_until(
            executor, window, state,
            lambda: (
                window.map_widget.map_image is not None and
                window.robot_x_value.text() == '2.500 m' and
                window.navigation_status_value.text() == 'Navigating' and
                window.arrival_status_value.text() == 'Not Arrived' and
                window.position_error_value.text() == '0.125 m' and
                window.global_path_value.text() == '2 points' and
                window.local_path_value.text() == '2 points'),
        )

        _spin_until(
            executor, window, state,
            lambda: (
                window.actual_path_value.text() == '1 points' and
                len(actual_path_messages) == 1),
            timeout=8.0,
        )
        assert len(actual_path_messages[0].poses) == 1

        _click_world(window, 4.0, 6.0)
        _spin_until(
            executor, window, state,
            lambda: len(goal_messages) == 1,
        )
        assert goal_messages[0].header.frame_id == 'map'
        assert goal_messages[0].pose.position.x == 4.0
        assert goal_messages[0].pose.position.y == 6.0

        window.multi_goal_mode_button.setChecked(True)
        _click_world(window, 1.0, 1.0)
        _click_world(window, 2.0, 2.0)
        window.submit_task_button.click()
        _spin_until(
            executor, window, state,
            lambda: len(task_messages) == 1,
        )
        assert task_messages[0].header.frame_id == 'map'
        assert [
            (pose.position.x, pose.position.y)
            for pose in task_messages[0].poses
        ] == [(1.0, 1.0), (2.0, 2.0)]

        task_state_publisher.publish(_task_state(active=True))
        _spin_until(
            executor, window, state,
            lambda: (
                window.task_table.item(1, 4).text() == 'Navigating' and
                window.cancel_task_button.isEnabled()),
        )
        assert not window.submit_task_button.isEnabled()
        assert not window.undo_task_button.isEnabled()

        window.cancel_task_button.click()
        _spin_until(
            executor, window, state,
            lambda: len(cancel_messages) == 1,
        )
        assert not window.cancel_task_button.isEnabled()
    finally:
        executor.remove_node(driver)
        executor.remove_node(display_node)
        executor.shutdown()
        driver.destroy_node()
        display_node.destroy_node()
        window.close()
        rclpy.shutdown()
