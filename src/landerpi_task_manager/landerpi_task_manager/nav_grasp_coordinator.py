#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node

from landerpi_msgs.msg import (
    NavigationPointState,
    NavigationTaskState,
)
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, Trigger


class NavGraspCoordinator(Node):

    def __init__(self):
        super().__init__('nav_grasp_coordinator')

        self.enabled = False
        self.task_id = None

        # P1：抓取点
        self.p1_pause_requested = False

        # P2：放置点
        self.p2_pause_requested = False

        self.grasp_started = False
        self.grasp_finished = False

        self.place_started = False
        self.place_finished = False

        self.pause_pub = self.create_publisher(
            Empty,
            '/navigation_task/pause',
            10,
        )

        self.resume_pub = self.create_publisher(
            Empty,
            '/navigation_task/resume',
            10,
        )

        self.grasp_client = self.create_client(
            Trigger,
            '/fire_agv/grasp_cube',
        )

        self.pick_success_client = self.create_client(
            Trigger,
            '/fire_agv/pick_success',
        )

        self.place_client = self.create_client(
            Trigger,
            '/fire_agv/place_cube',
        )

        self.place_success_client = self.create_client(
            Trigger,
            '/fire_agv/place_success',
        )

        self.create_service(
            SetBool,
            '/nav_grasp/enable',
            self.enable_callback,
        )

        self.create_subscription(
            NavigationTaskState,
            '/navigation_task/state',
            self.state_callback,
            10,
        )

        self.get_logger().info(
            'nav_grasp_coordinator ready; NORMAL mode'
        )

    def reset_state_flags(self):
        self.p1_pause_requested = False
        self.p2_pause_requested = False

        self.grasp_started = False
        self.grasp_finished = False

        self.place_started = False
        self.place_finished = False

    def enable_callback(self, request, response):
        self.enabled = request.data
        self.task_id = None

        self.reset_state_flags()

        mode = (
            'PICK_AND_PLACE'
            if self.enabled
            else 'NORMAL'
        )

        response.success = True
        response.message = mode

        self.get_logger().info(
            'mode -> ' + mode
        )

        return response

    def reset_task(self, task_id):
        self.task_id = task_id
        self.reset_state_flags()

        self.get_logger().info(
            f'new three-point pick-and-place task={task_id}'
        )

    def state_callback(self, msg):

        # NORMAL 模式完全不介入。
        if not self.enabled:
            return

        # 三点搬运任务：
        # P1 = grasp
        # P2 = place
        # P3 = return
        if msg.total_points != 3:
            return

        if self.task_id != msg.task_id:
            self.reset_task(msg.task_id)

        # --------------------------------------------------
        # P1 正在导航：
        # 提前通知 task_queue，在 P1 到达后暂停。
        # --------------------------------------------------
        if (
            msg.active
            and msg.current_index == 1
            and not self.p1_pause_requested
        ):
            self.pause_pub.publish(Empty())
            self.p1_pause_requested = True

            self.get_logger().info(
                'P1 active -> pause after P1 requested'
            )

        # --------------------------------------------------
        # P1 已成功：
        # 开始抓取。
        # --------------------------------------------------
        if (
            len(msg.points) >= 1
            and msg.points[0].state
            == NavigationPointState.SUCCEEDED
            and not self.grasp_started
        ):
            self.grasp_started = True

            threading.Thread(
                target=self.grasp_then_resume,
                daemon=True,
            ).start()

        # --------------------------------------------------
        # P2 正在导航：
        # 抓取已经真正完成后，再请求第二次暂停。
        # --------------------------------------------------
        if (
            self.grasp_finished
            and msg.active
            and msg.current_index == 2
            and not self.p2_pause_requested
        ):
            self.pause_pub.publish(Empty())
            self.p2_pause_requested = True

            self.get_logger().info(
                'P2 active -> pause after P2 requested'
            )

        # --------------------------------------------------
        # P2 已成功：
        # 执行放置，并等待真正完成后才恢复 P3。
        # --------------------------------------------------
        if (
            self.grasp_finished
            and len(msg.points) >= 2
            and msg.points[1].state
            == NavigationPointState.SUCCEEDED
            and not self.place_started
        ):
            self.place_started = True

            threading.Thread(
                target=self.place_then_resume,
                daemon=True,
            ).start()

        # --------------------------------------------------
        # P3 已成功：
        # 整个搬运任务才真正完成，切回 NORMAL。
        # --------------------------------------------------
        if (
            self.place_finished
            and len(msg.points) >= 3
            and msg.points[2].state
            == NavigationPointState.SUCCEEDED
            and not msg.active
        ):
            self.enabled = False

            self.get_logger().info(
                'P3 reached -> mission complete'
            )
            self.get_logger().info(
                'mode -> NORMAL'
            )

    def wait_service(self, client, name):
        while rclpy.ok():
            if client.wait_for_service(
                timeout_sec=1.0
            ):
                return True

            self.get_logger().warn(
                'waiting for ' + name
            )

        return False

    def grasp_then_resume(self):

        if not self.wait_service(
            self.grasp_client,
            '/fire_agv/grasp_cube',
        ):
            return

        self.get_logger().info(
            'P1 reached -> grasp cube'
        )

        future = self.grasp_client.call_async(
            Trigger.Request()
        )

        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'grasp service error: {error}'
            )
            return

        if not response.success:
            self.get_logger().error(
                'grasp rejected: '
                + response.message
            )
            return

        self.get_logger().info(
            'grasp started; waiting real success'
        )

        if not self.wait_service(
            self.pick_success_client,
            '/fire_agv/pick_success',
        ):
            return

        deadline = time.time() + 60.0

        while (
            rclpy.ok()
            and time.time() < deadline
        ):
            check = (
                self.pick_success_client.call_async(
                    Trigger.Request()
                )
            )

            while (
                rclpy.ok()
                and not check.done()
            ):
                time.sleep(0.1)

            try:
                result = check.result()
            except Exception:
                time.sleep(0.5)
                continue

            if result.success:
                self.grasp_finished = True

                self.get_logger().info(
                    'GRASP SUCCESS -> resume P2'
                )

                self.resume_pub.publish(
                    Empty()
                )
                return

            time.sleep(0.5)

        self.get_logger().error(
            'grasp timeout; P2 will NOT start'
        )

    def place_then_resume(self):

        if not self.wait_service(
            self.place_client,
            '/fire_agv/place_cube',
        ):
            return

        self.get_logger().info(
            'P2 reached -> place cube'
        )

        future = self.place_client.call_async(
            Trigger.Request()
        )

        while (
            rclpy.ok()
            and not future.done()
        ):
            time.sleep(0.1)

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'place service error: {error}'
            )
            return

        if not response.success:
            self.get_logger().error(
                'place rejected: '
                + response.message
            )
            return

        self.get_logger().info(
            'place started; waiting real success'
        )

        if not self.wait_service(
            self.place_success_client,
            '/fire_agv/place_success',
        ):
            return

        deadline = time.time() + 30.0

        while (
            rclpy.ok()
            and time.time() < deadline
        ):
            check = (
                self.place_success_client.call_async(
                    Trigger.Request()
                )
            )

            while (
                rclpy.ok()
                and not check.done()
            ):
                time.sleep(0.1)

            try:
                result = check.result()
            except Exception:
                time.sleep(0.5)
                continue

            if result.success:
                self.place_finished = True

                self.get_logger().info(
                    'PLACE SUCCESS -> resume P3'
                )

                self.resume_pub.publish(
                    Empty()
                )
                return

            time.sleep(0.5)

        self.get_logger().error(
            'place timeout; P3 will NOT start'
        )


def main(args=None):
    rclpy.init(args=args)

    node = NavGraspCoordinator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
