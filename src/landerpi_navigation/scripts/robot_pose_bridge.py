#!/usr/bin/env python3
"""Expose AMCL localization through the project's standard /robot_pose topic."""

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node


class RobotPoseBridge(Node):
    def __init__(self):
        super().__init__('robot_pose_bridge')
        self._publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            10,
        )

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self._publisher.publish(pose)


def main() -> None:
    rclpy.init()
    node = RobotPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
