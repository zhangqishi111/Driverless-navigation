import math
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

from landerpi_sandbox_display.sandbox_display_node import SandboxDisplayNode


def test_local_plan_callback_transforms_odom_to_map():
    started_here = False

    if not rclpy.ok():
        rclpy.init()
        started_here = True

    received = []
    node = SandboxDisplayNode(
        local_plan_update_callback=received.append
    )

    try:
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"

        transform.transform.translation.x = 2.0
        transform.transform.translation.y = 3.0

        transform.transform.rotation.z = math.sin(math.pi / 4.0)
        transform.transform.rotation.w = math.cos(math.pi / 4.0)

        node.tf_buffer = Mock()
        node.tf_buffer.lookup_transform.return_value = transform

        msg = Path()
        msg.header.frame_id = "odom"

        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        msg.poses = [pose]

        node.local_plan_callback(msg)

        assert len(received) == 1

        result = received[0]

        assert result.header.frame_id == "map"
        assert len(result.poses) == 1

        assert abs(result.poses[0].pose.position.x - 2.0) < 1e-6
        assert abs(result.poses[0].pose.position.y - 4.0) < 1e-6

        call_args = node.tf_buffer.lookup_transform.call_args[0]

        assert call_args[0] == "map"
        assert call_args[1] == "odom"

    finally:
        node.destroy_node()

        if started_here:
            rclpy.shutdown()
