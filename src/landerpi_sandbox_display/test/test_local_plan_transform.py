import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

from landerpi_sandbox_display.sandbox_display_node import (
    transform_path,
)


def test_transform_path_from_odom_to_map():
    path = Path()
    path.header.frame_id = 'odom'

    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.pose.position.x = 1.0
    pose.pose.position.y = 0.0
    pose.pose.orientation.w = 1.0

    path.poses = [pose]

    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'

    # map <- odom:
    # 平移 (2, 3)
    transform.transform.translation.x = 2.0
    transform.transform.translation.y = 3.0

    # 再旋转 90°
    transform.transform.rotation.z = math.sin(
        math.pi / 4.0
    )
    transform.transform.rotation.w = math.cos(
        math.pi / 4.0
    )

    result = transform_path(
        path,
        transform,
    )

    assert result.header.frame_id == 'map'
    assert len(result.poses) == 1

    transformed_pose = result.poses[0]

    assert transformed_pose.header.frame_id == 'map'

    # (1, 0) 旋转 90° -> (0, 1)
    # 再平移 (2, 3) -> (2, 4)
    assert abs(
        transformed_pose.pose.position.x - 2.0
    ) < 1e-6

    assert abs(
        transformed_pose.pose.position.y - 4.0
    ) < 1e-6