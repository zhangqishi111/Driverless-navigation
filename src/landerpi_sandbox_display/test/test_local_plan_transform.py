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

from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path

from landerpi_sandbox_display.sandbox_display_node import (
    SandboxDisplayNode,
)


class FakeTfBuffer:
    def __init__(self):
        self.requested_time = None

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        time,
    ):
        self.requested_time = time

        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'

        return transform


class FakeDisplayState:
    def update_local_plan(self, plan):
        self.plan = plan


class FakeLogger:
    def warning(self, *_args, **_kwargs):
        pass


def test_local_plan_uses_message_timestamp_for_tf_lookup():
    msg = Path()
    msg.header.frame_id = 'odom'

    msg.header.stamp.sec = 821
    msg.header.stamp.nanosec = 613000000

    tf_buffer = FakeTfBuffer()

    fake_node = SimpleNamespace(
        tf_buffer=tf_buffer,
        display_state=FakeDisplayState(),
        get_logger=lambda: FakeLogger(),
    )

    SandboxDisplayNode.local_plan_callback(
        fake_node,
        msg,
    )

    assert (
        tf_buffer.requested_time.nanoseconds
        == 821_613_000_000
    )