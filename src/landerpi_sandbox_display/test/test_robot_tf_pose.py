from geometry_msgs.msg import TransformStamped
from types import SimpleNamespace

from landerpi_sandbox_display.sandbox_display_node import (
    SandboxDisplayNode,
    downsample_poses,
    transform_to_pose_stamped,
)

def test_transform_to_pose_stamped_uses_map_frame():
    transform = TransformStamped()

    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'

    transform.transform.translation.x = 1.2
    transform.transform.translation.y = -0.8
    transform.transform.translation.z = 0.1

    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.3
    transform.transform.rotation.w = 0.954

    pose = transform_to_pose_stamped(
        transform
    )

    assert pose.header.frame_id == 'map'

    assert pose.pose.position.x == 1.2
    assert pose.pose.position.y == -0.8
    assert pose.pose.position.z == 0.1

    assert pose.pose.orientation.z == 0.3
    assert pose.pose.orientation.w == 0.954

class FakeDisplayState:
    def __init__(self):
        self.robot_pose = None

    def update_robot_pose(self, pose):
        self.robot_pose = pose


class FakeTfBuffer:
    def __init__(self, transform):
        self.transform = transform

    def lookup_transform(self, *_args, **_kwargs):
        return self.transform


def test_tf_pose_also_updates_actual_path():
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.rotation.w = 1.0

    state = FakeDisplayState()
    actual_path_calls = []

    fake_node = SimpleNamespace(
        tf_buffer=FakeTfBuffer(transform),
        display_state=state,
        update_actual_path=actual_path_calls.append,
    )

    SandboxDisplayNode.update_robot_pose_from_tf(
        fake_node
    )

    assert state.robot_pose is not None
    assert len(actual_path_calls) == 1

    assert (
        actual_path_calls[0].pose.position.x
        == 1.0
    )
    assert (
        actual_path_calls[0].pose.position.y
        == 2.0
    )

def test_downsample_poses_limits_size_and_preserves_ends():
    poses = list(range(2000))

    result = downsample_poses(
        poses,
        max_points=800,
    )

    assert len(result) == 800
    assert result[0] == 0
    assert result[-1] == 1999