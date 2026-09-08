from pathlib import Path
import importlib.util


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "localization_safety_guard.py"
)


def load_guard_module():
    assert SCRIPT_PATH.exists(), (
        "localization_safety_guard.py does not exist yet"
    )

    spec = importlib.util.spec_from_file_location(
        "localization_safety_guard",
        SCRIPT_PATH,
    )

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_gate_starts_unsafe():
    module = load_guard_module()

    assert hasattr(module, "LocalizationGate"), (
        "LocalizationGate has not been implemented yet"
    )

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    assert gate.is_ready(now_sec=0.0) is False


def test_gate_requires_three_consecutive_good_updates():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.0,
    )
    assert gate.is_ready(now_sec=0.0) is False

    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.1,
    )
    assert gate.is_ready(now_sec=0.1) is False

    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.2,
    )
    assert gate.is_ready(now_sec=0.2) is True


def test_bad_update_immediately_locks_gate():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    for stamp in (0.0, 0.1, 0.2):
        gate.update_pose(
            position_variance=0.10,
            yaw_variance=0.10,
            stamp_sec=stamp,
        )

    assert gate.is_ready(now_sec=0.2) is True

    gate.update_pose(
        position_variance=0.80,
        yaw_variance=0.10,
        stamp_sec=0.3,
    )

    assert gate.is_ready(now_sec=0.3) is False


def test_pose_timeout_locks_gate():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    for stamp in (0.0, 0.1, 0.2):
        gate.update_pose(
            position_variance=0.10,
            yaw_variance=0.10,
            stamp_sec=stamp,
        )

    assert gate.is_ready(now_sec=0.5) is True

    assert gate.is_ready(now_sec=1.3) is False


def test_timeout_requires_reconvergence_before_unlock():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    for stamp in (0.0, 0.1, 0.2):
        gate.update_pose(
            position_variance=0.10,
            yaw_variance=0.10,
            stamp_sec=stamp,
        )

    assert gate.is_ready(now_sec=0.2) is True

    assert gate.is_ready(now_sec=1.3) is False

    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=1.4,
    )
    assert gate.is_ready(now_sec=1.4) is False

    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=1.5,
    )
    assert gate.is_ready(now_sec=1.5) is False

    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=1.6,
    )
    assert gate.is_ready(now_sec=1.6) is True


def test_velocity_is_zero_when_localization_not_ready():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    output = gate.filter_velocity(
        linear_x=0.10,
        linear_y=0.05,
        angular_z=0.30,
        now_sec=0.0,
    )

    assert output == (0.0, 0.0, 0.0)


def test_velocity_passes_through_when_localization_ready():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    for stamp in (0.0, 0.1, 0.2):
        gate.update_pose(
            position_variance=0.10,
            yaw_variance=0.10,
            stamp_sec=stamp,
        )

    output = gate.filter_velocity(
        linear_x=0.10,
        linear_y=0.0,
        angular_z=0.30,
        now_sec=0.3,
    )

    assert output == (0.10, 0.0, 0.30)


def test_lateral_velocity_passes_through_when_localization_ready():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    for stamp in (0.0, 0.1, 0.2):
        gate.update_pose(
            position_variance=0.10,
            yaw_variance=0.10,
            stamp_sec=stamp,
        )

    output = gate.filter_velocity(
        linear_x=0.0,
        linear_y=0.08,
        angular_z=0.0,
        now_sec=0.3,
    )

    assert output == (0.0, 0.08, 0.0)


def test_velocity_stays_zero_while_localization_is_lost():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.25,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    for stamp in (0.0, 0.1, 0.2):
        gate.update_pose(
            position_variance=0.10,
            yaw_variance=0.10,
            stamp_sec=stamp,
        )

    assert gate.is_ready(now_sec=0.2) is True

    gate.update_pose(
        position_variance=0.80,
        yaw_variance=0.80,
        stamp_sec=0.3,
    )

    for now_sec in (0.3, 0.4, 0.5):
        output = gate.filter_velocity(
            linear_x=0.10,
            linear_y=0.08,
            angular_z=0.30,
            now_sec=now_sec,
        )

        assert output == (0.0, 0.0, 0.0)


def test_extract_amcl_variances():
    module = load_guard_module()

    covariance = [0.0] * 36
    covariance[0] = 0.10
    covariance[7] = 0.18
    covariance[35] = 0.12

    position_variance, yaw_variance = module.extract_amcl_variances(
        covariance
    )

    assert position_variance == 0.18
    assert yaw_variance == 0.12


def test_ros_node_has_safe_default_parameters():
    import rclpy

    module = load_guard_module()

    assert hasattr(module, "LocalizationSafetyGuardNode")

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        assert node.get_parameter(
            "position_variance_threshold"
        ).value == 0.25

        assert node.get_parameter(
            "yaw_variance_threshold"
        ).value == 0.25

        assert node.get_parameter(
            "pose_timeout"
        ).value == 1.0

        assert node.get_parameter(
            "required_good_updates"
        ).value == 3

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def test_ros_node_exposes_expected_topics():
    import rclpy

    module = load_guard_module()

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        amcl_subscriptions = node.get_subscriptions_info_by_topic(
            "/amcl_pose"
        )
        cmd_subscriptions = node.get_subscriptions_info_by_topic(
            "/cmd_vel_guard_in"
        )
        cmd_publishers = node.get_publishers_info_by_topic(
            "/cmd_vel"
        )

        assert len(amcl_subscriptions) >= 1
        assert len(cmd_subscriptions) >= 1
        assert len(cmd_publishers) >= 1

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
