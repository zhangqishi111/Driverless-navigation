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


def test_amcl_callback_updates_localization_gate():
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    module = load_guard_module()

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        msg = PoseWithCovarianceStamped()
        msg.pose.covariance[0] = 0.10
        msg.pose.covariance[7] = 0.12
        msg.pose.covariance[35] = 0.10

        node._amcl_pose_callback(msg)
        node._amcl_pose_callback(msg)
        node._amcl_pose_callback(msg)

        now_sec = (
            node.get_clock().now().nanoseconds / 1e9
        )

        assert node.gate.is_ready(now_sec)

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def test_cmd_vel_callback_publishes_zero_when_localization_not_ready():
    import rclpy
    from geometry_msgs.msg import Twist

    module = load_guard_module()

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        fake_publisher = FakePublisher()
        node.cmd_publisher = fake_publisher

        msg = Twist()
        msg.linear.x = 0.10
        msg.linear.y = 0.05
        msg.angular.z = 0.20

        node._cmd_vel_callback(msg)

        assert len(fake_publisher.messages) == 1

        output = fake_publisher.messages[0]

        assert output.linear.x == 0.0
        assert output.linear.y == 0.0
        assert output.angular.z == 0.0

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def test_cmd_vel_callback_publishes_zero_when_localization_not_ready():
    import rclpy
    from geometry_msgs.msg import Twist

    module = load_guard_module()

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        fake_publisher = FakePublisher()
        node.cmd_publisher = fake_publisher

        msg = Twist()
        msg.linear.x = 0.10
        msg.linear.y = 0.05
        msg.angular.z = 0.20

        node._cmd_vel_callback(msg)

        assert len(fake_publisher.messages) == 1

        output = fake_publisher.messages[0]

        assert output.linear.x == 0.0
        assert output.linear.y == 0.0
        assert output.angular.z == 0.0

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def test_cmd_vel_callback_passes_velocity_when_localization_ready():
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

    module = load_guard_module()

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        fake_publisher = FakePublisher()
        node.cmd_publisher = fake_publisher

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.covariance[0] = 0.10
        pose_msg.pose.covariance[7] = 0.12
        pose_msg.pose.covariance[35] = 0.10

        node._amcl_pose_callback(pose_msg)
        node._amcl_pose_callback(pose_msg)
        node._amcl_pose_callback(pose_msg)

        cmd_msg = Twist()
        cmd_msg.linear.x = 0.10
        cmd_msg.linear.y = 0.05
        cmd_msg.angular.z = 0.20

        node._cmd_vel_callback(cmd_msg)

        assert len(fake_publisher.messages) == 1

        output = fake_publisher.messages[0]

        assert output.linear.x == 0.10
        assert output.linear.y == 0.05
        assert output.angular.z == 0.20

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def test_watchdog_publishes_zero_after_localization_timeout():
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    module = load_guard_module()

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        fake_publisher = FakePublisher()
        node.cmd_publisher = fake_publisher

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.covariance[0] = 0.10
        pose_msg.pose.covariance[7] = 0.12
        pose_msg.pose.covariance[35] = 0.10

        node._amcl_pose_callback(pose_msg)
        node._amcl_pose_callback(pose_msg)
        node._amcl_pose_callback(pose_msg)

        now_sec = node.get_clock().now().nanoseconds / 1e9
        assert node.gate.is_ready(now_sec)

        node.gate._last_pose_stamp = (
            now_sec - node.gate.pose_timeout - 0.1
        )

        assert hasattr(node, "watchdog_timer")

        node._watchdog_callback()

        assert len(fake_publisher.messages) == 1

        output = fake_publisher.messages[0]

        assert output.linear.x == 0.0
        assert output.linear.y == 0.0
        assert output.angular.z == 0.0

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def test_stationary_odom_keeps_gate_ready_when_amcl_pose_is_old():
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
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.1,
    )
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.2,
    )

    assert gate.is_ready(0.2)

    # Odom remains alive, but the robot stays at the same pose.
    gate.update_odometry(
        x=1.0,
        y=2.0,
        yaw=0.5,
        stamp_sec=0.2,
    )
    gate.update_odometry(
        x=1.0,
        y=2.0,
        yaw=0.5,
        stamp_sec=2.0,
    )

    # AMCL pose is now older than pose_timeout, but the robot did not move.
    assert gate.is_ready(2.0)


def test_small_odom_motion_keeps_gate_ready_when_amcl_pose_is_old():
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
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.1,
    )
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.2,
    )

    assert gate.is_ready(0.2)

    gate.update_odometry(
        x=0.0,
        y=0.0,
        yaw=0.0,
        stamp_sec=0.2,
    )

    # Small odom drift: below AMCL update thresholds
    # update_min_d = 0.25 m, update_min_a = 0.20 rad.
    gate.update_odometry(
        x=0.05,
        y=0.03,
        yaw=0.05,
        stamp_sec=2.0,
    )

    assert gate.is_ready(2.0)


def test_large_odom_motion_locks_gate_when_amcl_pose_is_old():
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
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.1,
    )
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.2,
    )

    assert gate.is_ready(0.2)

    gate.update_odometry(
        x=0.0,
        y=0.0,
        yaw=0.0,
        stamp_sec=0.2,
    )

    # Robot moved beyond AMCL's update_min_d = 0.25 m,
    # but no new AMCL pose arrived.
    gate.update_odometry(
        x=0.40,
        y=0.0,
        yaw=0.0,
        stamp_sec=2.0,
    )

    assert not gate.is_ready(2.0)


def test_new_amcl_pose_resets_odom_motion_reference():
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
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.1,
    )
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.2,
    )

    gate.update_odometry(
        x=0.0,
        y=0.0,
        yaw=0.0,
        stamp_sec=0.2,
    )

    # Robot moves beyond the old reference.
    gate.update_odometry(
        x=0.30,
        y=0.0,
        yaw=0.0,
        stamp_sec=0.5,
    )

    # AMCL successfully updates at the new location.
    gate.update_pose(
        position_variance=0.10,
        yaw_variance=0.10,
        stamp_sec=0.5,
    )

    # Only a tiny amount of motion occurs after that new AMCL update.
    gate.update_odometry(
        x=0.34,
        y=0.0,
        yaw=0.02,
        stamp_sec=2.0,
    )

    # The 0.30 m old motion must no longer count.
    assert gate.is_ready(2.0)


def test_ros_node_subscribes_to_odom():
    import rclpy

    module = load_guard_module()

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        subscriptions = node.get_subscriptions_info_by_topic(
            "/odom"
        )

        assert len(subscriptions) >= 1

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def test_extract_yaw_from_quaternion():
    import math

    module = load_guard_module()

    yaw = 0.5
    x = 0.0
    y = 0.0
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)

    result = module.extract_yaw_from_quaternion(x, y, z, w)

    assert abs(result - yaw) < 1e-6


def test_odom_callback_updates_gate():
    import math
    import rclpy
    from nav_msgs.msg import Odometry

    module = load_guard_module()

    rclpy.init()
    node = None

    try:
        node = module.LocalizationSafetyGuardNode()

        msg = Odometry()
        msg.pose.pose.position.x = 1.20
        msg.pose.pose.position.y = -0.40

        yaw = 0.5
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        node._odom_callback(msg)

        assert node.gate._latest_odom is not None

        x, y, actual_yaw, _ = node.gate._latest_odom

        assert abs(x - 1.20) < 1e-6
        assert abs(y - (-0.40)) < 1e-6
        assert abs(actual_yaw - 0.5) < 1e-6

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def test_amcl_update_margin_does_not_lock_at_point_three_meters():
    module = load_guard_module()

    gate = module.LocalizationGate(
        position_variance_threshold=0.30,
        yaw_variance_threshold=0.25,
        pose_timeout=1.0,
        required_good_updates=3,
    )

    gate.update_pose(0.10, 0.10, 0.0)
    gate.update_pose(0.10, 0.10, 0.1)
    gate.update_pose(0.10, 0.10, 0.2)

    gate.update_odometry(
        x=0.0,
        y=0.0,
        yaw=0.0,
        stamp_sec=0.2,
    )

    # AMCL update_min_d is 0.25 m.
    # Give AMCL some margin before declaring localization stale.
    gate.update_odometry(
        x=0.30,
        y=0.0,
        yaw=0.0,
        stamp_sec=2.0,
    )

    assert gate.is_ready(2.0)
