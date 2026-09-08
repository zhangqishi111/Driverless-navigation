#!/usr/bin/env python3


def extract_amcl_variances(covariance):
    x_variance = covariance[0]
    y_variance = covariance[7]
    yaw_variance = covariance[35]

    position_variance = max(x_variance, y_variance)

    return position_variance, yaw_variance


class LocalizationGate:
    """Pure localization-safety state machine."""

    def __init__(
        self,
        position_variance_threshold,
        yaw_variance_threshold,
        pose_timeout,
        required_good_updates,
    ):
        self.position_variance_threshold = position_variance_threshold
        self.yaw_variance_threshold = yaw_variance_threshold
        self.pose_timeout = pose_timeout
        self.required_good_updates = required_good_updates

        self._ready = False
        self._good_updates = 0
        self._last_pose_stamp = None

    def update_pose(
        self,
        position_variance,
        yaw_variance,
        stamp_sec,
    ):
        self._last_pose_stamp = stamp_sec

        localization_good = (
            position_variance <= self.position_variance_threshold
            and yaw_variance <= self.yaw_variance_threshold
        )

        if localization_good:
            self._good_updates += 1
            if self._good_updates >= self.required_good_updates:
                self._ready = True
        else:
            self._good_updates = 0
            self._ready = False

    def filter_velocity(
        self,
        linear_x,
        linear_y,
        angular_z,
        now_sec,
    ):
        if not self.is_ready(now_sec):
            return (0.0, 0.0, 0.0)

        return (linear_x, linear_y, angular_z)

    def is_ready(self, now_sec):
        if self._last_pose_stamp is None:
            return False

        if now_sec - self._last_pose_stamp > self.pose_timeout:
            self._ready = False
            self._good_updates = 0

        return self._ready


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


class LocalizationSafetyGuardNode(Node):
    def __init__(self):
        super().__init__("localization_safety_guard")

        self.declare_parameter("position_variance_threshold", 0.25)
        self.declare_parameter("yaw_variance_threshold", 0.25)
        self.declare_parameter("pose_timeout", 1.0)
        self.declare_parameter("required_good_updates", 3)

        self.gate = LocalizationGate(
            position_variance_threshold=self.get_parameter(
                "position_variance_threshold"
            ).value,
            yaw_variance_threshold=self.get_parameter(
                "yaw_variance_threshold"
            ).value,
            pose_timeout=self.get_parameter(
                "pose_timeout"
            ).value,
            required_good_updates=self.get_parameter(
                "required_good_updates"
            ).value,
        )

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_callback,
            10,
        )

        self.cmd_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_guard_in",
            self._cmd_vel_callback,
            10,
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

    def _amcl_pose_callback(self, msg):
        pass

    def _cmd_vel_callback(self, msg):
        pass
