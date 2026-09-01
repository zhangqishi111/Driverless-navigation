import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelAdapterNode(Node):
    """Forward standard /cmd_vel commands to the real LanderPi controller."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_adapter')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/controller/cmd_vel')
        self.declare_parameter('max_linear_speed', 0.20)
        self.declare_parameter('max_angular_speed', 0.50)
        self.declare_parameter('command_timeout', 0.50)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._max_linear = float(
            self.get_parameter('max_linear_speed').value
        )
        self._max_angular = float(
            self.get_parameter('max_angular_speed').value
        )
        self._timeout = float(
            self.get_parameter('command_timeout').value
        )

        self._publisher = self.create_publisher(
            Twist,
            output_topic,
            10,
        )
        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._on_cmd_vel,
            10,
        )

        self._last_command_time = None
        self._command_active = False
        self._timer = self.create_timer(0.05, self._check_timeout)

        self.get_logger().info(
            f'Command adapter ready: {input_topic} -> {output_topic}'
        )
        self.get_logger().info(
            f'Limits: linear={self._max_linear:.2f} m/s, '
            f'angular={self._max_angular:.2f} rad/s, '
            f'timeout={self._timeout:.2f} s'
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _on_cmd_vel(self, msg: Twist) -> None:
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )

        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                'Rejected cmd_vel containing NaN or infinity'
            )
            self._publish_stop()
            return

        output = Twist()
        output.linear.x = self._clamp(
            msg.linear.x,
            self._max_linear,
        )
        output.linear.y = self._clamp(
            msg.linear.y,
            self._max_linear,
        )
        output.linear.z = 0.0
        output.angular.x = 0.0
        output.angular.y = 0.0
        output.angular.z = self._clamp(
            msg.angular.z,
            self._max_angular,
        )

        self._publisher.publish(output)
        self._last_command_time = self.get_clock().now()
        self._command_active = True

    def _check_timeout(self) -> None:
        if not self._command_active or self._last_command_time is None:
            return

        elapsed = (
            self.get_clock().now() - self._last_command_time
        ).nanoseconds / 1e9

        if elapsed >= self._timeout:
            self.get_logger().warning(
                'cmd_vel timeout: publishing stop command'
            )
            self._publish_stop()

    def _publish_stop(self) -> None:
        self._publisher.publish(Twist())
        self._command_active = False
        self._last_command_time = None

    def stop(self) -> None:
        self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelAdapterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
