import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class SimInterfaceNode(Node):
    """Normalize Gazebo topics and publish the standard odom TF."""

    def __init__(self):
        super().__init__('landerpi_sim_interface')
        self._odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self._scan_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, '/sim/odom', self._on_odom, 20)
        self.create_subscription(
            LaserScan, '/sim/scan', self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            'Simulation interface ready: /sim/odom -> /odom, '
            '/sim/scan -> /scan')

    def _on_odom(self, msg: Odometry) -> None:
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self._odom_pub.publish(msg)

        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _on_scan(self, msg: LaserScan) -> None:
        msg.header.frame_id = 'laser_link'
        self._scan_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

