import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver_share = get_package_share_directory(
        'landerpi_base_driver'
    )
    config_file = os.path.join(
        driver_share,
        'config',
        'real_driver.yaml',
    )

    return LaunchDescription([
        Node(
            package='landerpi_base_driver',
            executable='cmd_vel_adapter_node',
            name='cmd_vel_adapter',
            output='screen',
            parameters=[config_file],
        ),
    ])
