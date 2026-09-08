import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    input_topic = LaunchConfiguration('input_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/cmd_vel',
            description='Input velocity topic for cmd_vel_adapter',
        ),
        Node(
            package='landerpi_base_driver',
            executable='cmd_vel_adapter_node',
            name='cmd_vel_adapter',
            output='screen',
            parameters=[
                config_file,
                {'input_topic': input_topic},
            ],
        ),
    ])
