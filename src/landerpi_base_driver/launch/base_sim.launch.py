import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('landerpi_description')
    driver_share = get_package_share_directory('landerpi_base_driver')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(description_share, 'worlds', 'landerpi_test.sdf')
    models = os.path.join(description_share, 'models')
    xacro_file = os.path.join(
        description_share, 'urdf', 'landerpi_sim.urdf.xacro')
    bridge_config = os.path.join(
        driver_share, 'config', 'gz_bridge.yaml')

    existing_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    resource_path = models
    if existing_resource_path:
        resource_path += os.pathsep + existing_resource_path

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world}'}.items(),
    )

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
    ])

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path),
        gazebo,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
        Node(
            package='landerpi_base_driver',
            executable='sim_interface_node',
            name='landerpi_sim_interface',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])

