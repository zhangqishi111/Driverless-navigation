#!/usr/bin/env python3
"""Launch map-based AMCL + Nav2 navigation for the LanderPi.

This launch file intentionally does not start SLAM Toolbox.  In navigation
mode AMCL is the sole publisher of map -> odom; B's mapping launch belongs to
the separate mapping mode.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    navigation_dir = get_package_share_directory('landerpi_navigation')
    bringup_dir = get_package_share_directory('landerpi_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(bringup_dir, 'maps', 'sim_map.yaml')
    default_params = os.path.join(navigation_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            # Separate processes make integration failures visible.
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    # Nav2's stock navigation launch feeds the velocity smoother's output to
    # ``/cmd_vel``.  The vendor odom/controller node also subscribes directly
    # to that topic, bypassing our safety adapter and receiving a second copy
    # of every navigation command.  Scope a remap around Nav2 so the smoother
    # publishes only to the adapter's dedicated input topic.
    nav2_with_dedicated_output = GroupAction([
        SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_nav_output'),
        nav2,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock published by Gazebo or the fake driver.'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Absolute path to the static map YAML file.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Absolute path to the Nav2 parameter YAML file.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Transition Nav2 lifecycle nodes to active automatically.'),
        nav2_with_dedicated_output,
        Node(
            package='landerpi_navigation',
            executable='robot_pose_bridge.py',
            name='robot_pose_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='landerpi_navigation',
            executable='goal_bridge.py',
            name='goal_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
