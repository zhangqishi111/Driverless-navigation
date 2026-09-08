#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_dir = get_package_share_directory('landerpi_navigation')
    bringup_dir = get_package_share_directory('landerpi_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(
        bringup_dir,
        'maps',
        'real_sandbox_clean.yaml'
    )

    default_params = os.path.join(
        navigation_dir,
        'config',
        'amcl_real.yaml'
    )

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Real sandbox map YAML'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Real robot AMCL parameter file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use real robot clock'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate localization lifecycle nodes'
        ),
        localization,
    ])
