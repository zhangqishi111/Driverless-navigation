#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    mode = LaunchConfiguration('mode').perform(context)

    if mode not in ('mapping', 'navigation'):
        raise RuntimeError(
            "Invalid mode. Use mode:=mapping or mode:=navigation"
        )

    driver_dir = get_package_share_directory(
        'landerpi_base_driver'
    )
    bringup_dir = get_package_share_directory(
        'landerpi_bringup'
    )
    navigation_dir = get_package_share_directory(
        'landerpi_navigation'
    )

    real_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                driver_dir,
                'launch',
                'real_base.launch.py',
            )
        )
    )

    if mode == 'mapping':
        selected_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_dir,
                    'launch',
                    'mapping.launch.py',
                )
            ),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        )

        mode_message = LogInfo(
            msg='Starting real mapping mode: '
                'SLAM Toolbox owns map -> odom'
        )
    else:
        selected_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    navigation_dir,
                    'launch',
                    'navigation_real.launch.py',
                )
            )
        )

        mode_message = LogInfo(
            msg='Starting real navigation mode: '
                'AMCL owns map -> odom'
        )

    return [
        mode_message,
        real_base_launch,
        selected_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='navigation',
            description='Real vehicle mode: mapping or navigation',
        ),
        OpaqueFunction(function=launch_setup),
    ])
