#!/usr/bin/env python3
"""Launch map-based navigation against the final real-sandbox map.

This wrapper deliberately reuses ``navigation.launch.py`` so the complete
Nav2 parameter set stays shared with simulation. Its only fixed differences
are the cleaned real map and wall-clock time. Real-world AMCL, controller,
and costmap tuning must happen from measured vehicle data, not here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    navigation_dir = get_package_share_directory('landerpi_navigation')
    bringup_dir = get_package_share_directory('landerpi_bringup')

    navigation_launch = os.path.join(
        navigation_dir, 'launch', 'navigation.launch.py')
    real_map = os.path.join(
        bringup_dir, 'maps', 'real_sandbox_clean.yaml')
    params_file = os.path.join(
        navigation_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'map': real_map,
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
    ])
