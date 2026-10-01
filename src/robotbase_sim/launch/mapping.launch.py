#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('robotbase_bringup')
    params_root = os.environ.get(
        'ROBOTBASE_PARAMS_DIR',
        os.path.join(os.path.expanduser('~'), 'robotbase_ws', 'params'))
    tf_prefix = LaunchConfiguration('tf_prefix')
    slam_config_file = LaunchConfiguration('slam_config_file')
    return LaunchDescription([
        DeclareLaunchArgument('tf_prefix', default_value='robot'),
        DeclareLaunchArgument(
            'slam_config_file',
            default_value=os.path.join(params_root, 'sim', 'slam_toolbox.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'slam.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'tf_prefix': tf_prefix,
                'slam_config_file': slam_config_file,
            }.items(),
        ),
    ])
