#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('robotbase_sim')
    bringup = get_package_share_directory('robotbase_bringup')
    params_root = os.environ.get(
        'ROBOTBASE_PARAMS_DIR',
        os.path.join(os.path.expanduser('~'), 'robotbase_ws', 'params'))
    map_file = LaunchConfiguration('map')
    localization = LaunchConfiguration('localization')
    params_file = LaunchConfiguration('params_file')
    tf_prefix = LaunchConfiguration('tf_prefix')
    default_map = os.path.join(sim_share, 'maps', 'test_arena.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'localization', default_value='amcl',
            description='amcl (RViz 2D Pose Estimate) or static (ground-truth test mode).'),
        DeclareLaunchArgument('tf_prefix', default_value='robot'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(params_root, 'sim', 'nav2.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'nav2.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'localization': localization,
                'map': map_file,
                'tf_prefix': tf_prefix,
                'odom_topic': '/odom',
                'params_file': params_file,
            }.items(),
        ),
    ])
