#!/usr/bin/env python3

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def configure_navigation(context):
    """Optionally initialize AMCL at the selected world's spawn pose."""
    params = LaunchConfiguration('params_file')
    pose = {key: LaunchConfiguration(f'initial_pose_{key}').perform(context)
            for key in ('x', 'y', 'yaw')}
    if any(pose.values()):
        if not all(pose.values()):
            raise ValueError('Specify initial_pose_x, initial_pose_y and initial_pose_yaw together')
        for value in pose.values():
            if not math.isfinite(float(value)):
                raise ValueError('Initial pose must be finite')
        params = RewrittenYaml(source_file=params, param_rewrites={
            f'amcl.ros__parameters.initial_pose.{key}': value for key, value in pose.items()
        }, convert_types=True)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robotbase_bringup'), 'launch', 'nav2.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'localization': LaunchConfiguration('localization'),
            'map': LaunchConfiguration('map'), 'tf_prefix': LaunchConfiguration('tf_prefix'),
            'odom_topic': '/odom', 'params_file': params,
        }.items())]


def generate_launch_description():
    sim_share = get_package_share_directory('robotbase_sim')
    params_root = os.environ.get(
        'ROBOTBASE_PARAMS_DIR',
        os.path.join(os.path.expanduser('~'), 'robotbase_ws', 'params'))
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
        DeclareLaunchArgument('initial_pose_x', default_value=''),
        DeclareLaunchArgument('initial_pose_y', default_value=''),
        DeclareLaunchArgument('initial_pose_yaw', default_value=''),
        OpaqueFunction(function=configure_navigation),
    ])
