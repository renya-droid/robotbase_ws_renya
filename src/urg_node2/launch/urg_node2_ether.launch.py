# Copyright 2026
# Licensed under the Apache License, Version 2.0

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('urg_node2'), 'config', 'params_ether.yaml')
    with open(config_path, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)['urg_node2']['ros__parameters']

    node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name'),
        namespace='',
        output='screen',
        remappings=[('scan', LaunchConfiguration('scan_topic_name'))],
        parameters=[
            config,
            {
                'ip_address': ParameterValue(
                    LaunchConfiguration('ip_address'), value_type=str),
                'ip_port': ParameterValue(
                    LaunchConfiguration('ip_port'), value_type=int),
                'frame_id': ParameterValue(
                    LaunchConfiguration('frame_id'), value_type=str),
            },
        ],
    )

    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_CONFIGURE))],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name', default_value='urg_node2'),
        DeclareLaunchArgument('scan_topic_name', default_value='hokuyo_scan'),
        DeclareLaunchArgument('ip_address', default_value='192.168.0.9'),
        DeclareLaunchArgument('ip_port', default_value='10940'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        node,
        configure,
        activate,
    ])
