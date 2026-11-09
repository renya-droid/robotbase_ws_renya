#!/usr/bin/env python3
"""Velodyne observation range / costmap clearing study (simulation only).

Starts sim.launch.py with an open outdoor world, a standalone costmap using the
real robot's Velodyne obstacle-layer values, the coverage visualizer, the
obstacle scenario and RViz.

  ros2 launch robotbase_sim velodyne_clearing_study.launch.py
  ros2 launch robotbase_sim velodyne_clearing_study.launch.py scenario_loop:=true
  ros2 launch robotbase_sim velodyne_clearing_study.launch.py rviz:=false \\
      gui:=false shutdown_when_done:=true output_dir:=/path/to/result
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

WORLD_NAME = 'velodyne_clearing_study'


def generate_launch_description():
    sim_share = get_package_share_directory('robotbase_sim')
    params_root = os.environ.get(
        'ROBOTBASE_PARAMS_DIR',
        os.path.join(os.path.expanduser('~'), 'robotbase_ws', 'params'))
    laserscan_yaml = os.path.join(params_root, 'sim', 'velodyne_laserscan.yaml')
    with open(laserscan_yaml, 'r', encoding='utf-8') as f:
        scan_params = yaml.safe_load(f)['vlp16_to_scan']['ros__parameters']

    tf_prefix = LaunchConfiguration('tf_prefix')
    odom_frame = PythonExpression(["'", tf_prefix, "' + '/odom'"])
    base_frame = PythonExpression(["'", tf_prefix, "' + '/base_footprint'"])
    shutdown_when_done = LaunchConfiguration('shutdown_when_done')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'tf_prefix': tf_prefix,
            'world': os.path.join(sim_share, 'worlds', f'{WORLD_NAME}.sdf'),
            'laserscan_params_file': laserscan_yaml,
        }.items())

    set_pose_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='velodyne_study_set_pose_bridge', output='screen',
        arguments=[f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose'])

    costmap_params = RewrittenYaml(
        source_file=LaunchConfiguration('costmap_params_file'),
        param_rewrites={'global_frame': odom_frame, 'robot_base_frame': base_frame},
        convert_types=True)
    costmap = Node(
        package='nav2_costmap_2d', executable='nav2_costmap_2d', name='costmap',
        namespace='velodyne_study', output='screen', parameters=[costmap_params])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager', namespace='velodyne_study', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['costmap'],
                     'bond_timeout': 0.0}])

    viz = Node(
        package='robotbase_sim', executable='velodyne_coverage_viz.py',
        name='velodyne_coverage_viz', output='screen',
        parameters=[{
            'use_sim_time': True,
            'ring_ids': scan_params['ring_ids'],
            'ring_max_distances': scan_params['ring_max_distances'],
            'resolution': scan_params['resolution'],
            'sensor_height': 0.72,  # robotbase.sdf / robotbase.urdf lidar_link z
            'sensor_max_range': 30.0,  # robotbase.sdf gpu_lidar range max
            'sensor_min_range': 0.20,  # robotbase.sdf gpu_lidar range min
            'raytrace_max_range': 20.0,
            'use_imu_tilt': ParameterValue(LaunchConfiguration('use_imu_tilt'), value_type=bool),
        }],
        remappings=[('velodyne_points', '/velodyne_points'), ('scan3', '/scan3'),
                    ('imu', '/imu')])

    scenario = Node(
        package='robotbase_sim', executable='velodyne_clearing_scenario.py',
        name='velodyne_clearing_scenario', output='screen',
        condition=IfCondition(LaunchConfiguration('scenario')),
        parameters=[{
            'use_sim_time': True,
            'world_name': WORLD_NAME,
            'frame_id': odom_frame,
            'costmap_topic': '/velodyne_study/costmap',
            'loop': ParameterValue(LaunchConfiguration('scenario_loop'), value_type=bool),
            'output_dir': ParameterValue(LaunchConfiguration('output_dir'), value_type=str),
            'shutdown_when_done': ParameterValue(shutdown_when_done, value_type=bool),
        }])
    stop_after_scenario = RegisterEventHandler(
        OnProcessExit(target_action=scenario, on_exit=[EmitEvent(event=Shutdown(
            reason='velodyne clearing scenario finished'))]),
        condition=IfCondition(shutdown_when_done))

    rviz = Node(
        package='rviz2', executable='rviz2', name='velodyne_study_rviz', output='screen',
        arguments=['-d', os.path.join(sim_share, 'rviz', 'velodyne_clearing_study.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Gazebo GUI (RViz is the main view)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('tf_prefix', default_value='robot'),
        DeclareLaunchArgument('scenario', default_value='true',
                              description='Run the obstacle place/move/remove timeline'),
        DeclareLaunchArgument('scenario_loop', default_value='false'),
        DeclareLaunchArgument('shutdown_when_done', default_value='false'),
        DeclareLaunchArgument('use_imu_tilt', default_value='false',
                              description='Tilt ground-contact curves by IMU roll/pitch'),
        DeclareLaunchArgument('output_dir', default_value='',
                              description='Default: ~/.ros/velodyne_clearing_study/<time>'),
        DeclareLaunchArgument(
            'costmap_params_file',
            default_value=os.path.join(params_root, 'sim', 'velodyne_clearing_study.yaml')),
        sim,
        TimerAction(period=2.5, actions=[set_pose_bridge, viz]),
        TimerAction(period=5.0, actions=[costmap, lifecycle]),
        TimerAction(period=7.0, actions=[scenario, rviz]),
        stop_after_scenario,
    ])
