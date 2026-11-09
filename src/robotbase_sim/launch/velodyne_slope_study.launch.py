#!/usr/bin/env python3
"""Velodyne slope study (simulation only).

Generates a flat -> uphill -> plateau -> downhill -> flat course with the given
slope, drives the robot over it with /cmd_vel_teleop and records how the
fixed ring max distances behave when the ground is not level.

  ros2 launch robotbase_sim velodyne_slope_study.launch.py slope_deg:=10 gui:=true
  ros2 launch robotbase_sim velodyne_slope_study.launch.py slope_deg:=14 \\
      gui:=false rviz:=false shutdown_when_done:=true output_dir:=/path/to/result
"""

import importlib.util
import json
import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def load_generator():
    path = os.path.join(get_package_prefix('robotbase_sim'), 'lib', 'robotbase_sim',
                        'generate_slope_world.py')
    spec = importlib.util.spec_from_file_location('generate_slope_world', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def setup(context):
    sim_share = get_package_share_directory('robotbase_sim')
    params_root = os.environ.get(
        'ROBOTBASE_PARAMS_DIR',
        os.path.join(os.path.expanduser('~'), 'robotbase_ws', 'params'))
    laserscan_yaml = os.path.join(params_root, 'sim', 'velodyne_laserscan.yaml')
    with open(laserscan_yaml, 'r', encoding='utf-8') as f:
        scan_params = yaml.safe_load(f)['vlp16_to_scan']['ros__parameters']

    gen = load_generator()
    slope = float(LaunchConfiguration('slope_deg').perform(context))
    sdf, course = gen.build(slope)
    tmp = tempfile.mkdtemp(prefix='velodyne_slope_')
    world_file = os.path.join(tmp, 'slope.sdf')
    with open(world_file, 'w', encoding='utf-8') as f:
        f.write(sdf)
    # Kinematic robot for this study only (shared robotbase.sdf is unchanged).
    # With physics the rigid 2-wheel + 2-caster model cannot cross a concave
    # edge (casters carry the chassis, the drive wheels lift and spin), and the
    # result depended on caster tuning. Instead the drive node sets the robot
    # pose along the course; collisions, gravity and DiffDrive are removed and
    # an OdometryPublisher (2D, from the true pose) provides odom and TF.
    with open(os.path.join(sim_share, 'models', 'robotbase.sdf'), encoding='utf-8') as f:
        model = f.read()
    model = re.sub(r'<collision name="[^"]*">.*?</collision>', '', model, flags=re.S)
    model = re.sub(r'(<link name="[^"]*">)', r'\1<gravity>false</gravity>', model)
    model = model.replace('<link name="base_footprint"/>',
                          '<link name="base_footprint"><gravity>false</gravity></link>')
    model, n = re.subn(
        r'<plugin filename="gz-sim-diff-drive-system".*?</plugin>',
        """<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>ROBOTBASE_TF_PREFIX/odom</odom_frame>
      <robot_base_frame>ROBOTBASE_TF_PREFIX/base_footprint</robot_base_frame>
      <odom_topic>/ROBOTBASE_TF_PREFIX/odom</odom_topic>
      <tf_topic>/ROBOTBASE_TF_PREFIX/tf</tf_topic>
      <dimensions>2</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>""", model, flags=re.S)
    if n != 1 or '<collision' in model:
        raise RuntimeError('unexpected robotbase.sdf structure')
    model_file = os.path.join(tmp, 'robotbase_slope.sdf')
    with open(model_file, 'w', encoding='utf-8') as f:
        f.write(model)
    world = gen.WORLD_NAME

    prefix = LaunchConfiguration('tf_prefix').perform(context)
    odom_frame, base_frame = f'{prefix}/odom', f'{prefix}/base_footprint'
    shutdown_when_done = LaunchConfiguration('shutdown_when_done')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'), 'tf_prefix': prefix, 'world': world_file,
            'model_file': model_file,
            'laserscan_params_file': laserscan_yaml,
        }.items())
    world_pose_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='velodyne_study_world_pose_bridge', output='screen',
        arguments=[f'/world/{world}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   f'/world/{world}/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        remappings=[(f'/world/{world}/dynamic_pose/info', '/velodyne_study/world_poses')])
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
            'sensor_height': 0.72, 'sensor_max_range': 30.0, 'sensor_min_range': 0.20,
            'raytrace_max_range': 20.0,
            'use_imu_tilt': True,  # tilt the assumed ground plane with the robot pitch
        }],
        remappings=[('velodyne_points', '/velodyne_points'), ('scan3', '/scan3'),
                    ('imu', '/imu')])
    drive = Node(
        package='robotbase_sim', executable='velodyne_slope_drive.py',
        name='velodyne_slope_drive', output='screen',
        condition=IfCondition(LaunchConfiguration('drive')),
        parameters=[{
            'use_sim_time': True,
            'course_json': json.dumps(course),
            'world_name': world,
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'loop': ParameterValue(LaunchConfiguration('loop'), value_type=bool),
            'output_dir': ParameterValue(LaunchConfiguration('output_dir'), value_type=str),
            'shutdown_when_done': ParameterValue(shutdown_when_done, value_type=bool),
        }])
    stop_after = RegisterEventHandler(
        OnProcessExit(target_action=drive, on_exit=[EmitEvent(event=Shutdown(
            reason='slope drive finished'))]),
        condition=IfCondition(shutdown_when_done))
    rviz = Node(
        package='rviz2', executable='rviz2', name='velodyne_study_rviz', output='screen',
        arguments=['-d', os.path.join(sim_share, 'rviz', 'velodyne_slope_study.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))
    return [
        sim,
        TimerAction(period=2.5, actions=[world_pose_bridge, viz]),
        TimerAction(period=5.0, actions=[costmap, lifecycle]),
        TimerAction(period=7.0, actions=[drive, rviz]),
        stop_after,
    ]


def generate_launch_description():
    params_root = os.environ.get(
        'ROBOTBASE_PARAMS_DIR',
        os.path.join(os.path.expanduser('~'), 'robotbase_ws', 'params'))
    return LaunchDescription([
        DeclareLaunchArgument('slope_deg', default_value='10.0'),
        DeclareLaunchArgument('speed', default_value='0.4'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('drive', default_value='true'),
        DeclareLaunchArgument('loop', default_value='false',
                              description='Drive back and forth over the course'),
        DeclareLaunchArgument('tf_prefix', default_value='robot'),
        DeclareLaunchArgument('shutdown_when_done', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument(
            'costmap_params_file',
            default_value=os.path.join(params_root, 'sim', 'velodyne_clearing_study.yaml')),
        OpaqueFunction(function=setup),
    ])
