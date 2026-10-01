#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    pkg_share = get_package_share_directory('robotbase_sim')
    bringup_share = get_package_share_directory('robotbase_bringup')
    world = os.path.join(pkg_share, 'worlds', 'test_arena.sdf')
    model = os.path.join(pkg_share, 'models', 'robotbase.sdf')
    urdf = os.path.join(bringup_share, 'urdf', 'robotbase.urdf')
    params_root = os.environ.get(
        'ROBOTBASE_PARAMS_DIR',
        os.path.join(os.path.expanduser('~'), 'robotbase_ws', 'params'))

    with open(urdf, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    gui = LaunchConfiguration('gui')
    tf_prefix = LaunchConfiguration('tf_prefix')
    twist_mux_params_file = LaunchConfiguration('twist_mux_params_file')
    laserscan_params_file = LaunchConfiguration('laserscan_params_file')
    idle_twist_params_file = LaunchConfiguration('idle_twist_params_file')
    frame_prefix = PythonExpression(["'", tf_prefix, "/'"])
    base_partition = os.environ.get('GZ_PARTITION', 'koko')
    session_partition = f'{base_partition}_sim_{os.getpid()}'
    configured_model = ReplaceString(
        source_file=model,
        replacements={'ROBOTBASE_TF_PREFIX': tf_prefix},
    )

    def gz_topic(name):
        return PathJoinSubstitution(['/', tf_prefix, name])

    def bridge_arg(name, type_suffix):
        return PythonExpression([
            "'/' + '", tf_prefix, f"' + '/{name}{type_suffix}'",
        ])

    gz_cmd_vel = gz_topic('cmd_vel')
    gz_odom = gz_topic('odom')
    gz_tf = gz_topic('tf')
    gz_joint_states = gz_topic('joint_states')
    gz_points = gz_topic('velodyne_points/points')
    gz_imu = gz_topic('imu')

    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        condition=IfCondition(gui),
    )
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world],
        output='screen',
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'frame_prefix': frame_prefix,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', configured_model,
            '-name', tf_prefix,
            '-x', '0.0', '-y', '0.0', '-z', '0.01', '-Y', '0.0',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robotbase_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            bridge_arg('cmd_vel', '@geometry_msgs/msg/Twist]gz.msgs.Twist'),
            bridge_arg('odom', '@nav_msgs/msg/Odometry[gz.msgs.Odometry'),
            bridge_arg('tf', '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'),
            bridge_arg('joint_states', '@sensor_msgs/msg/JointState[gz.msgs.Model'),
            bridge_arg(
                'velodyne_points/points',
                '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'),
            bridge_arg('imu', '@sensor_msgs/msg/Imu[gz.msgs.IMU'),
        ],
        remappings=[
            (gz_cmd_vel, '/cmd_vel'),
            (gz_odom, '/odom'),
            (gz_tf, '/tf'),
            (gz_joint_states, '/joint_states'),
            (gz_points, '/velodyne_points'),
            (gz_imu, '/imu'),
        ],
    )

    vlp16_to_scan = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name='vlp16_to_scan',
        output='screen',
        parameters=[laserscan_params_file],
        remappings=[
            ('velodyne_points', '/velodyne_points'),
            ('scan', '/scan'),
            ('scan3', '/scan3'),
        ],
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params_file],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    idle_twist = Node(
        package='robotbase_sim',
        executable='idle_twist_publisher.py',
        name='idle_twist_publisher',
        output='screen',
        parameters=[idle_twist_params_file],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_PARTITION', session_partition),
        LogInfo(msg=f'Gazebo session partition: {session_partition}'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo GUI. Set false for a headless run.'),
        DeclareLaunchArgument('tf_prefix', default_value='robot'),
        DeclareLaunchArgument(
            'twist_mux_params_file',
            default_value=os.path.join(params_root, 'sim', 'twist_mux.yaml')),
        DeclareLaunchArgument(
            'laserscan_params_file',
            default_value=os.path.join(params_root, 'sim', 'velodyne_laserscan.yaml')),
        DeclareLaunchArgument(
            'idle_twist_params_file',
            default_value=os.path.join(params_root, 'sim', 'idle_twist.yaml')),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        TimerAction(period=1.5, actions=[spawn_robot]),
        TimerAction(period=2.0, actions=[bridge, vlp16_to_scan, twist_mux, idle_twist]),
    ])
