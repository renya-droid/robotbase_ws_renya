#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share = get_package_share_directory('robotbase_bringup')
    params_root = os.environ.get(
        'ROBOTBASE_PARAMS_DIR',
        os.path.join(os.path.expanduser('~'), 'robotbase_ws', 'params'))
    default_params = os.path.join(params_root, 'real', 'nav2.yaml')
    nav_to_pose_bt = os.path.join(
        bringup_share, 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery_no_spin.xml')
    nav_through_poses_bt = os.path.join(
        bringup_share, 'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery_no_spin.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    tf_prefix = LaunchConfiguration('tf_prefix')
    base_frame = PathJoinSubstitution([tf_prefix, 'base_footprint'])
    odom_frame = PathJoinSubstitution([tf_prefix, 'odom'])
    odom_topic = LaunchConfiguration('odom_topic')

    configured = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'base_frame_id': base_frame,
            'robot_base_frame': base_frame,
            'odom_frame_id': odom_frame,
            'odom_topic': odom_topic,
            'local_costmap.local_costmap.ros__parameters.global_frame': odom_frame,
            'behavior_server.ros__parameters.local_frame': odom_frame,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt,
        },
        convert_types=True,
    )
    parameters = ParameterFile(configured, allow_substs=True)
    amcl_mode = IfCondition(PythonExpression(["'", localization, "' == 'amcl'"]))
    static_mode = IfCondition(PythonExpression(["'", localization, "' == 'static'"]))
    map_mode = IfCondition(PythonExpression(["'", localization, "' != 'slam'"]))
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[parameters, {'yaml_filename': map_file}],
        remappings=tf_remaps, condition=map_mode,
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[parameters], remappings=tf_remaps, condition=amcl_mode,
    )
    static_localization = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom_ground_truth', output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', odom_frame,
        ],
        condition=static_mode,
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=amcl_mode,
    )
    static_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_static_map', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        condition=static_mode,
    )

    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[parameters],
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    )
    smoother = Node(
        package='nav2_smoother', executable='smoother_server',
        name='smoother_server', output='screen', parameters=[parameters],
        remappings=tf_remaps,
    )
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[parameters],
        remappings=tf_remaps,
    )
    behaviors = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen', parameters=[parameters],
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    )
    navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', parameters=[parameters],
        remappings=tf_remaps,
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen', parameters=[parameters],
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[parameters],
        remappings=tf_remaps,
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server', 'smoother_server', 'planner_server',
                'behavior_server', 'bt_navigator', 'velocity_smoother',
                'waypoint_follower',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'localization', default_value='amcl',
            description='amcl, static, or slam (map and localization supplied by slam_toolbox).'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Absolute map YAML path; unused when localization:=slam.'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('tf_prefix', default_value='robot'),
        DeclareLaunchArgument('odom_topic', default_value='/odom/filtered'),
        map_server,
        amcl,
        static_localization,
        localization_lifecycle,
        static_lifecycle,
        controller,
        smoother,
        planner,
        behaviors,
        navigator,
        velocity_smoother,
        waypoint_follower,
        navigation_lifecycle,
    ])
