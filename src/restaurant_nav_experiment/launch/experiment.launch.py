"""Robot + people in the restaurant with one costmap condition.

condition:=B0  LiDAR only (base params/sim/nav2.yaml, no people layer)
condition:=B1  every person as one circle (external_camera_people_layer defaults)
condition:=P   walking people as circles; seated people with their own layer
               (seated_radius:=0 leaves seated people to LiDAR and the map)
Base params, robot model and real-robot settings are read, never modified; the
per-condition Nav2 YAML is a temporary file.
"""
import atexit
import json
import os
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LAYER = 'external_camera_people_layer::PeopleLayer'


def default_params():
    if os.environ.get('ROBOTBASE_PARAMS_DIR'):
        return Path(os.environ['ROBOTBASE_PARAMS_DIR'])/'sim/nav2.yaml'
    for parent in Path(__file__).resolve().parents:
        if (parent/'params/sim/nav2.yaml').is_file():
            return parent/'params/sim/nav2.yaml'
    return Path.home()/'robotbase_ws_renya/params/sim/nav2.yaml'


def nav2_params(base_file, condition, v):
    params = yaml.safe_load(Path(base_file).read_text())
    g = params['global_costmap']['global_costmap']['ros__parameters']
    if any('people' in p for p in g['plugins']):
        raise ValueError('Base params already contain a people layer')
    layers = {}
    if condition == 'B1':
        layers['people_layer'] = ('/external_camera/people', v('all_radius'), v('all_cost'))
    elif condition == 'P':
        layers['people_walking_layer'] = ('/external_camera/people_walking', v('walking_radius'), v('walking_cost'))
        if float(v('seated_radius')) > 0:
            layers['people_seated_layer'] = ('/external_camera/people_seated', v('seated_radius'), v('seated_cost'))
    elif condition != 'B0':
        raise ValueError('condition must be B0, B1 or P')
    # Under full simulation load the BT's 20 ms acknowledgement timeout made
    # compute_path_to_pose fail spuriously; same value for every condition.
    params['bt_navigator']['ros__parameters']['default_server_timeout'] = int(v('bt_server_timeout_ms'))
    # The local costmap has no static layer and the 0.72 m LiDAR (±15°) cannot
    # see 0.45 m chairs within about 1 m, so MPPI turned the robot's corner into
    # chairs. Same addition for every condition; base params are not modified.
    if v('local_static') == 'true':
        loc = params['local_costmap']['local_costmap']['ros__parameters']
        if 'static_layer' not in loc['plugins']:
            loc['plugins'].insert(0, 'static_layer')
            loc['static_layer'] = dict(plugin='nav2_costmap_2d::StaticLayer', map_subscribe_transient_local=True)
    for name, (topic, radius, cost) in layers.items():
        g['plugins'].append(name)
        g[name] = dict(plugin=LAYER, enabled=True, people_topic=topic, radius=float(radius),
                       max_cost=int(cost), timeout=float(v('people_timeout')))
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=f'restaurant_nav_{condition}_', delete=False) as f:
        yaml.safe_dump(params, f, sort_keys=False)
    atexit.register(lambda: Path(f.name).unlink(missing_ok=True))
    return f.name, layers


def configure(context):
    v = lambda k: LaunchConfiguration(k).perform(context)
    share = Path(get_package_share_directory('restaurant_nav_experiment'))
    sim = Path(get_package_share_directory('robotbase_sim'))
    meta_file = share/'generated/experiment.json'
    meta = json.loads(meta_file.read_text())
    bx, by = meta['base_world']
    condition, scenario = v('condition'), v('scenario')
    params_file, layers = nav2_params(v('params_file') or default_params(), condition, v)
    # robotbase_sim/sim.launch.py renames the Gazebo partition to
    # <GZ_PARTITION>_sim_<launch pid>; the people node's gz CLI must match it.
    partition = f"{os.environ.get('GZ_PARTITION', 'koko')}_sim_{os.getpid()}"
    world = share/f'generated/worlds/restaurant_robot_{scenario}.sdf'
    actions = [
        LogInfo(msg=f'condition={condition} layers={list(layers)} world={world.name} base=({bx},{by}) '
                    f'gz_partition={partition} nav2_params={params_file}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/robot_sim.launch.py')),
            launch_arguments=dict(world=str(world), gui=v('gui'), spawn_x=str(bx), spawn_y=str(by),
                                  spawn_yaw='0.0', tf_prefix='robot').items()),
        TimerAction(period=6.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim/'launch/navigation.launch.py')),
            launch_arguments=dict(map=str(share/'generated/maps/restaurant.yaml'), localization='static',
                                  tf_prefix='robot', params_file=params_file).items())]),
        Node(package='restaurant_nav_experiment', executable='ground_truth_tf', output='screen',
             parameters=[dict(use_sim_time=True, spawn_x=float(bx), spawn_y=float(by))]),
        Node(package='restaurant_nav_experiment', executable='people_bridge', output='screen',
             parameters=[dict(use_sim_time=True, base_x=float(bx), base_y=float(by))]),
    ]
    if scenario != 'base':  # 'base' = furniture only: a no-people reachability check
        actions.append(TimerAction(period=4.0, actions=[Node(
            package='restaurant_sim', executable='people', output='screen',
            additional_env={'GZ_PARTITION': partition},
            parameters=[dict(scenario=scenario, use_sim_time=True, failure_policy='log',
                             seed=int(v('seed')), pose_transport='ros')])]))
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='restaurant_pose_bridge',
            output='screen', additional_env={'GZ_PARTITION': partition},
            arguments=['/world/restaurant/set_pose@ros_gz_interfaces/srv/SetEntityPose']))
    if v('run_trips') == 'true':
        actions.append(Node(package='restaurant_nav_experiment', executable='serving_runner', output='screen',
            additional_env={'GZ_PARTITION': partition},
            parameters=[dict(use_sim_time=True, condition=condition, scenario=scenario, seed=int(v('seed')),
                             experiment_json=str(meta_file), out_dir=v('out_dir'),
                             start_sim=float(v('start_sim')), stop_new_trips_sim=float(v('stop_sim')))]))
    return actions


def generate_launch_description():
    args = dict(condition='B0', scenario='high', seed='42', gui='false', params_file='',
                all_radius='1.0', all_cost='220', walking_radius='1.0', walking_cost='220',
                seated_radius='0.0', seated_cost='120', people_timeout='1.5', bt_server_timeout_ms='200', local_static='true',
                run_trips='false', out_dir='artifacts/restaurant_nav/latest', start_sim='40.0', stop_sim='330.0')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=d) for k, d in args.items()]
                             + [OpaqueFunction(function=configure)])
