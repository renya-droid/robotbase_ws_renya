"""Opt-in overlay of the existing simulation params; no real/local config changes."""
import atexit
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def configure(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)
    source = Path(value('params_file')).expanduser().resolve()
    params = yaml.safe_load(source.read_text())
    initial_x, initial_y = value('initial_pose_x'), value('initial_pose_y')
    if initial_x or initial_y:
        if not initial_x or not initial_y:
            raise ValueError('Specify both initial_pose_x and initial_pose_y')
        params['amcl']['ros__parameters']['initial_pose'].update(x=float(initial_x), y=float(initial_y))
    global_params = params['global_costmap']['global_costmap']['ros__parameters']
    if value('people_enabled').lower() == 'true':
        if 'people_layer' in global_params['plugins']:
            raise ValueError('Base params already contain people_layer; choose a base without this layer.')
        global_params['plugins'].append('people_layer')
        global_params['people_layer'] = dict(
            plugin='external_camera_people_layer::PeopleLayer', enabled=True,
            people_topic=value('people_topic'), radius=float(value('people_radius')),
            max_cost=int(value('people_max_cost')), timeout=float(value('people_timeout')))
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='robotbase_people_', delete=False) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        generated = Path(stream.name)
    atexit.register(lambda: generated.unlink(missing_ok=True))
    sim = Path(get_package_share_directory('robotbase_sim'))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim/'launch/navigation.launch.py')),
        launch_arguments=dict(map=value('map'), localization=value('localization'),
            tf_prefix=value('tf_prefix'), params_file=str(generated),
            initial_pose_x=initial_x, initial_pose_y=initial_y,
            initial_pose_yaw=value('initial_pose_yaw') if initial_x and initial_y else '').items())]


def generate_launch_description():
    sim = Path(get_package_share_directory('robotbase_sim'))
    # Resolve this checkout, not the old ~/robotbase_ws default, when installed with symlinks.
    import os
    default_params = Path(os.environ.get('ROBOTBASE_PARAMS_DIR',
        str(Path(__file__).resolve().parents[3]/'params')))/'sim/nav2.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(default_params)),
        DeclareLaunchArgument('map', default_value=str(sim/'maps/test_arena.yaml')),
        DeclareLaunchArgument('localization', default_value='amcl'),
        DeclareLaunchArgument('tf_prefix', default_value='robot'),
        DeclareLaunchArgument('initial_pose_x', default_value=''),
        DeclareLaunchArgument('initial_pose_y', default_value=''),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('people_enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('people_topic', default_value='/external_camera/people'),
        DeclareLaunchArgument('people_radius', default_value='1.0'),
        DeclareLaunchArgument('people_max_cost', default_value='220'),
        DeclareLaunchArgument('people_timeout', default_value='1.5'),
        OpaqueFunction(function=configure),
    ])
