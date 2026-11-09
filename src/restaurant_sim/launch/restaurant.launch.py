import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    speed = LaunchConfiguration('speed').perform(context)
    if scenario not in ('base', 'low', 'medium', 'high'):
        raise ValueError('Unknown scenario')
    if speed not in ('1', '2'):
        raise ValueError('speed must be 1 or 2')
    world = Path(get_package_share_directory('restaurant_sim')) / 'worlds' / f'restaurant_{scenario}.sdf'
    cleanup = []
    if speed == '2':
        tree = ET.parse(world)
        tree.find('world/physics/real_time_factor').text = '2.0'
        with tempfile.NamedTemporaryFile(prefix='restaurant_2x_', suffix='.sdf', delete=False) as output:
            tree.write(output, encoding='utf-8')
            world = Path(output.name)

        def remove_world(context):
            world.unlink(missing_ok=True)
            return []

        cleanup = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=remove_world)]))]
    actions = cleanup + [
        ExecuteProcess(cmd=['gz', 'sim', '-r', str(world)], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
    ]
    if scenario != 'base':
        actions.append(Node(package='restaurant_sim', executable='people',
                            parameters=[{'scenario': scenario, 'use_sim_time': True,
                                         'failure_policy': LaunchConfiguration('failure_policy').perform(context)}],
                            output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='base'),
        DeclareLaunchArgument('speed', default_value='1', choices=['1', '2'],
                              description='Gazebo target speed: 1=normal, 2=double'),
        DeclareLaunchArgument('failure_policy', default_value='fail', choices=['fail', 'log'],
                              description='fail: stop the people node on a stall/timeout; log: record it and continue'),
        OpaqueFunction(function=launch),
    ])
