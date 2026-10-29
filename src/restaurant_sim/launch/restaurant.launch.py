from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch(context):
    scenario=LaunchConfiguration('scenario').perform(context)
    if scenario not in ('base','low','medium','high'):raise ValueError('Unknown scenario')
    world=get_package_share_directory('restaurant_sim')+'/worlds/restaurant_'+scenario+'.sdf'
    actions=[ExecuteProcess(cmd=['gz','sim','-r',world],output='screen'),Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])]
    if scenario!='base':actions.append(Node(package='restaurant_sim',executable='people',parameters=[{'scenario':scenario,'use_sim_time':True}],output='screen'))
    return actions

def generate_launch_description():return LaunchDescription([DeclareLaunchArgument('scenario',default_value='base'),OpaqueFunction(function=launch)])
