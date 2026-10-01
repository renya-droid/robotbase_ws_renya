import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import subprocess

def convert_sdf_to_urdf(sdf_file_path, pkg_path):
    # Reuse the conversion logic from sim.launch.py
    result = subprocess.run(
        ['gz', 'sdf', '-p', sdf_file_path],
        capture_output=True, text=True, check=True
    )
    urdf_content = result.stdout
    urdf_content = urdf_content.replace('../urdf/meshes/', f'file://{pkg_path}/urdf/meshes/')
    urdf_content = urdf_content.replace(f'file://{pkg_path}/urdf/meshes/', 'package://sirius_description/urdf/meshes/')
    urdf_content = urdf_content.replace('../dae/', f'file://{pkg_path}/dae/')
    urdf_content = urdf_content.replace(f'file://{pkg_path}/dae/', 'package://sirius_description/dae/')
    return urdf_content

def generate_launch_description():
    pkg_share = FindPackageShare('sirius_description')
    pkg_path = get_package_share_directory('sirius_description')
    
    # Paths
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'sirius_robot.rviz'])
    robot_sdf_file_abs = os.path.join(pkg_path, 'sdf', 'sirius3.sdf')
    
    # Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time via /clock provided by Unity'
    )

    # URDF Generation
    robot_description_content = convert_sdf_to_urdf(robot_sdf_file_abs, pkg_path)

    # Robot State Publisher (calculates TFs from joint_states)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
            'frame_prefix': 'sirius3/' # Keep prefix to match Unity configuration
        }],
        output='screen'
    )

    # Rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Note: We do NOT launch gz sim or ros_gz_bridge here.
    # Unity provides /clock, /joint_states, /odom, /tf, /cmd_vel handling.

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        rviz2
    ])
