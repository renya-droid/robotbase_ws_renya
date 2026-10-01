#!/usr/bin/env python3

import os
import subprocess
import shutil
import logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the package
    pkg_path = get_package_share_directory('sirius_description')
    
    # Path to the URDF (xacro) file and SDF file (fallback if URDF is missing)
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'sirius3.urdf.xacro')
    robot_sdf_file_abs = os.path.join(pkg_path, 'sdf', 'sirius3.sdf')

    log = logging.getLogger('display_launch')

    def convert_sdf_to_urdf(sdf_file_path, pkg_path):
        """Convert an SDF file to URDF using available sdftool (gz sdf -p, gzsdf, or sdformat).

        The function tries several common commands and updates mesh paths to package:// form.
        It raises RuntimeError if conversion fails or if no tool is found.
        """
        # prefer `gz sdf -p` (Ignition/Gazebo)
        candidates = [['gz', 'sdf', '-p', sdf_file_path], ['gzsdf', 'print', sdf_file_path], ['sdformat', 'print', sdf_file_path]]

        last_err = None
        for cmd in candidates:
            if shutil.which(cmd[0]) is None:
                continue
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, check=True)
                urdf_content = result.stdout

                # try to normalize mesh paths – replace relative/absolute with package:// path
                urdf_content = urdf_content.replace('../urdf/meshes/', f'file://{pkg_path}/urdf/meshes/')
                urdf_content = urdf_content.replace(f'file://{pkg_path}/urdf/meshes/', 'package://sirius_description/urdf/meshes/')

                # DAE (collada) files are sometimes referenced from ../dae/
                urdf_content = urdf_content.replace('../dae/', f'file://{pkg_path}/dae/')
                urdf_content = urdf_content.replace(f'file://{pkg_path}/dae/', 'package://sirius_description/dae/')

                return urdf_content
            except subprocess.CalledProcessError as e:
                last_err = e
                log.debug('Conversion attempt failed with %s: %s', cmd, e)

        raise RuntimeError('Failed to convert SDF to URDF. Tried commands and failed: {}'.format(last_err))
    
    # Path to the RViz config file
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'sirius_robot.rviz')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start joint_state_publisher_gui if true'
    )
    
    use_dummy_joint_arg = DeclareLaunchArgument(
        'use_dummy_joint',
        default_value='false',
        description='Start dummy joint_state_publisher (disable when using Unity/Gazebo)'
    )
    
    # Decide how to obtain robot_description (xacro URDF preferred, otherwise convert SDF)
    if os.path.exists(urdf_file_path):
        robot_description_value = Command(['xacro ', urdf_file_path])
    elif os.path.exists(robot_sdf_file_abs):
        try:
            robot_description_value = str(convert_sdf_to_urdf(robot_sdf_file_abs, pkg_path))
        except Exception as e:
            log.error('Failed to build URDF from SDF: %s', e)
            raise
    else:
        raise FileNotFoundError('No URDF xacro file or SDF file found in package: {}'.format(pkg_path))

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            # provide robot_description (computed above)
            'robot_description': ParameterValue(robot_description_value, value_type=str),
            'frame_prefix': 'sirius3/',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint State Publisher (for manual control of joints in RViz)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(LaunchConfiguration('use_dummy_joint'))
    )
    
    # Joint State Publisher GUI (optional - for interactive joint control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )
    
    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_gui_arg,
        use_dummy_joint_arg,
        robot_state_publisher,
        joint_state_publisher,
        # joint_state_publisher_gui,
        rviz2
    ])
