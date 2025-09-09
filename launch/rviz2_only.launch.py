#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    rviz_config_dir = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(
            get_package_share_directory('lucia_navigation2'),
            'rviz',
            'tb3_navigation2.rviz'))

    # AMD Mesa GLSL error workaround environment variables
    amd_mesa_env_vars = {
        'QT_OPENGL': 'software',
        'QT_QUICK_BACKEND': 'software',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'GALLIUM_DRIVER': 'llvmpipe',
        'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
        'MESA_GL_VERSION_OVERRIDE': '4.5',
        'MESA_GLSL_VERSION_OVERRIDE': '450',
        'AMD_DEBUG': 'nohyperz',
        'RADV_DEBUG': 'noshaderballot',
        'QT_LOGGING_RULES': 'qt.qpa.gl.debug=false'
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_dir,
            description='Full path to RViz config file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Set AMD Mesa workaround environment variables
        *[SetEnvironmentVariable(name=key, value=value) 
          for key, value in amd_mesa_env_vars.items()],

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', rviz_config_dir,
                '--ros-args', 
                '--log-level', 'error'  # Reduce log verbosity
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            additional_env=amd_mesa_env_vars,
            output='screen'),
    ])