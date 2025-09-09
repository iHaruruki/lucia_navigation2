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

    # GLSL エラー対策用の環境変数設定
    set_qt_opengl = SetEnvironmentVariable(
        name='QT_OPENGL',
        value='software'
    )
    
    set_mesa_gl_version = SetEnvironmentVariable(
        name='MESA_GL_VERSION_OVERRIDE',
        value='3.3'
    )
    
    set_mesa_glsl_version = SetEnvironmentVariable(
        name='MESA_GLSL_VERSION_OVERRIDE',
        value='330'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_dir,
            description='Full path to RViz config file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # GLSL エラー対策用の環境変数
        set_qt_opengl,
        set_mesa_gl_version,
        set_mesa_glsl_version,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', rviz_config_dir,
                '--ros-args', 
                '--log-level', 'warn'
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            additional_env={
                'QT_LOGGING_RULES': 'qt.qpa.gl.debug=true',
                'QT_OPENGL': 'software',
                'MESA_GL_VERSION_OVERRIDE': '3.3',
                'MESA_GLSL_VERSION_OVERRIDE': '330'
            },
            output='screen'),
    ])