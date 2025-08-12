# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Generic launch file adapted from TurtleBot3 example for a custom robot (Lucia).
# You can override map, params_file, rviz_config, namespace, and use_sim_time at launch.
PACKAGE_NAME = 'lucia_navigation2'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    namespace = LaunchConfiguration('namespace')

    default_share_dir = get_package_share_directory(PACKAGE_NAME)

    default_map = os.path.join(default_share_dir, 'map', 'map.yaml')
    default_params = os.path.join(default_share_dir, 'param', 'lucia.yaml')
    # Fallback if lucia.yaml not present yet; keep waffle.yaml for backward compatibility
    if not os.path.exists(default_params):
        default_params = os.path.join(default_share_dir, 'param', 'waffle.yaml')
    default_rviz = os.path.join(default_share_dir, 'rviz', 'tb3_navigation2.rviz')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file_dir = os.path.join(nav2_bringup_dir, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Full path to the RVIZ config file to use'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'namespace': namespace,
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true'
            }.items(),
        ),

        # Launch RViz2 with provided configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
