import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_dir = get_package_share_directory('sim_py_01')
    urdf = os.path.join(package_dir, 'urdf', 'wheel_robot_simple.urdf')
    rviz = os.path.join(package_dir, 'rviz', 'wheel_robot_simple_nav2.rviz')
    world = os.path.join(package_dir, 'world', 'maze.world')
    map = os.path.join(package_dir, 'maps', 'map_01.yaml')
    nav2_params = os.path.join(package_dir, 'config', 'navigation_params.yaml')
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(package_dir, 'models')
    nav2_package_dir = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf],),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf],),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s',
                 'libgazebo_ros_init.so', '-s',
                 'libgazebo_ros_factory.so', world],
            output='screen',),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-topic', '/robot_description',
                       '-entity', 'wheel_robot_simple'],),
        SetRemap(src='cmd_vel', dst='/wheel_robot_simple/cmd_vel'),
        SetRemap(src='odom', dst='/wheel_robot_simple/odom'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_package_dir, 'launch',
                             'bringup_launch.py')),
            launch_arguments={
                'map': map,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params}.items(),),
    ])
