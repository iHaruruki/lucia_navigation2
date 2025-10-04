# 🤖 lucia_nav2
## 🚀 Overview
lucia_navigation2 is a ROS 2 Navigation2 integration package for the Lucia mobile robot, providing a structured setup for mapping, localization, path planning, behavior tree–based task flow, and controller tuning across simulation and real hardware.

## 🛠️ Setup
Install packages
```shell
sudo apt install ros-humble-navigation2    
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-collision-monitor
Clone & Build
```shell
cd ~/ros2_ws/src  #Go to ros workspace
git clone https://github.com/iHaruruki/lucia_nav2.git #clone this package
cd ~/ros2_ws
colcon build --symlink-install --packages-select lucia_navigation2
source install/setup.bash
```
## 🎮 Usage
Lucia have two modes of operation.

### Navigation mode
#### Launch Lucia's motor and LiDAR
```shell
ros2 launch lucia_controller bringup.launch.py
```
#### Launch Nav2 & rviz2
```shell
ros2 launch lucia_navigation2 navigation2.launch.py 
map:=$HOME/ros2_ws/maps/map.yaml 
params_file:=$HOME/ros2_ws/src/lucia_navigation2/param/lucia.yaml 
use_sim_time:=false
```
Launch arguments:
- map: path to map yaml
- params_file: path to Navigation2 params (default tries param/lucia.yaml then waffle.yaml)
- rviz_config: custom rviz config file
- namespace: robot namespace (empty by default)
- use_sim_time: use simulated clock

#### Initialize the Location of Lucia
First, find where the robot is on the map. Check where your robot is in the room./
Set the pose of the robot in RViz. Click on the `2D Pose Estimate` button and point the location of the robot on the map. The direction of the green arrow is the orientation of Lucia.

#### Send a Goal Pose
Pick a target location for Lucia on the map. You can send Lucia a goal position and a goal orientation by using the `Nav2 Goal` or the GoalTool buttons.

![Nav2 Video](media/nav2.gif)

### Navigating while Mapping
#### Launch Lucia's motor and LiDAR
```shell
ros2 launch lucia_controller bringup.launch.py
```
#### Launch Navigation2
```shell
ros2 launch nav2_bringup navigation_launch.py
```
#### Launch SLAM
```shell
ros2 launch slam_toolbox online_async_launch.py
```
#### Working with SLAM
Move your robot by requesting a goal through RViz or the ROS 2 CLI, ie:
```shell
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```
You should see the map update live! To save this map to file:
```shell
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 📜 License

## 👤 Authors
- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

## 📚 References
[Nav2](https://docs.nav2.org/index.html)
[(SLAM) Navigating While Mapping](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)