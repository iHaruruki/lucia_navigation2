# lucia_nav2
### Node and Topic
## Dependency
```shell
sudo apt install ros-humble-navigation2    
sudo apt install ros-humble-nav2-bringup
```
## Setup
```
$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone https://github.com/iHaruruki/lucia_nav2.git #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```
## Usage
### Launch Lucia's motor and LiDAR
```shell
ros2 launch lucia_controller bringup.launch.py
```
### Run Navigation2
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

Tune `param/lucia.yaml` for your robot (velocities, radii, frames). If you omit it, `waffle.yaml` is used.

If `navigate_to_pose` action server is unavailable, ensure `lifecycle_manager` section exists in your params file and autostart is true.

![Nav2 Video](media/nav2.gif)

## License
## Authors
## References
