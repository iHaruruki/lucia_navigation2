# lucia_nav2
### Node and Topic
## Dependency
```shell
$ sudo apt install ros-humble-navigation2
$ sudo apt install ros-humble-nav2-bringup
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
```
ros2 launch lucia_navigation2 navigation2.launch.py map:=$HOME/ros2_ws/src/lucia_navigation2/map/map.yaml use_sim_time:=false
```
![](media/nav2.gif)

## License
## Authors
## References
