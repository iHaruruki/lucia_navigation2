# lucia_nav2
### Node and Topic
```mermaid
flowchart LR
    A(["/"]) ==> B["/"] ==> C(["/"])
```
## Dependency
```
$ sudo apt install ros-humble-twist-mux
$ sudo apt install ros-humble-navigation2
$ sudo apt install ros-humble-nav2-bringup
$ sudo apt install ros-humble-robot-localization
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
```
ros2 run lucia_controller lucia_controller_node
ros2 launch urg_node2 urg_node2.launch.py
ros2 launch lucia_description robot.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 launch lucia_navigation2 navigation2.launch.py
```

## License
## Authors
## References
