# 🤖 lucia_navigation2
[![ROS 2 Distro - Humble](https://img.shields.io/badge/ros2-Humble-blue)](https://docs.ros.org/en/humble/)

## 🚀 Overview
`lucia_navigation2` is a ROS 2 Navigation2 integration package for the Lucia, providing a structured setup for mapping, localization, path planning, behavior tree–based task flow, and controller tuning across simulation and real hardware.

---

## 🧭 Operation Modes

Lucia supports to **4 operation modes**.  
Use the mode that best matches your task:

| Icon | Mode | Description | Typical Use |
|------|------|-------------|-------------|
| 🟢 | [**Navigation mode**](#-navigation-mode) | Navigate to a single goal pose on an existing map | Point-to-point navigation |
| 🔵 | [**Waypoint follow mode**](#-waypoint-follow-mode) | Visit multiple waypoints in a specified order, then go to a final goal | Patrol routes, inspection paths |
| 🟡 | [**Patrol mode**](#-patrol-mode) | Randomly patrol within a rectangular area on the map | Area surveillance, coverage |
| 🟣 | [**Navigating while Mapping mode**](#-navigating-while-mapping-mode) | Build a new map while moving using SLAM | First-time exploration in unknown environments |

> [!TIP] 
> All modes assume that Lucia is already properly powered on and that the onboard PC is accessible over ROS 2.

---

## 🛠️ Setup

### 1. Install required packages

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-collision-monitor \
  ros-humble-nav2-simple-commander \
  ros-humble-tf-transformations \
  python3-numpy \
  python3-transforms3d
```

### 2. Clone & Build this package

```bash
cd ~/ros2_ws/src  # Go to your ROS 2 workspace source directory
git clone https://github.com/iHaruruki/lucia_navigation2.git  # Clone this package

cd ~/ros2_ws
colcon build --symlink-install --packages-select lucia_navigation2
source install/setup.bash
```

> [!WARNING] 
> Make sure that `lucia_controller` and sensor drivers (e.g., LiDAR) are already installed and buildable in the same workspace, since all modes depend on them.

---

## 🎮 Usage

Below are instructions for each operation mode.  
If you just want to quickly try things, start with **🟢 Navigation mode**.


### 🟢 Navigation mode

> Navigate to a **single goal pose** on an existing map.

#### 1. Launch Lucia's motor and LiDAR

```bash
ros2 launch lucia_controller bringup.launch.py
```

- Starts the base controller (motors) and LiDAR node.
- Run this **once**, and keep the terminal open (or run in a tmux/screen session).

#### 2. Launch Nav2 & RViz2

```bash
ros2 launch lucia_navigation2 navigation2.launch.py \
  map:=$HOME/ros2_ws/src/lucia_navigation2/map/map.yaml \
  params_file:=$HOME/ros2_ws/src/lucia_navigation2/param/lucia.yaml \
  use_sim_time:=false
```

**Launch arguments:**

- `map`: path to map `.yaml` file (must correspond to an existing `.pgm` or `.png` map)
- `params_file`: path to Navigation2 parameters  
  - By default, the launch file tries `param/lucia.yaml` and then `waffle.yaml` if not specified.
- `use_sim_time`: whether to use simulated clock (`true` for Gazebo, `false` for real robot)

#### 3. Initialize the location of Lucia (set initial pose)

1. In RViz, make sure the **map** and **laser scan** are visible.
2. Visually find where the robot is in the room, and locate the corresponding position on the map.
3. Click the **2D Pose Estimate** button in the toolbar.
4. Click and drag on the map:
   - The clicked point becomes Lucia’s estimated position.
   - The direction of the green arrow is Lucia’s yaw (orientation).

> [!NOTE] 
> This initial pose is used by AMCL / localization to start tracking the robot correctly.

![nav2 initial pose](media/nav2_initial.gif)

#### 4. Send a Goal Pose

1. Click the **Nav2 Goal** (or **GoalTool**) button in the RViz toolbar.
2. Click and drag on the target position on the map:
   - The point is the goal position.
   - The arrow direction is the desired final orientation.
3. Nav2 will plan a path and send velocity commands to Lucia.


![Nav2 Video](media/nav2_goal.gif)

> [!NOTE] 
> If everything is set correctly, you should see a global path, local trajectory, and Lucia moving towards the goal.


### 🔵 Waypoint follow mode

> Follow **multiple waypoints in sequence**, then move to a final goal pose.

#### 1. Launch Lucia's motor and LiDAR

```bash
ros2 launch lucia_controller bringup.launch.py
```

#### 2. Launch Nav2 & RViz2

```bash
ros2 launch lucia_navigation2 navigation2.launch.py \
  map:=$HOME/ros2_ws/maps/map.yaml \
  params_file:=$HOME/ros2_ws/src/lucia_navigation2/param/lucia.yaml \
  use_sim_time:=false
```

**Launch arguments:**

- `map`: path to map `.yaml`
- `params_file`: Navigation2 parameter file (behavior tree, planners, controllers, etc.)
- `use_sim_time`: set to `false` for real robot

#### 3. Run `waypoint_navi` node (send waypoints)

This node sends a list of waypoints to Nav2 and then a final goal pose.

##### 3.1 Set `initial_pose`, `waypoints`, and `goal point` in `waypoint_navi.py`

```py
# --- Set initial pose
initial_pose = make_pose(0.0, 0.0, 0.0)
self.get_logger().info('Setting initial pose...')
self.navigator.setInitialPose(initial_pose)

###################################################
# === 1. Set multiple waypoints ===
waypoints = [
    # make_pose(x, y, yaw)
    make_pose(1.0, 0.0, 0.0),    # Waypoint 1
    make_pose(1.0, 1.0, 1.57),   # Waypoint 2
    make_pose(0.0, 1.0, 3.14),   # Waypoint 3
]
####################################################
# === 2. Set final goal point (use goToPose, which provides distance/time feedback) ===
goal_pose = make_pose(2.0, 2.0, 0.0)
self.get_logger().info(
    f"Starting final goal navigation to "
    f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})..."
)
```

- `make_pose(x, y, yaw)` returns a `geometry_msgs/PoseStamped` in the map frame.
- `waypoints`: a list of intermediate points that Lucia will visit in order.
- `goal_pose`: final destination after all waypoints are completed.

> [!NOTE] 
> All coordinates (`x`, `y`, `yaw`) are expressed in the **map** frame, in meters and radians.

##### 3.2 How to find coordinates on an environmental map

If you don't know the numeric coordinates of a point:

```bash
ros2 topic echo /clicked_point
```

Then in RViz2:

1. Enable the **Publish Point** tool.
2. Click a point on the map.
3. The clicked coordinates will be printed to `/clicked_point` — you can copy them into `make_pose()`.

##### 3.3 Run the waypoint navigation loop

```bash
ros2 run lucia_navigation2 waypoint_navi_loop.py
```

- The node will:
  - Set the initial pose.
  - Send each waypoint to Nav2 sequentially.
  - Finally navigate to `goal_pose`.

Stop the node with:

```text
Ctrl+C
```
![lucia_waypoint_follow](/media/lucia_waypoint_follower.gif)


### 🟡 Patrol Mode

> Randomly patrol within a **rectangular area** on the map.

This mode repeatedly sends random goals inside a specified rectangular region, causing Lucia to patrol that area.

#### 1. Launch Lucia's motor and LiDAR

```bash
ros2 launch lucia_controller bringup.launch.py
```

#### 2. Launch Nav2 & RViz2

```bash
ros2 launch lucia_navigation2 navigation2.launch.py \
  map:=$HOME/ros2_ws/maps/map.yaml \
  params_file:=$HOME/ros2_ws/src/lucia_navigation2/param/lucia.yaml \
  use_sim_time:=false
```

**Launch arguments:**

- `map`: path to map `.yaml`
- `params_file`: Navigation2 parameter file
- `use_sim_time`: set to `false` for real robot

#### 3. Run `patrol` node (random goals in patrol area)

##### 3.1 Set `initial_pose` and `patrol area` in `patrol.py`

```py
# --- Create a BasicNavigator instance
self.navigator = BasicNavigator()

# Rectangle area (map frame) for random goals
self.xmin =  0.0
self.xmax =  5.0
self.ymin = -2.0
self.ymax =  4.0

###################################################
# --- Set initial pose
# Initial pose (map frame)
self.initial_x   = 0.0
self.initial_y   = 0.0
self.initial_yaw = 0.0  # rad
```

- `(xmin, xmax, ymin, ymax)` define a **patrol rectangle** in the map frame.
- The node will repeatedly generate random target points within this rectangle and send them to Nav2.
- `initial_x`, `initial_y`, `initial_yaw` define where Lucia starts in the map.

> [!TIP] 
> Choose the rectangle so it stays within **free space** on your map (not in walls or unknown areas).

##### 3.2 How to find coordinates on an environmental map

Same procedure as waypoint mode:

```bash
ros2 topic echo /clicked_point
```

Then:

1. Click **Publish Point** in RViz2.
2. Click several corners in the area where you want Lucia to patrol.
3. Use those coordinates to set `xmin`, `xmax`, `ymin`, `ymax`, and the `initial_*` values.

##### 3.3 Run the patrol node

```bash
ros2 run lucia_navigation2 patrol.py
```

(or the actual node name if different; update here accordingly.)

- Lucia will continuously move to random goals in the patrol area until you stop the node.

Stop with:

```text
Ctrl+C
```


### 🟣 Navigating while Mapping mode

> Build a **new map** of the environment while navigating using SLAM.

This mode is useful when you are in a new environment without a pre-made map.  
You move Lucia around while SLAM builds the map in real time.

#### 1. Launch Lucia's motor and LiDAR

```shell
ros2 launch lucia_controller bringup.launch.py
```

#### 2. Launch Navigation2

```shell
ros2 launch nav2_bringup navigation_launch.py
```

- Starts Nav2 in a configuration suitable for SLAM-based navigation.

#### 3. Launch SLAM (slam_toolbox)

```shell
ros2 launch slam_toolbox online_async_launch.py
```

- `slam_toolbox` will:
  - Subscribe to LiDAR and odometry.
  - Publish a `map` frame and `/map` topic.
  - Continuously update the 2D occupancy grid map.

#### 4. Working with SLAM

Move your robot by requesting a goal through RViz or the ROS 2 CLI, for example:

```shell
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
"{header: {stamp: {sec: 0}, frame_id: 'map'}, \
  pose: {position: {x: 0.2, y: 0.0, z: 0.0}, \
         orientation: {w: 1.0}}}"
```

- Alternatively, use a navigation plugin in RViz to click on the map and send goals.
- You should see the map update live in RViz as Lucia moves.

To save the generated map to disk:

```shell
ros2 run nav2_map_server map_saver_cli -f ~/map
```

- This will create `~/map.yaml` and an image file (`~/map.pgm` or `~/map.png`).
- You can later re-use this map for **Navigation mode** or **Waypoint/Patrol modes** by setting the `map` launch argument.

---

## 📜 License


---

## 👤 Authors

- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

---

## 📚 References

- [Nav2](https://docs.nav2.org/index.html)

### controller_server:
- [nav2_mppi_controller](https://docs.ros.org/en/iron/p/nav2_mppi_controller/)


- [(SLAM) Navigating While Mapping](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [Turtlebot3_Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes)
- [ROS 2 Navigation Tuning Guide - Nav2](https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/)
- [A Navigation System (IROS 2020)](https://youtu.be/QB7lOKp3ZDQ?si=lvRZoMHLNqqNln23)