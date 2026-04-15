# UAV-UGV Navigation

ROS 2 package for cooperative UAV/UGV autonomous navigation in ROS2/Gazebo.  
The project combines:

- A ground robot (UGV) performing LiDAR-based SLAM and  autonomous navigation via Nav2
- A UAV camera feed used to build an occupancy map via OpenCV  
- A reactive waypoint service pipeline useing PRM-style path planning

## What this package contains

### Core nodes

- `waypoints_server` (`src/waypoints_server.cpp`)
  - Subscribes to `/camera_uav/image_raw`
  - Processes incoming images with OpenCV
  - Builds graph data for planning and serves waypoints through `waypoints_service` (`tutorial_interfaces/srv/GetWaypoints`)
- `waypoints_client` (`src/waypoints_client.cpp`)
  - Subscribes to `/odom` for current robot pose
  - Requests path waypoints from `waypoints_service`
  - Publishes navigation goals to `/goal_pose`
- `nav2_handler` (`src/nav2_handler.cpp`)
  - Helper node that converts `/waypoints` (`geometry_msgs/Point`) to `/goal_pose` (`PoseStamped`)
  - Included for experimentation/integration; not enabled in default sim launch

### Launch files

- `launch/launch_sim.launch.py`
  - Main simulation entrypoint
  - Starts Gazebo, UGV + UAV spawning, joystick stack, twist mux, and waypoint server/client
- `launch/online_async_launch.py`
  - Starts `slam_toolbox` async mode with project config
- `launch/localization_launch.py`
  - Nav2 localization bringup (`map_server` + `amcl`)
- `launch/navigation_launch.py`
  - Nav2 navigation stack bringup

### Assets and configuration

- Robot/UAV descriptions: `description/`
- Gazebo worlds: `worlds/`
- Nav2, SLAM, joystick, and mux configs: `config/`
- Gazebo models: `models/`

## Prerequisites

- ROS 2 (tested with a standard desktop installation)
- Gazebo ROS integration (`gazebo_ros`)
- Nav2 packages (`nav2_*`, `nav2_bringup`, `nav2_simple_commander`)
- `slam_toolbox`
- `joy` and `teleop_twist_joy`
- OpenCV development libraries
- Custom interface package providing:
  - `tutorial_interfaces/srv/GetWaypoints`

If `tutorial_interfaces` is not already available in your workspace, build/source it first.

## Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select my_bot
source install/setup.bash
```

## Running the simulation workflow

### 1) Start simulation + project nodes

```bash
ros2 launch my_bot launch_sim.launch.py
```

This starts Gazebo, spawns the UGV and UAV, and runs waypoint server/client nodes.

### 2) Start SLAM (mapping)

```bash
ros2 launch my_bot online_async_launch.py use_sim_time:=true
```

### 3) Start Nav2 navigation stack

```bash
ros2 launch my_bot navigation_launch.py use_sim_time:=true
```

### 4) (Optional) Start RViz

```bash
rviz2
```

## Package structure

```text
my_bot/
├── config/          # Nav2, SLAM, gazebo, joystick, mux params
├── description/     # UGV/UAV xacro and SDF definitions
├── includes/        # C++ image processing and planning helpers
├── launch/          # Simulation, SLAM, localization, and navigation launch files
├── models/          # Gazebo model assets
├── src/             # C++/Python nodes
└── worlds/          # Gazebo world files
```

## Notes

- Package name in ROS 2 is currently `my_bot` (see `package.xml`).
- `waypoints_client` publishes waypoints directly as `PoseStamped` goals on `/goal_pose`.
- Localization launch (`launch/localization_launch.py`) requires a `map` argument when used:

```bash
ros2 launch my_bot localization_launch.py map:=/absolute/path/to/map.yaml use_sim_time:=true
```

## License

MIT. See `LICENSE.md`.
