# Husky LIO-SAM Tunnel Inspection

ROS 2 simulation project for 3D mapping and autonomous inspection of tunnel
networks using a Clearpath Husky A200, Velodyne-style 3D LiDAR, IMU, LIO-SAM,
AMCL, and Navigation2.

## Current status

The current development branch runs genuine LIO-SAM with LiDAR processing,
feature extraction, map optimization, IMU deskewing, and IMU preintegration.
The robot can be driven through the Gazebo tunnel network while LIO-SAM builds
a registered 3D map in RViz.

The autonomous exploration checkpoint additionally runs SLAM Toolbox scan
matching, Navigation2, and the C++ `frontier_exploration_ros2` explorer. The
robot autonomously selects distant, high-information frontiers while the live
2D occupancy map and independent LIO-SAM 3D map are generated concurrently. A
combined launch file starts the complete pipeline.

The main stability issue was an unnecessary IMU orientation transformation.
Gazebo already publishes an ENU, robot-aligned orientation, but LIO-SAM applied
`extQRPY` again and introduced a false initial roll of approximately 0.49 rad.
The saved simulation patch passes through the already-aligned orientation while
retaining accelerometer, gyroscope, orientation, deskewing, and preintegration.

The LIO-SAM launch also remaps the standard TF topics to Clearpath's namespaced
TF tree:

```text
/a200_0000/tf
/a200_0000/tf_static
```

## Capabilities

- Clearpath Husky A200 simulation in a multi-branch tunnel network
- Simulated 3D LiDAR and robot-aligned IMU
- LIO-SAM-compatible cloud adapter with `ring` and `time` fields
- Genuine LIO-SAM IMU deskewing and preintegration
- LIO-SAM feature extraction, scan-to-map optimization, and 3D mapping
- Live registered-cloud and global-map visualization in RViz
- Live 2D occupancy mapping alongside LIO-SAM
- Navigation2 planning and control on the expanding live map
- C++ nearest-frontier and coverage-backtracking tunnel exploration
- Saving generated maps as PCD files
- Conversion of PCD maps into Navigation2 occupancy maps
- AMCL localization and Navigation2 planning on a saved map
- Waypoint-based inspection mission from the verified navigation baseline
- One-command Gazebo, LIO-SAM, SLAM Toolbox, Nav2, and C++ exploration startup

## Architecture

```text
Gazebo tunnel network
        |
        +-- Husky IMU ----------------------+
        |                                    |
        +-- 3D LiDAR -> cloud adapter -------+--> LIO-SAM
                                                   |
                                                   +--> IMU preintegration
                                                   +--> feature extraction
                                                   +--> map optimization
                                                   +--> registered 3D cloud
                                                   `--> PCD map
```

Wheel odometry remains available for the Navigation2 baseline, but it is not a
replacement for IMU preintegration in the current LIO-SAM pipeline.

## Tested environment

- Ubuntu 22.04 under WSL2
- ROS 2 Humble
- Gazebo Fortress / Ignition Gazebo 6
- Clearpath ROS 2 simulator
- LIO-SAM ROS 2 branch
- GTSAM 4.1.1
- PCL and `pcl_ros`
- Navigation2

Gazebo's wall-clock sensor rates may be lower than the configured simulation
rates when the real-time factor drops, especially under WSL2.

## Repository structure

```text
husky_ws/
|-- clearpath/                         # Generated Clearpath robot configuration
|-- src/
|   |-- husky_tunnel_bringup/
|   |   |-- config/                    # LIO-SAM and Nav2 parameters, RViz configs
|   |   |-- launch/                    # Simulation, mapping, and navigation launch files
|   |   |-- maps/                      # Packaged 2D navigation map
|   |   |-- patches/                   # LIO-SAM and Clearpath simulator fixes
|   |   |-- scripts/                   # Cloud adapter, map conversion, mission tools
|   |   `-- worlds/tunnel.sdf          # Tunnel-network world
|   |-- frontier_exploration_ros2/     # C++ frontier explorer submodule
|   `-- lio_sam/                       # Upstream LIO-SAM submodule
`-- README.md
```

Generated `build/`, `install/`, `log/`, and runtime map directories are not
committed.

## Clone

Clone the repository and initialize LIO-SAM:

```bash
git clone --recurse-submodules \
  https://github.com/mohamadalquraan99-arch/husky-lio-sam-tunnel-inspection.git \
  husky_ws

cd husky_ws
```

If it was cloned without submodules:

```bash
git submodule update --init --recursive
```

## Apply the simulation compatibility changes

Apply the repository's LIO-SAM IMU-alignment patch:

```bash
git -C src/lio_sam apply \
  ../husky_tunnel_bringup/patches/lio_sam_gazebo_imu_alignment.patch
```

The modified Clearpath sensor-description files are preserved under:

```text
src/husky_tunnel_bringup/patches/clearpath_sensor_overrides/
```

On a matching ROS 2 Humble installation, install those overrides before
regenerating the Clearpath robot description:

```bash
sudo cp \
  src/husky_tunnel_bringup/patches/clearpath_sensor_overrides/*.xacro \
  /opt/ros/humble/share/clearpath_sensors_description/urdf/
```

The simulation is configured for a high-rate IMU and a 10 Hz LiDAR scan rate,
following LIO-SAM's requirement that IMU measurements arrive substantially
faster than complete LiDAR scans.

## Build

Install the ROS 2, Clearpath, LIO-SAM, GTSAM, PCL, and Navigation2 dependencies,
then build:

```bash
source /opt/ros/humble/setup.bash

cd ~/husky_ws
colcon build --symlink-install

source install/setup.bash
```

## Launch Gazebo and LIO-SAM

The combined launch starts the tunnel simulation, waits for Gazebo and the
sensors to initialize, then starts LIO-SAM and RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/husky_ws/install/setup.bash

ros2 launch husky_tunnel_bringup tunnel_lio_sam.launch.py
```

Gazebo's Teleop plugin should publish velocity commands to:

```text
/a200_0000/cmd_vel
```

Alternatively, drive using the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p speed:=0.15 \
  -p turn:=0.30 \
  -r cmd_vel:=/a200_0000/cmd_vel
```

## Verify LIO-SAM

Check that the IMU and initial attitude are available:

```bash
timeout 10s ros2 topic echo \
  /lio_sam/deskew/cloud_info \
  --once | \
grep -E \
  'imu_available|odom_available|imu_roll_init|imu_pitch_init|imu_yaw_init'
```

For a stationary, level robot, roll and pitch should be close to zero and the
mapping odometry should remain near the origin.

## Save a 3D map

While LIO-SAM is running:

```bash
ros2 service call \
  /lio_sam/save_map \
  lio_sam/srv/SaveMap \
  "{resolution: 0.0, destination: '/lio_sam_maps/tunnel_run_01'}"
```

The files are written under:

```text
~/lio_sam_maps/tunnel_run_01/
```

Use a unique destination for each run because LIO-SAM replaces an existing
destination directory.

## Important topics

| Purpose | Topic |
|---|---|
| Husky velocity command | `/a200_0000/cmd_vel` |
| Wheel odometry | `/a200_0000/platform/odom` |
| IMU | `/a200_0000/sensors/imu_0/data` |
| Raw 3D cloud | `/a200_0000/sensors/lidar3d_0/points` |
| Adapted LIO-SAM cloud | `/lio_sam/points` |
| Deskew information | `/lio_sam/deskew/cloud_info` |
| IMU odometry | `/lio_sam/odometry/imu` |
| Mapping odometry | `/lio_sam/mapping/odometry` |
| Registered cloud | `/lio_sam/mapping/cloud_registered` |
| Global map visualization | `/lio_sam/mapping/map_global` |

## Live autonomous mapping and navigation

Start the complete verified pipeline with one command:

```bash
source /opt/ros/humble/setup.bash
source ~/husky_ws/install/setup.bash

ros2 launch husky_tunnel_bringup \
  tunnel_backtracking_exploration.launch.py
```

This starts Gazebo, the Husky simulation, LiDAR and IMU bridges, LIO-SAM,
SLAM Toolbox, Navigation2, RViz, and the C++ tunnel backtracking explorer.

The active explorer is implemented in:

```text
src/husky_tunnel_bringup/src/tunnel_backtracking_explorer.cpp
```

Its configuration is:

```text
src/husky_tunnel_bringup/config/tunnel_backtracking.yaml
```

The explorer selects the nearest reachable frontier using path distance. While
the robot moves, it records physically covered map cells. When normal frontiers
are exhausted, it searches the complete occupancy map for the nearest reachable
unvisited region. Exploration is considered complete only when neither a
reachable frontier nor an uncovered region remains.

The explorer does not continuously preempt active Navigation2 goals. Navigation
uses a custom behavior tree that avoids unnecessary spin and wait recovery
actions:

```text
src/husky_tunnel_bringup/config/tunnel_no_spin_wait.xml
```

Confirm the correct implementation is running:

```bash
ros2 node list | grep tunnel_backtracking_explorer

ros2 param get /tunnel_backtracking_explorer no_progress_timeout_s
ros2 param get /tunnel_backtracking_explorer min_frontier_cells
ros2 param get /tunnel_backtracking_explorer goal_standoff_m
ros2 param get /tunnel_backtracking_explorer maximum_cost
```

Monitor exploration decisions with:

```bash
ros2 topic echo /rosout --field msg | \
grep --line-buffered -Ei \
'navigating to nearest|nearest unvisited|frontier reached|failed|complete'
```

LIO-SAM concurrently creates the registered 3D map while SLAM Toolbox publishes
the live 2D occupancy map used by Navigation2. The Gazebo IMU orientation
compatibility adjustment is stored as:

```text
src/husky_tunnel_bringup/patches/lio_sam_gazebo_imu_orientation.patch
```

After a fresh clone, apply it once before building:

```bash
git -C src/lio_sam apply \
  ../husky_tunnel_bringup/patches/lio_sam_gazebo_imu_orientation.patch
```

For debugging without automatic movement:

```bash
ros2 launch husky_tunnel_bringup tunnel_live_nav2.launch.py
```

## Navigation baseline

The repository also retains the AMCL and inspection-mission pipeline for
navigation on a saved occupancy map. The live exploration pipeline does not use
AMCL or the packaged map server; SLAM Toolbox publishes `map -> odom` while the
map grows.

## Known limitations

1. The project currently targets the simulated, level Husky sensor mounting.
2. The IMU pass-through patch assumes Gazebo publishes an ENU, robot-aligned
   orientation; physical hardware requires measured extrinsic calibration.
3. The cloud adapter is simulator-specific, and per-point timing accuracy must
   be validated before high-speed motion.
4. Long, repetitive tunnel sections remain geometrically difficult for LiDAR
   scan matching.
5. Clearpath sensor-rate overrides currently modify installed ROS description
   files and should eventually become workspace-owned configuration.
6. Frontier exploration remains sensitive to occupancy-map quality, frontier
   suppression, and repetitive tunnel geometry; complete coverage must be
   verified after each autonomous run.
7. No project-level license has been selected yet; upstream dependencies retain
   their own licenses.

## Roadmap

Completed autonomy milestone:

- Concurrent LIO-SAM 3D mapping and SLAM Toolbox 2D mapping
- Navigation2 operation on the expanding live map
- C++ frontier exploration with tuned tunnel parameters
- One-command autonomous exploration launch

Next milestones:

- Validate complete autonomous coverage of the tunnel network
- Save and evaluate the resulting 2D occupancy and LIO-SAM 3D maps
- Relaunch Navigation2 using the verified saved map
- Add repeatable autonomous inspection routes and sensor capture
- Detect, localize, and report tunnel defects or obstacles
- Validate the pipeline on physical Husky, LiDAR, and IMU hardware

## Upstream projects

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [Clearpath Robotics](https://github.com/clearpathrobotics)
- [Navigation2](https://github.com/ros-navigation/navigation2)
- [frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2)

This repository is a research and educational simulation project and is not an
official Clearpath or LIO-SAM distribution.
