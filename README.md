# Husky LIO-SAM Tunnel Inspection

ROS 2 simulation project for autonomous tunnel mapping and inspection with a
Clearpath Husky A200, 3D LiDAR, IMU, LIO-SAM, AMCL, and Navigation2.

This repository preserves a **verified working baseline** before the next
development phase. The current system can generate and save a 3D tunnel map,
convert it into a 2D occupancy map, localize with AMCL, navigate through an
L-shaped tunnel, and execute a waypoint-based inspection mission.

> [!IMPORTANT]
> The current mapping pipeline uses LIO-SAM's LiDAR processing, feature
> extraction, and map optimization, but full IMU preintegration is disabled.
> Clearpath wheel odometry supplies the incremental motion estimate because the
> simulated IMU/preintegration pipeline was unstable. Improving genuine
> LiDAR-IMU fusion is the main goal of the next project phase.

## Current capabilities

- Clearpath Husky A200 simulation in an L-shaped tunnel
- Simulated 3D LiDAR and IMU
- LiDAR point-cloud adapter for LIO-SAM compatibility
- LIO-SAM feature extraction and 3D map optimization
- Wheel-odometry assistance in geometrically repetitive tunnel sections
- Saving LIO-SAM maps as PCD files
- Converting a saved 3D PCD map to a Navigation2 occupancy map
- AMCL localization on the saved 2D map
- Navigation2 planning and control through the tunnel corner
- Automatic initial localization in simulation
- Autonomous waypoint inspection mission with configurable pauses

## Verified results

- Stable Husky simulation and teleoperation
- Stable three-node LIO-SAM mapping pipeline
- Saved `GlobalMap.pcd` with approximately 96,000 points
- Valid `354 x 248` Nav2 occupancy map at `0.10 m/cell`
- AMCL localization with a connected `map -> odom -> base_link` TF tree
- Successful Navigation2 goal around the L-shaped corner
- Successful autonomous inspection mission ending near `(29, 15)`

## System architecture

### Mapping mode

```text
Clearpath 3D LiDAR
        |
        v
lio_cloud_adapter.py
  - removes invalid points
  - adds the required time field
        |
        v
LIO-SAM imageProjection
        |
        v
LIO-SAM featureExtraction
        |
        v
LIO-SAM mapOptimization --------> 3D PCD map
        ^
        |
wheel_odom_relay.py
```

The simulated LiDAR provides `ring` but no per-point `time` field. The adapter
currently assigns `time = 0.0` to every point. The tunnel is also degenerate
along its long straight axis, so wheel odometry is used as the incremental
motion estimate.

### Navigation mode

```text
Saved 3D PCD map
        |
        v
pcd_to_nav2_map.py
        |
        v
2D occupancy map + LaserScan + wheel odometry
        |
        v
AMCL + Navigation2
        |
        v
Husky autonomous inspection mission
```

Mapping mode and saved-map navigation mode are intentionally separate in the
current baseline.

## Tested environment

- Ubuntu 22.04 under WSL2
- ROS 2 Humble
- Ignition Gazebo Fortress 6.18
- Clearpath ROS 2 simulator
- Navigation2
- GTSAM 4.1.1
- PCL / `pcl_ros`

Actual wall-clock sensor rates may be lower than configured rates when Gazebo's
real-time factor drops under WSL.

## Repository structure

```text
husky_ws/
|-- .gitmodules
|-- src/
|   |-- husky_tunnel_bringup/
|   |   |-- config/
|   |   |   |-- lio_sam.yaml
|   |   |   |-- nav2_params.yaml
|   |   |   |-- tunnel_mapping.rviz
|   |   |   `-- tunnel_nav2.rviz
|   |   |-- launch/
|   |   |   |-- tunnel_sim.launch.py
|   |   |   |-- tunnel_mapping.launch.py
|   |   |   |-- tunnel_localization.launch.py
|   |   |   |-- tunnel_navigation.launch.py
|   |   |   `-- tunnel_nav2.launch.py
|   |   |-- maps/
|   |   |   |-- tunnel_nav2.pgm
|   |   |   `-- tunnel_nav2.yaml
|   |   |-- patches/
|   |   |   `-- lio_sam_planar_husky.patch
|   |   |-- scripts/
|   |   |   |-- lio_cloud_adapter.py
|   |   |   |-- wheel_odom_relay.py
|   |   |   |-- pcd_to_nav2_map.py
|   |   |   `-- inspection_mission.py
|   |   `-- worlds/
|   |       `-- tunnel.sdf
|   `-- lio_sam/                 # Git submodule
`-- maps/                         # Generated maps; ignored by Git
```

`build/`, `install/`, `log/`, and generated maps are not intended to be
committed.

## Clone and build

Clone the repository with its LIO-SAM submodule:

```bash
git clone --recurse-submodules <repository-url> husky_ws
cd husky_ws
```

If the submodule was not cloned initially:

```bash
git submodule update --init --recursive
```

Apply the saved planar simulation patch from the workspace root:

```bash
git -C src/lio_sam apply \
  ../husky_tunnel_bringup/patches/lio_sam_planar_husky.patch
```

Install the required ROS 2 Humble dependencies, GTSAM, the Clearpath simulator,
Navigation2, and PCL packages before building. Then run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The current simulator setup expects the generated Clearpath robot configuration
under `~/husky_ws/clearpath`. Packaging a fully portable robot configuration is
part of the roadmap.

## Generate a 3D map

### Terminal 1 - simulation

```bash
source /opt/ros/humble/setup.bash
source ~/husky_ws/install/setup.bash
ros2 launch husky_tunnel_bringup tunnel_sim.launch.py
```

### Terminal 2 - LIO-SAM mapping

```bash
source /opt/ros/humble/setup.bash
source ~/husky_ws/install/setup.bash
ros2 launch husky_tunnel_bringup tunnel_mapping.launch.py
```

### Terminal 3 - teleoperation

```bash
source /opt/ros/humble/setup.bash
source ~/husky_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p speed:=0.15 \
  -p turn:=0.30 \
  -r cmd_vel:=/a200_0000/cmd_vel
```

### Save the map

Use a new destination name so an earlier verified map is not overwritten:

```bash
ros2 service call /lio_sam/save_map \
  lio_sam/srv/SaveMap \
  "{resolution: 0.1, destination: '/husky_ws/maps/tunnel_map_02'}"
```

The main saved cloud is:

```text
~/husky_ws/maps/tunnel_map_02/GlobalMap.pcd
```

## View a saved 3D map

Publish the PCD as a ROS 2 point cloud:

```bash
ros2 run pcl_ros pcd_to_pointcloud \
  --ros-args \
  -p file_name:=/home/momo1/husky_ws/maps/tunnel_map_01/GlobalMap.pcd \
  -p tf_frame:=map \
  -p publishing_period_ms:=1000 \
  -r cloud_pcd:=/tunnel_3d_map
```

Open RViz, set the fixed frame to `map`, and add a `PointCloud2` display using
`/tunnel_3d_map`.

## Run saved-map navigation

### Terminal 1 - simulation

```bash
ros2 launch husky_tunnel_bringup tunnel_sim.launch.py
```

### Terminal 2 - localization, Navigation2, and RViz

```bash
ros2 launch husky_tunnel_bringup tunnel_nav2.launch.py
```

The simulation baseline initializes AMCL automatically at `(0, 0, 0)`.

## Run the inspection mission

After simulation and `tunnel_nav2.launch.py` are active:

```bash
ros2 run husky_tunnel_bringup inspection_mission.py \
  --ros-args \
  -p use_sim_time:=true \
  -p start_waypoint:=1 \
  -p waypoint_limit:=0 \
  -p inspection_pause:=3.0
```

The mission travels through the straight tunnel, turns the corner, visits the
remaining inspection points, and stops at the final waypoint. It is currently a
one-way navigation-and-pause mission; automatic return and sensor capture are
future work.

## Important topics

| Purpose | Topic |
|---|---|
| Husky velocity command | `/a200_0000/cmd_vel` |
| Wheel odometry | `/a200_0000/platform/odom` |
| IMU | `/a200_0000/sensors/imu_0/data` |
| Raw 3D point cloud | `/a200_0000/sensors/lidar3d_0/points` |
| 2D LaserScan | `/a200_0000/sensors/lidar3d_0/scan` |
| Adapted LIO-SAM cloud | `/lio_sam/points` |
| LIO-SAM mapping odometry | `/lio_sam/mapping/odometry` |
| Registered cloud | `/lio_sam/mapping/cloud_registered` |
| Global map visualization | `/lio_sam/mapping/map_global` |

Clearpath publishes robot transforms on namespaced topics:

```text
/a200_0000/tf
/a200_0000/tf_static
```

## Known limitations

1. Full LIO-SAM IMU preintegration is disabled.
2. The simulator produced a false initial roll and unstable inertial velocity.
3. The current planar patch forces LIO-SAM roll and pitch initialization to zero.
4. The LiDAR adapter inserts zero-valued point timestamps, limiting deskewing.
5. Wheel odometry supplies the incremental motion estimate used by LIO-SAM.
6. The current solution assumes a flat tunnel.
7. Mapping and saved-map Navigation2 are separate operating modes.
8. The inspection mission pauses at waypoints but does not yet capture or
   analyze inspection data.
9. Clean-machine Clearpath robot-configuration generation is not yet fully
   packaged in this repository.

## Development roadmap

### Phase 1 - preserve the working baseline

- Publish this verified version to GitHub
- Tag the working wheel-odometry-assisted baseline
- Keep the existing mapping and navigation workflows reproducible

### Phase 2 - genuine LiDAR-IMU LIO-SAM

- Verify IMU axes, gravity sign, orientation convention, and timestamps
- Verify LiDAR-to-IMU extrinsic calibration
- Generate meaningful per-point LiDAR timestamps
- Remove the forced planar roll/pitch patch
- Re-enable `lio_sam_imuPreintegration`
- Remove the wheel-odometry relay from LIO-SAM's incremental IMU odometry topic
- Compare LIO-SAM, wheel odometry, and Gazebo ground truth

### Phase 3 - improved simulation environment

- Redesign the tunnel world with stronger 3D geometric features
- Add inspection targets and realistic surface defects
- Improve simulator performance under WSL
- Rebuild and validate the 3D and 2D maps

### Phase 4 - inspection capability

- Add camera or higher-detail inspection sensors
- Capture sensor data automatically at inspection waypoints
- Detect and report tunnel defects
- Add automatic return-to-start and repeat patrols

## Baseline Git strategy

After publishing the repository, tag this working state before starting major
changes:

```bash
git tag -a baseline-wheel-odom-nav2-v1 \
  -m "Verified LIO-SAM mapping, Nav2, and inspection baseline"
git push origin baseline-wheel-odom-nav2-v1
```

Develop the IMU integration and tunnel redesign on separate branches so this
working baseline always remains recoverable.

## Acknowledgements

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [Clearpath Robotics](https://clearpathrobotics.com/)
- [ROS 2 Navigation2](https://navigation.ros.org/)

## License

No project license has been selected yet. Add an appropriate license before
accepting external contributions or redistributing third-party components.
