# Husky LIO-SAM Tunnel Inspection

ROS 2 Humble simulation project for autonomous tunnel inspection using a
Clearpath Husky A200, 3D LiDAR, IMU, LIO-SAM, SLAM Toolbox, Navigation2,
frontier/coverage exploration, automatic return-to-home, mission data saving,
and repeat-mission point-cloud anomaly detection.

## Current system

The integrated mission now performs the complete inspection sequence:

1. Start the Gazebo tunnel, Husky sensors, LIO-SAM, SLAM Toolbox, and Nav2.
2. Explore reachable frontiers while recording physical coverage.
3. Search meaningful uncovered regions after map frontiers are exhausted.
4. Confirm completion and navigate back to the stored start pose.
5. Save the 2D occupancy map, SLAM pose graph, and dense LIO-SAM PCD map.
6. During inspection missions, align the saved cloud with a baseline and mark
   possible structural changes in RViz.

The first open staging room is treated as already covered by the explorer. It
remains navigable and is not inserted as an obstacle into the Nav2 costmap.

## Architecture

```text
Gazebo Husky + tunnel
        |
        +-- IMU + 3D LiDAR --> LIO-SAM ------------------> dense 3D map
        |
        +-- 3D cloud --> 2D scan --> SLAM Toolbox ------> occupancy grid
                                                      |
                                                      v
Explorer --> Nav2 global/local planning --> Husky motion
    |                                      |
    +-- coverage memory                    +-- return home
                                                      |
                                                      v
Mission manager --> save 2D/3D maps + pose graph --> anomaly detector
                                                      |
                                                      v
                                           RViz change cloud and markers
```

LIO-SAM supplies three-dimensional inspection geometry. SLAM Toolbox supplies
the two-dimensional occupancy representation used by the explorer and Nav2.

## Main packages

| Package | Purpose |
|---|---|
| `husky_tunnel_bringup` | Simulation, mapping, navigation, backtracking explorer, RViz, and integrated launch files |
| `tunnel_mission_manager` | Mission completion handling and automatic 2D map, 3D map, pose-graph, and metadata saving |
| `tunnel_anomaly_detection` | Baseline/current PCD alignment, change extraction, clustering, and RViz markers |
| `lio_sam` | Upstream LIO-SAM ROS 2 submodule |

## Implemented algorithms

- IMU preintegration, LiDAR deskewing, feature extraction, scan-to-map
  optimization, factor-graph smoothing, and ICP loop closure through LIO-SAM
- SLAM Toolbox scan matching, pose-graph optimization, and occupancy mapping
- NavFn/Dijkstra global planning, DWB local trajectory rollout, layered
  costmaps, inflation, and velocity smoothing
- Reachability BFS, frontier connected components, centroid goals, standoff
  goals, temporary spatial blacklists, coverage-grid memory, and uncovered-region
  flood fill
- Repeated-empty-search completion confirmation and stored-pose return home
- NaN removal, voxel filtering, ICP registration, k-d-tree nearest-neighbour
  differencing, Euclidean clustering, and bounding-box marker generation

## Tested environment

- Ubuntu 22.04 under WSL2
- ROS 2 Humble
- Gazebo Fortress / Ignition Gazebo 6
- Clearpath ROS 2 simulator
- LIO-SAM ROS 2 branch, GTSAM 4.1.1, and PCL
- SLAM Toolbox and Navigation2

## Mapping results

### LIO-SAM 3D map

![LIO-SAM 3D tunnel map](docs/images/lio_sam_3d_map.png)

### SLAM Toolbox 2D map

![SLAM Toolbox 2D tunnel map](docs/images/slam_toolbox_2d_map.png)

## Clone and build

```bash
git clone --recurse-submodules \
  https://github.com/mohamadalquraan99-arch/husky-lio-sam-tunnel-inspection.git \
  husky_ws

cd husky_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If the repository was cloned without its submodule:

```bash
git submodule update --init --recursive
```

The repository also provides a helper for installing the Ubuntu/ROS
dependencies used by the tested simulation:

```bash
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

Apply the simulation IMU-alignment patch when using the matching LIO-SAM
submodule version:

```bash
git -C src/lio_sam apply \
  ../husky_tunnel_bringup/patches/lio_sam_gazebo_imu_alignment.patch
```

## Run a baseline mission

Use a unique baseline identifier. The mission manager refuses to overwrite an
existing `GlobalMap.pcd` unless overwrite protection is explicitly disabled.

```bash
source /opt/ros/humble/setup.bash
source ~/husky_ws/install/setup.bash

ros2 launch husky_tunnel_bringup tunnel_inspection_system.launch.py \
  mission_mode:=baseline \
  baseline_id:=baseline_01 \
  data_root:=/home/momo1/husky_ws/inspection_data
```

After exploration and return home, the baseline data are saved automatically.

## Run an inspection mission

Place a new object or simulated obstruction in the tunnel, then use a new
mission identifier:

```bash
ros2 launch husky_tunnel_bringup tunnel_inspection_system.launch.py \
  mission_mode:=inspection \
  baseline_id:=baseline_01 \
  mission_id:=mission_03 \
  data_root:=/home/momo1/husky_ws/inspection_data
```

When the robot reaches home, the manager saves the new mission and triggers the
anomaly detector with the saved `GlobalMap.pcd` path.

## Saved mission data

Each baseline or inspection directory contains:

```text
inspection_data/<mission_id>/
|-- tunnel_2d_map.pgm
|-- tunnel_2d_map.yaml
|-- tunnel_posegraph.posegraph
|-- tunnel_posegraph.data
|-- mission_metadata.json
`-- lio_sam_3d/
    |-- GlobalMap.pcd
    |-- CornerMap.pcd
    |-- SurfMap.pcd
    |-- trajectory.pcd
    `-- transformations.pcd
```

Mission datasets are runtime outputs and are intentionally excluded from Git.

## RViz outputs

| Purpose | Topic |
|---|---|
| LIO-SAM global map | `/lio_sam/mapping/map_global` |
| Registered LiDAR cloud | `/lio_sam/mapping/cloud_registered` |
| SLAM occupancy map | `/map` |
| Robot coverage | `/exploration/visited_area` |
| Explorer mission state | `/exploration/mission_status` |
| System mission state | `/inspection/system_status` |
| Detected change cloud | `/inspection/anomaly_points` |
| Change bounding boxes and labels | `/inspection/anomaly_markers` |

The exact anomaly output topic names can be verified with:

```bash
ros2 topic list | grep -E 'inspection|anomaly'
```

## Explorer behavior

The custom C++ explorer accepts known free cells only, verifies Nav2-costmap
reachability, groups frontier cells into connected regions, selects a reachable
representative, and backs the goal away from walls. Successful and failed goals
are remembered temporarily to prevent immediate re-selection.

After no frontier remains, a persistent coverage grid identifies reachable but
unvisited regions. Twenty consecutive empty planning checks are required before
the mission is declared complete. The robot then navigates to the first valid
`map -> base_link` pose recorded at startup.

## Anomaly-detection behavior

The detector downsamples the baseline and current maps with the same voxel size,
optionally aligns them using ICP, and searches the baseline k-d tree for the
nearest neighbour of every current-map point. Points farther than the configured
distance threshold become candidates. Euclidean clustering rejects isolated
noise, and accepted clusters are published as a colored point cloud and RViz
bounding-box markers.

This comparison is directional: it emphasizes new or displaced material in the
current inspection. A future bidirectional comparison is required to classify
removed material independently.

## Repository structure

```text
husky_ws/
|-- clearpath/                         # Clearpath robot configuration
|-- src/
|   |-- husky_tunnel_bringup/          # Integrated simulation and autonomy
|   |-- tunnel_mission_manager/        # Mission saving/orchestration
|   |-- tunnel_anomaly_detection/      # Point-cloud comparison
|   `-- lio_sam/                       # Upstream Git submodule
`-- README.md
```

Generated `build/`, `install/`, `log/`, mission data, PCD files, ROS logs,
editor caches, and development backup files are not committed.

## Known limitations

1. The system is validated in simulation with a level, robot-aligned IMU.
2. Repetitive tunnel geometry can weaken LiDAR scan matching and ICP alignment.
3. Coverage is binary and circular around the robot; it does not explicitly
   model LiDAR visibility or inspection quality.
4. Anomaly thresholds are fixed and do not yet adapt to range-dependent noise.
5. Current-to-baseline comparison is strongest for additions, not removals.
6. The staging-area exclusion is configured for the current map coordinates.
7. WSL2 performance can reduce the Gazebo real-time factor and sensor rates.

## License note

No project-level license has been selected yet. Upstream dependencies retain
their original licenses.
