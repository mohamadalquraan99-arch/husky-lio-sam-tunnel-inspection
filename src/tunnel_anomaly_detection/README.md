# Tunnel Anomaly Detection

This ROS 2 Humble package compares the current LIO-SAM global point cloud with
a saved baseline PCD. It publishes candidate structural-change clusters for
RViz without modifying the exploration or navigation nodes.

## Build

Place this directory in `~/husky_ws/src`, then run:

```bash
cd ~/husky_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tunnel_anomaly_detection
source install/setup.bash
```

## First validation

Run the completed baseline mission and launch:

```bash
ros2 launch tunnel_anomaly_detection anomaly_detection.launch.py
```

The default baseline path is:

```text
/home/momo1/husky_ws/inspection_data/baseline_01/lio_sam_3d/GlobalMap.pcd
```

The default `process_once: true` is intentional. Launch the detector after the
inspection map is complete; it processes one global map and then stops doing
heavy point-cloud work. This protects the simulation and LIO-SAM from avoidable
CPU load during the first validation.

Published topics:

- `/inspection/anomaly_points`
- `/inspection/aligned_current_cloud`
- `/inspection/anomaly_markers`
- `/inspection/anomaly_status`

For the baseline-against-itself validation, the expected status is `No
meaningful structural change detected`. Add PointCloud2 and MarkerArray displays
for the topics above in RViz. Use red for anomaly points.

## Dense saved-map comparison

After saving a later inspection with `/lio_sam/save_map`, compare the two PCDs:

```bash
ros2 launch tunnel_anomaly_detection anomaly_detection.launch.py \
  current_pcd:=/home/momo1/husky_ws/inspection_data/mission_02_with_box/lio_sam_3d/GlobalMap.pcd
```

When `current_pcd` is provided, no live point-cloud topic is required. This is
the preferred inspection-result mode because both maps were saved at the same
0.1 m resolution.
