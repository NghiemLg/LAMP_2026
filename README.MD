# Workspace Guide

This workspace is set up for `localizer_lamp`, a ROS Noetic multi-robot SLAM
stack. The system builds pose graphs from robot odometry, point clouds, loop
closures, and optional radio/artifact constraints, then optimizes the graph and
publishes a merged map.

Current local environment assumptions:

- Ubuntu 20.04
- ROS Noetic
- Python 3
- PyTorch 2.x / PyTorch Geometric for the optional GNN loop-closure batcher
- GTSAM tag `4.2`
- Catkin workspace root: `/home/nlg/catkin1_ws`

## What The System Produces

Runtime outputs are ROS topics:

- `pose_graph_msgs/PoseGraph` on topics such as `/base1/lamp/pose_graph`,
  `/base1/lamp/pose_graph_to_optimize`, and `/base1/lamp_pgo/optimized_values`
- `sensor_msgs/PointCloud2` maps such as `/base1/lamp/octree_map`
- RViz marker topics for pose graph nodes, odometry edges, loop-closure edges,
  artifacts, and per-robot maps
- `pose_graph_msgs/KeyedScan` messages used to rebuild point-cloud maps from
  optimized poses

Saved/offline outputs can include:

- `.g2o` pose graph files from `lamp_pgo`
- `.pcd` point-cloud maps
- saved pose graph archives
- ROS bags containing pre/post optimization graph messages for analysis or GNN
  training

## Core Packages

- `localizer_lamp/lamp`: main front-end. It builds and publishes the current
  pose graph, stores keyed scans, sends graphs to the optimizer, and regenerates
  maps after optimization.
- `localizer_lamp/lamp_pgo`: pose graph optimizer. It consumes
  `pose_graph_to_optimize`, runs GTSAM/Kimera-RPGO optimization, rejects bad loop
  closures when configured, and publishes `optimized_values`.
- `localizer_lamp/loop_closure`: loop candidate generation, prioritization,
  batching, and geometric computation. Classical methods include ICP, FPFH,
  SAC-IA, TEASER++, proximity search, and RSSI-based logic.
- `localizer_lamp/factor_handlers`: converts incoming odometry, pose graph,
  stationary, and manual loop-closure data into graph factors.
- `localizer_lamp/pose_graph_msgs`: ROS message definitions for graph nodes,
  graph edges, keyed scans, loop candidates, and graph status.
- `localizer_lamp/lamp_utils`: shared graph conversion, graph bookkeeping,
  point-cloud helpers, file I/O, and transform utilities.
- `localizer_lamp/pose_graph_merger`: merges fast and slow pose graph streams.
  It does not solve unknown map alignment by itself; relative alignment must come
  from shared frames, priors, inter-robot loop closures, UWB, artifacts, or other
  cross-robot constraints.
- `localizer_lamp/point_cloud_visualizer`: rebuilds and republishes per-robot
  and merged point-cloud maps for RViz.
- `localizer_lamp/pose_graph_visualizer`: publishes graph visualization markers
  for RViz.
- `localizer_lamp/localizer_zero_velocity_detector`: detects stationary robot
  states from IMU stability.
- `localizer_lamp/artifact_msgs` and `localizer_lamp/silvus_msgs`: message
  packages for artifact and radio/RSSI workflows.

## Build

From the workspace root:

```bash
cd /home/nlg/catkin1_ws
catkin config -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_TANGENT_PREINTEGRATION=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF \
  -DBUILD_TEASER_FPFH=ON
catkin build
source devel/setup.bash
```

Install Python dependencies for the optional GNN loop-closure batcher:

```bash
cd /home/nlg/catkin1_ws/src/localizer_lamp
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

The current `requirements.txt` is intended for Python 3 and the local
PyTorch/PyG stack. If Torch or CUDA changes, update the PyG wheel source in that
file before reinstalling.

## Mission Modes

### 1. Dataset Replay / Offline Exploration Test

Use this mode first when validating the stack without live hardware.

Terminal 1:

```bash
source /home/nlg/catkin1_ws/devel/setup.bash
roslaunch lamp turn_on_lamp_base.launch robot_namespace:=base1 sim:=true
```

Terminal 2:

```bash
rosbag play <path-to-data>/*.bag -r1 --clock clock:=/clock --wait-for-subscribers
```

Terminal 3:

```bash
rviz -d $(rospack find lamp)/rviz/lamp_base.rviz
```

If RViz says `No TF data`, publish the base static transform:

```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world base1/map
```

Useful topic checks:

```bash
rostopic list | rg 'base1/.+(pose_graph|optimized_values|keyed_scans|octree_map)'
rostopic echo -n 1 /base1/lamp/octree_map/header
rostopic info /base1/lamp/octree_map
```

Expected map topic:

```text
/base1/lamp/octree_map
```

### 2. Single Robot + Base Station

Use this when one robot publishes odometry and filtered point clouds, and the
base station merges/optimizes the graph.

Base station:

```bash
roslaunch lamp turn_on_lamp_base.launch robot_namespace:=base1
```

Robot:

```bash
roslaunch lamp turn_on_lamp.launch robot_namespace:=husky1 base_namespace:=base1
```

Important robot-side inputs from `turn_on_lamp.launch`:

- point cloud: `body_filter/output` or `hvm/body_filter/output`
- odometry: `lo_frontend/odometry`, `hvm/odometry`, or `hero/odometry`
- IMU: `vn100/imu_wori_wcov`
- stationary topic: `stationary_accel`

The robot namespace must have a configured prefix in
`localizer_lamp/lamp/launch/turn_on_lamp.launch`. Examples include `husky1`,
`husky2`, `spot1`, `aquila1`, `kaist1`, `zoe1`, and `xmaxx1`.

### 3. Multi-Robot Exploration

Start one base station and one `turn_on_lamp.launch` process per robot. The
base station robot list is configured in:

```text
localizer_lamp/lamp/config/robot_names.yaml
```

Example base:

```bash
roslaunch lamp turn_on_lamp_base.launch robot_namespace:=base1
```

Example robots:

```bash
roslaunch lamp turn_on_lamp.launch robot_namespace:=husky1 base_namespace:=base1
roslaunch lamp turn_on_lamp.launch robot_namespace:=husky2 base_namespace:=base1
```

Map merging is correct only when the robot pose graphs become connected by a
shared frame or real graph constraints. Different spawn locations are fine, but
the system still needs one of:

- a shared global map/world frame
- known initial transforms
- inter-robot loop closures
- shared artifacts, UWB/range constraints, or manual loop closures

If every robot runs its own private local SLAM frame and no cross-robot
constraint ever appears, the relative map placement remains ambiguous.

## Real Hardware Checklist

Before a field run, verify these items on each robot:

- `roscore` or the networked ROS master is reachable from every robot.
- `ROS_MASTER_URI`, `ROS_IP`, and hostnames resolve correctly.
- Clock synchronization is active across machines.
- TF frames exist for `world`, `<robot>/map`, `<robot>/base_link`, and lidar.
- Odometry is publishing on the topic selected by `turn_on_lamp.launch`.
- Filtered point clouds are publishing on the remapped `~pcld` topic.
- IMU is publishing if stationary detection or IMU factors are enabled.
- Robot namespace matches one of the configured prefix rules.
- Base station has enough CPU/RAM for loop closure, PGO, and map visualization.

Minimal runtime checks:

```bash
rostopic hz /husky1/lo_frontend/odometry
rostopic hz /husky1/body_filter/output
rostopic echo -n 1 /base1/lamp/pose_graph
rostopic echo -n 1 /base1/lamp_pgo/optimized_values
```

## Loop Closure Modes

The default base launch includes:

```text
localizer_lamp/loop_closure/launch/loop_closure_modules.launch
```

That launch starts:

- loop candidate generation
- RSSI loop generation
- loop prioritization
- optional GNN loop-closure batcher
- candidate queue
- loop computation using keyed scans, ICP, FPFH, and TEASER++

The GNN batcher is not the mapper and does not process raw million-point point
clouds directly. It scores graph states or loop-closure batches using pose graph
nodes and edges. Point-cloud registration still happens in the classical
geometry path.

Current saved GNN weights:

```text
localizer_lamp/loop_closure/model/current_gnn_model.pkl
```

Runtime architecture:

```text
localizer_lamp/loop_closure/src/gnn/gnn_model.py
```

Runtime loader:

```text
localizer_lamp/loop_closure/src/loop_closure_batcher.py
```

The runtime model is constructed as:

```python
LoopClosureGNN(64, 7, 4, 1)
```

That means hidden size `64`, node feature size `7`, edge feature size `4`, and
one scalar output.

## GNN Training Workflow

Training is optional for the classical SLAM stack, but required if the GNN
batcher should be trusted in a new domain. Use representative data from similar
robots, sensors, map scale, terrain, and loop-closure failure modes.

The intended data flow is:

```text
rosbag with PoseGraph topics
  -> convert_rosbags_to_gnn_dataset.py
  -> nodes.csv / edges.csv pairs
  -> dataset.py
  -> offline_refit.py
  -> model weights
  -> loop_closure/model/current_gnn_model.pkl
```

The converter reads:

```text
/base1/lamp/pose_graph_to_optimize
/base1/lamp_pgo/optimized_values
```

and writes pre/post graph snapshots. The graph data is stored as CSV, but the
learning problem is graph learning, not plain tabular learning:

- `nodes.csv` becomes node feature matrix `x`
- `edges.csv` becomes graph connectivity `edge_index`
- edge types become `edge_attr`
- optimized graph covariance is used as the training target

Important files:

```text
localizer_lamp/loop_closure/script/convert_rosbags_to_gnn_dataset.py
localizer_lamp/loop_closure/script/dataset.py
localizer_lamp/loop_closure/script/gnn_model.py
localizer_lamp/loop_closure/script/offline_refit.py
```

Before training, update the hard-coded dataset paths in
`convert_rosbags_to_gnn_dataset.py`, `dataset.py`, and `offline_refit.py`.
The current `offline_refit.py` has `should_train = False`; set it to `True`
when intentionally training new weights.

Use a newly trained model only after checking validation error and confirming it
does not degrade loop-closure selection on held-out missions.

## Offline Post-Processing

Record pose graph optimization data for analysis or GNN training:

```bash
rosbag record -O pose_graph_opt.bag \
  /base1/lamp/pose_graph_to_optimize \
  /base1/lamp_pgo/optimized_values \
  /base1/lamp/pose_graph \
  /base1/lamp/pose_graph_incremental
```

Export or inspect `.g2o` / map outputs through:

```bash
roslaunch lamp lamp_g2o.launch robot_namespace:=base1 input_path:=<path>
```

Map throttling/downsampling is available through:

```bash
roslaunch lamp map_processing.launch robot_namespace:=base1
```

## Tests

Build and run package tests from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
source /home/nlg/catkin1_ws/devel/setup.bash
catkin build run_tests
catkin_test_results build
```

Targeted package examples:

```bash
catkin build lamp_utils lamp_pgo loop_closure pose_graph_merger --no-status
catkin build run_tests_lamp_utils run_tests_loop_closure --no-status
```

Useful test locations:

```text
localizer_lamp/lamp_utils/test
localizer_lamp/lamp/test
localizer_lamp/loop_closure/test
localizer_lamp/pose_graph_merger/test
localizer_lamp/localizer_zero_velocity_detector/test
```

## RViz Troubleshooting

The base RViz config expects:

```text
Fixed Frame: world
Map topic: /base1/lamp/octree_map
Namespace: base1
```

If the map topic exists but RViz is blank:

```bash
rostopic echo -n 1 /base1/lamp/octree_map/width
rostopic info /base1/lamp/octree_map
```

Then add a fresh RViz `PointCloud2` display with:

```text
Topic: /base1/lamp/octree_map
Style: Points
Size (Pixels): 5
Color Transformer: FlatColor
Fixed Frame: world
```

If `rostopic info /base1/lamp/octree_map` shows no RViz subscriber, the display
is not actually subscribed to the map topic.

## Quick Mental Model

```text
robot odometry + scans
  -> factor_handlers / lamp robot node
  -> PoseGraph nodes and edges
  -> base station lamp
  -> lamp_pgo optimization
  -> optimized PoseGraph
  -> map regeneration from keyed scans
  -> /base1/lamp/octree_map + RViz markers
```

The pose graph is a SLAM/factor graph, not a planner graph. Nodes are robot
poses or landmarks. Edges are measurement constraints such as odometry, priors,
loop closures, UWB/range, IMU, or artifact constraints. The optimizer estimates
the most consistent set of poses given those constraints.
