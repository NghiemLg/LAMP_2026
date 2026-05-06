![LAMP-logo](https://github.com/NeBula-Autonomy/LAMP/blob/main/LAMP-logo.png)


## Build Instructions

Install [ROS](http://wiki.ros.org/ROS/Installation)

Build this package in a catkin workspace 
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TEASER_FPFH=ON
cd src
git clone git@github.com:NeBula-Autonomy/LAMP.git localizer_lamp
git clone git@github.com:NeBula-Autonomy/common_nebula_slam.git
wstool init
wstool merge localizer_lamp/install/lamp_ssh.rosinstall
wstool up
catkin build lamp
```
The rosinstall file should take care of most of the dependencies such as [GTSAM](https://github.com/borglab/gtsam) and Eigen.

### Python dependencies

For the loop closure prioritization module, install the Python dependencies with Python 3 on Ubuntu 20:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

The current `requirements.txt` is aligned to Python 3 and a modern PyTorch / PyTorch Geometric stack.

### GTSAM version

This workspace is currently using the upstream `gtsam` tag `4.2` instead of the moving `develop` branch. That avoids the TBB and compiler-compatibility issues we saw on Ubuntu 20.

## Run Instructions

### Multi-robot testing 
To run a multi-robot example with our released subterranean multi-robot [dataset](https://github.com/NeBula-Autonomy/nebula-multirobot-dataset), first download the dataset, 
then start the LAMP base-station process: 
```
roslaunch lamp turn_on_lamp_base.launch robot_namespace:=base1
```
then play the rosbag:
```
rosbag play <path-to-data>/*.bag -r1 --clock clock:=/clock --wait-for-subscribers
```
and to visualize the map, launch rviz:
```
rviz -d $(rospack find lamp)/rviz/lamp_base.rviz
```

### RViz / TF notes

The base RViz config expects:

- namespace `base1`
- fixed frame `world`
- map topic `/base1/lamp/octree_map`

If RViz reports `No TF data`, publish a static transform for the `world` frame:

```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world base1/map
```

If RViz still shows no map, add a fresh `PointCloud2` display manually and set its topic to:

```text
/base1/lamp/octree_map
```

Recommended temporary display settings for debugging:

- Fixed Frame: `world`
- Style: `Points`
- Size (Pixels): `5`
- Color Transformer: `FlatColor`

You can verify that the map topic is active with:

```bash
rostopic echo -n 1 /base1/lamp/octree_map/header
rostopic info /base1/lamp/octree_map
```

### Data Inputs


## Unit tests
To compile and run unit tests:
```bash
roscore & catkin build run_tests
``` 

To view the results of a package:
```bash
catkin_test_results build/<package_name>
``` 
Results for unit tests of packages are stored in the build/<package_name>/test_results folder.

## Publications to cite when using this code

Original LAMP paper - 2020
```
@inproceedings{ebadi2020lamp,
  title={LAMP: Large-scale autonomous mapping and positioning for exploration of perceptually-degraded subterranean environments},
  author={Ebadi, Kamak and Chang, Yun and Palieri, Matteo and Stephens, Alex and Hatteland, Alex and Heiden, Eric and Thakur, Abhishek and Funabiki, Nobuhiro and Morrell, Benjamin and Wood, Sally and others},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={80--86},
  year={2020},
  organization={IEEE}
}
```

-----------------------
# Explain repo
`localizer_lamp` is a ROS SLAM stack for multi-robot localization and mapping. Its job is to collect robot poses, scans, and loop-closure constraints, build a shared pose graph, optimize that graph, and publish/save the resulting map and trajectories.

The clean way to read it is by package role:

**Core pipeline**
- `lamp`
  - This is the main front-end coordinator.
  - It receives graph/scans/constraints from other modules, maintains the current pose graph, republishes keyed scans, and triggers map regeneration.
  - On the base-station side it also sends `pose_graph_to_optimize` to `lamp_pgo` and consumes `optimized_values` back from the optimizer.
  - Main code: [LampBase.cc](/home/nlg/catkin1_ws/src/localizer_lamp/lamp/src/LampBase.cc:1), [LampBaseStation.cc](/home/nlg/catkin1_ws/src/localizer_lamp/lamp/src/LampBaseStation.cc:1)

- `lamp_pgo`
  - This is the back-end pose graph optimizer.
  - It takes a `PoseGraph`, converts it to GTSAM/Kimera-RPGO form, runs robust optimization, rejects bad loop closures if configured, and publishes optimized graph values.
  - It can also remove loop closures, ignore specific robots, reset the solver, and export `.g2o` / map files.
  - Main code: [LampPgo.cc](/home/nlg/catkin1_ws/src/localizer_lamp/lamp_pgo/src/LampPgo.cc:1), [load_and_save_as_g2o.cc](/home/nlg/catkin1_ws/src/localizer_lamp/lamp_pgo/src/load_and_save_as_g2o.cc:1)

**Constraint / factor generation**
- `factor_handlers`
  - This package turns incoming robot data into graph factors that `lamp` can insert.
  - It includes handlers for odometry, robot pose updates, stationary detection, manual loop closures, and pose graph input.
  - Think of it as the adapter layer between ROS sensor/state topics and pose-graph edges/nodes.
  - Main files: [OdometryHandler.cc](/home/nlg/catkin1_ws/src/localizer_lamp/factor_handlers/src/OdometryHandler.cc:1), [ManualLoopClosureHandler.cc](/home/nlg/catkin1_ws/src/localizer_lamp/factor_handlers/src/ManualLoopClosureHandler.cc:1), [PoseGraphHandler.cc](/home/nlg/catkin1_ws/src/localizer_lamp/factor_handlers/src/PoseGraphHandler.cc:1)

- `loop_closure`
  - This package searches for loop closures and computes them.
  - It contains the full loop-closure pipeline: candidate generation, prioritization, queueing, geometric verification, and publication of loop-closure edges.
  - It supports several strategies, including proximity-based, laser-based, RSSI-based, and TEASER++/ICP-based validation paths.
  - Base behavior: subscribe to `pose_graph_incremental`, detect candidate matches, publish loop-closure edges.
  - Main files: [LoopClosureBase.cc](/home/nlg/catkin1_ws/src/localizer_lamp/loop_closure/src/LoopClosureBase.cc:1), [LoopGeneration.cc](/home/nlg/catkin1_ws/src/localizer_lamp/loop_closure/src/LoopGeneration.cc:1), [LoopComputation.cc](/home/nlg/catkin1_ws/src/localizer_lamp/loop_closure/src/LoopComputation.cc:1), [RssiLoopClosure.cc](/home/nlg/catkin1_ws/src/localizer_lamp/loop_closure/src/RssiLoopClosure.cc:1)

- `localizer_zero_velocity_detector`
  - Detects when a robot is stationary from IMU stability.
  - That information is used to create or gate stationary constraints, which can improve graph consistency.
  - Main files: [very_stable_genius.cpp](/home/nlg/catkin1_ws/src/localizer_lamp/localizer_zero_velocity_detector/src/very_stable_genius.cpp:1), [very_stable_genius_node.cpp](/home/nlg/catkin1_ws/src/localizer_lamp/localizer_zero_velocity_detector/src/very_stable_genius_node.cpp:1)

**Utilities / shared data structures**
- `lamp_utils`
  - Shared utility library used across almost every other package.
  - It handles pose-graph conversions, graph bookkeeping, file I/O, point-cloud helpers, common transforms, and GICP support.
  - If `lamp` and `lamp_pgo` are the main algorithm nodes, `lamp_utils` is the support library they both depend on.
  - Main files: [PoseGraphMessageConversion.cc](/home/nlg/catkin1_ws/src/localizer_lamp/lamp_utils/src/PoseGraphMessageConversion.cc:1), [PoseGraphFileIO.cc](/home/nlg/catkin1_ws/src/localizer_lamp/lamp_utils/src/PoseGraphFileIO.cc:1), [PointCloudUtils.cc](/home/nlg/catkin1_ws/src/localizer_lamp/lamp_utils/src/PointCloudUtils.cc:1)

- `pose_graph_msgs`
  - Message definitions for the SLAM graph itself.
  - This is the shared language between packages: nodes, edges, keyed scans, loop candidates, graph status, and map metadata.
  - Key messages include `PoseGraph`, `PoseGraphNode`, `PoseGraphEdge`, and `KeyedScan`.
  - Message directory: [pose_graph_msgs/msg](/home/nlg/catkin1_ws/src/localizer_lamp/pose_graph_msgs/msg)

**Visualization / inspection**
- `pose_graph_visualizer`
  - RViz-facing visualization of the graph structure.
  - It renders nodes, edges, loop closures, robot states, and artifact-related overlays as markers/interactive visuals.
  - Main files: [PoseGraphVisualizer.cc](/home/nlg/catkin1_ws/src/localizer_lamp/pose_graph_visualizer/src/PoseGraphVisualizer.cc:1), [pose_graph_visualizer.cc](/home/nlg/catkin1_ws/src/localizer_lamp/pose_graph_visualizer/src/pose_graph_visualizer.cc:1)

- `point_cloud_visualizer`
  - Builds and republishes robot or merged point-cloud maps for viewing.
  - It consumes keyed scans, pose graph, and optimized values, then publishes map clouds on visualization topics.
  - This is the package behind many `/point_cloud_visualizer/.../octree_map` topics you saw.
  - Main file: [PointCloudVisualizer.cc](/home/nlg/catkin1_ws/src/localizer_lamp/point_cloud_visualizer/src/PointCloudVisualizer.cc:1)

**Graph combination / multi-graph handling**
- `pose_graph_merger`
  - Merges a “fast” graph and a “slow” graph into one fused graph.
  - Used when combining graphs from different update sources or aligning robot-side and base-side graph views.
  - It also republishes merged poses in alternate frames.
  - Main file: [TwoPoseGraphMerge.cc](/home/nlg/catkin1_ws/src/localizer_lamp/pose_graph_merger/src/TwoPoseGraphMerge.cc:1)

**Domain-specific message packages**
- `artifact_msgs`
  - Message/service definitions for artifact detection and scoring.
  - Used when the mapping/localization stack is integrated with artifact reporting workflows.
  - Message directory: [artifact_msgs/msg](/home/nlg/catkin1_ws/src/localizer_lamp/artifact_msgs/msg)

- `silvus_msgs`
  - Message definitions for Silvus radio/network data.
  - Used by RSSI/network-aware loop-closure logic, especially in `loop_closure`.
  - Message directory: [silvus_msgs/msg](/home/nlg/catkin1_ws/src/localizer_lamp/silvus_msgs/msg)

**How they fit together**
1. `factor_handlers` and `loop_closure` generate graph constraints.
2. `lamp` assembles the current pose graph and keyed scans.
3. `lamp_pgo` optimizes that graph robustly.
4. `lamp` merges optimized results back and republishes the updated graph.
5. `point_cloud_visualizer` and `pose_graph_visualizer` publish the map and graph for RViz.
6. `lamp_utils` and `pose_graph_msgs` support all of the above.

**What the project outputs**
- Live optimized pose graph topics
- Live point-cloud map topics
- RViz marker/visualization topics
- Optional saved graph/map files such as `.g2o`, `.pcd`, and archived pose-graph data

If you want, I can do one more pass and give you the package dependency chain as a short diagram centered on `turn_on_lamp_base.launch`.

LAMP 2.0 paper - 2022 - Overall LAMP system
```
@article{chang2022lamp,
  title={LAMP 2.0: A robust multi-robot SLAM system for operation in challenging large-scale underground environments},
  author={Chang, Yun and Ebadi, Kamak and Denniston, Christopher E and Ginting, Muhammad Fadhil and Rosinol, Antoni and Reinke, Andrzej and Palieri, Matteo and Shi, Jingnan and Chatterjee, Arghya and Morrell, Benjamin and others},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={9175--9182},
  year={2022},
  publisher={IEEE}
}
```

Loop Closure Prioritization
```
@article{denniston2022loop,
  title={Loop closure prioritization for efficient and scalable multi-robot SLAM},
  author={Denniston, Christopher E and Chang, Yun and Reinke, Andrzej and Ebadi, Kamak and Sukhatme, Gaurav S and Carlone, Luca and Morrell, Benjamin and Agha-mohammadi, Ali-akbar},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={9651--9658},
  year={2022},
  publisher={IEEE}
}
```
