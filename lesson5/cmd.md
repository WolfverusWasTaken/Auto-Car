# Lesson 5 Commands

## Point of Lesson 5

Lesson 5 is about obstacle detection from lidar point clouds. The pipeline removes ground points, downsamples the remaining cloud, clusters nearby points with DBSCAN, and converts each cluster into a detected object with a centroid and convex hull.

The detected objects are published for later lessons, where the local planner uses them to slow down or stop near obstacles.

## What Needs To Be Done

1. Create the DBSCAN clusterer in `points_clusterer.py`.
2. Convert filtered `PointCloud2` messages into xyz numpy points.
3. Run clustering and skip empty point clouds.
4. Publish clustered points with `x`, `y`, `z`, and `label` fields.
5. Create `DetectedObjectArray` messages in `cluster_detector.py`.
6. Iterate through cluster labels and skip clusters smaller than `min_cluster_size`.
7. Calculate each object's centroid and 2D convex hull.
8. Publish detected objects and remove temporary debug prints.

## 1. Explore Detection Pipeline

Run Lesson 5 with your detection nodes disabled.

```bash
roslaunch autoware_mini_tutorial lesson5.launch use_detection:=false
```

Expected: RViz opens and shows the raw lidar, no-ground point cloud, and filtered point cloud. The `points_clusterer` and `cluster_detector` nodes should not appear yet.

Useful RViz displays:

```text
Sensing / Points raw center
Detection / Lidar / Center / Points no ground
Detection / Lidar / Points filtered
```

Tip: press space in the launch terminal to pause rosbag playback while inspecting points.

## 2. Check Pipeline Graph

Open the ROS graph viewer while Lesson 5 is running with detection disabled.

```bash
rqt_graph
```

Expected: the point cloud flows from the bag player through ground removal and voxel filtering.

Important connection before your TODOs:

```text
/player
-> /lidar_center/points_raw
-> /detection/lidar/center/naive_ground_removal
-> /detection/lidar/voxel_grid_filter
-> /detection/lidar/points_filtered
```

## 3. Run Lesson 5 With Detection

Run the full lesson after editing the TODOs.

```bash
roslaunch autoware_mini_tutorial lesson5.launch
```

Expected: RViz opens with no node crashes. The clusterer, detector, and detected-object visualizer should be active.

## 4. Validate Cluster Output

Open another terminal while Lesson 5 is running.

```bash
rostopic hz /detection/lidar/points_clustered
```

Expected: the clustered point cloud publishes near 10 Hz.

Then inspect one message.

```bash
rostopic echo -n 1 /detection/lidar/points_clustered
```

Expected: the message has the input point cloud header and fields for `x`, `y`, `z`, and `label`.

In RViz, enable:

```text
Detection / Lidar / Points clustered
```

Expected: different obstacle clusters appear in different colors.

## 5. Validate Detected Objects

Open another terminal while Lesson 5 is running.

```bash
rostopic echo -n 1 /detection/lidar/detected_objects
```

Expected: a `DetectedObjectArray` is printed with objects containing centroids and convex hull point lists.

Check the publish rate.

```bash
rostopic hz /detection/lidar/detected_objects
```

Expected: detected objects publish steadily while lidar data is playing.

In RViz, enable:

```text
Detection / Lidar / Lidar detections
```

Expected: blue object centroids and convex hull borders appear around detected obstacles.

## 6. Check Message Definitions

Use this if you need to confirm the custom object message fields.

```bash
rosmsg show autoware_mini/DetectedObject
```

```bash
rosmsg show autoware_mini/DetectedObjectArray
```

Expected: ROS prints the fields you need to populate in `cluster_detector.py`.

## 7. Check Detection Parameters

Review the DBSCAN and detector thresholds.

```bash
rosparam get /detection/lidar/points_clusterer/cluster_epsilon
rosparam get /detection/lidar/points_clusterer/cluster_min_samples
rosparam get /detection/lidar/cluster_detector/min_cluster_size
rosparam get /detection/output_frame
```

Expected default values come from `shared/config/detection.yaml`.

## 8. Syntax Check

Run this after editing the Lesson 5 nodes.

```bash
python3 -m py_compile lesson5/nodes/points_clusterer.py lesson5/nodes/cluster_detector.py
```

Expected: no output.

## 9. Git Check

Check which files changed.

```bash
git status --short
```

Expected: changed files show.

Check the Lesson 5 changes.

```bash
git diff -- lesson5/nodes/points_clusterer.py lesson5/nodes/cluster_detector.py lesson5/cmd.md
```

Expected: Lesson 5 TODO code changes and this command file are shown.
