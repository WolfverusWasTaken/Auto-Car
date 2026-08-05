# Lesson 7 Commands

## Point of Lesson 7

Lesson 7 is about camera-based traffic light detection. The map tells us where traffic lights should appear in the camera image, YOLO detects visible traffic lights, and the detector matches those two sources to publish stop line statuses.

The local planner then uses red and yellow stop line statuses as collision points, so the car stops before the traffic light stop line and continues when the light is green.

## What Needs To Be Done

1. Find traffic light stop lines that intersect the local path.
2. Transform traffic light map coordinates into the camera frame.
3. Project traffic light coordinates into camera pixel ROIs.
4. Run YOLO on camera images.
5. Match YOLO detections with map ROIs using IOU.
6. Publish `StopLineStatusArray` messages from each camera detector.
7. Store merged traffic light statuses in `simple_collision_checker.py`.
8. Add stop line collision points for red and yellow lights.
9. Validate that the car stops at red/yellow lights and continues on green.
10. Remove temporary debug prints and commit the finished work.

## 1. Run Lesson 7 Baseline

Start the full Lesson 7 stack.

```bash
roslaunch autoware_mini_tutorial lesson7.launch
```

Expected: RViz opens, the map is visible, camera image panels are visible, and there are no node crashes.

Set a destination past the traffic light stop line with `2D Nav Goal`.

Expected before implementing all TODOs: the car may drive through the light because the planner is not reacting to traffic light statuses yet.

## 2. Check Detector Nodes

Open another terminal while Lesson 7 is running.

```bash
rosnode info /detection/camera1/yolo_traffic_light_detector
```

Expected: the camera 1 detector publishes and subscribes to these topics.

```text
Publishes:
/detection/camera1/traffic_light_roi
/detection/camera1/traffic_light_status

Subscribes:
/camera_fl/camera_info
/camera_fl/image_raw
/planning/local_path
```

Check the second camera too.

```bash
rosnode info /detection/camera2/yolo_traffic_light_detector
```

Expected: the camera 2 detector uses `/camera_fr/camera_info` and `/camera_fr/image_raw`.

## 3. Find Stop Lines On The Path

Edit the detector.

```bash
nano lesson7/nodes/yolo_traffic_light_detector.py
```

Implement `TODO 1` in `local_path_callback`.

Expected behavior:

```text
local path waypoints
-> Shapely LineString
-> check traffic light stop lines
-> store stop line ids that intersect the path
```

Temporary debug print:

```python
print("stop_line_ids_on_path:", stop_line_ids_on_path)
```

Run Lesson 7 again.

```bash
roslaunch autoware_mini_tutorial lesson7.launch
```

Set a destination past the traffic light stop line.

Expected: while the stop line is ahead on the local path, the detector prints something like:

```text
stop_line_ids_on_path: [5003023]
```

## 4. Project Map ROIs Into The Camera

In `yolo_traffic_light_detector.py`, implement `TODO 2` and `TODO 3`.

Expected behavior:

```text
camera image timestamp
-> lookup transform from local path/map frame to camera frame
-> transform traffic light corners
-> project 3D camera points into pixels
-> create map ROIs
```

Temporary debug print after calculating ROIs:

```python
print("map_rois:", map_rois)
```

Run Lesson 7.

```bash
roslaunch autoware_mini_tutorial lesson7.launch
```

Set a destination past the traffic light stop line.

Expected: while traffic lights are visible, one or both camera detector nodes print ROIs like:

```text
map_rois: [[5003023, 2000160, 2006, 2063, 1210, 1289]]
```

In RViz camera panels, expected: gray boxes labelled `missing 0.00` appear around traffic lights. This means map ROIs exist, but YOLO matching is not done yet.

## 5. Run YOLO And Match Detections

In `yolo_traffic_light_detector.py`, implement both `TODO 4` sections:

```text
camera_image_callback:
YOLO predict image
-> discard classes >= 4
-> match map ROIs with YOLO ROIs
-> publish statuses

match_map_and_yolo_rois:
map ROI
-> calculate IOU with each YOLO ROI
-> keep best match above threshold
-> convert class to stop line status
```

Run Lesson 7.

```bash
roslaunch autoware_mini_tutorial lesson7.launch
```

Set a destination past the traffic light stop line.

Expected in RViz camera panels: colored boxes appear around traffic lights.

```text
large box = map ROI
small box = YOLO detection
label = detected state and score
```

Example labels:

```text
green 0.92
red 0.87
yellow 0.80
missing 0.00
```

## 6. Check Traffic Light Status Topic

Open another terminal while Lesson 7 is running.

```bash
rostopic echo /detection/traffic_light_status | grep status_text
```

Expected: merged statuses from the two cameras are printed.

```text
status_text: "green"
status_text: "red"
```

Inspect a full message if needed.

```bash
rostopic echo -n 1 /detection/traffic_light_status
```

Expected: a `StopLineStatusArray` with stop line ids, traffic light ids, status values, and status text.

Check message definitions if needed.

```bash
rosmsg show autoware_mini/StopLineStatus
rosmsg show autoware_mini/StopLineStatusArray
```

Expected: ROS prints the message fields used by the detector and merger.

## 7. Add Stop Line Collision Points

Edit the collision checker from Lesson 6.

```bash
nano lesson6/nodes/simple_collision_checker.py
```

Implement `TODO 8`.

Expected behavior:

```text
read braking_safety_distance_stopline
-> load lanelet2 map
-> extract traffic light stop lines
-> subscribe to /detection/traffic_light_status
-> store statuses by stop_line_id
```

Implement `TODO 9`.

Expected behavior:

```text
if stop line status is STOP
and stop line intersects local path
-> add collision point at intersection
-> category 2
-> zero velocity
```

## 8. Validate Planner Reaction

Run Lesson 7 after detector and collision checker TODOs are complete.

```bash
roslaunch autoware_mini_tutorial lesson7.launch
```

Set a destination past the traffic light stop line.

Expected when the light is red or yellow:

```text
/detection/traffic_light_status publishes STOP status
simple_collision_checker creates category 2 collision point
simple_speed_planner slows the car
car stops before the stop line
```

Expected when the light is green:

```text
stop line collision point disappears
target speed increases
car continues through the intersection
```

Useful dashboard checks:

```text
Target obj dist
Target obj spd
```

Expected: `Target obj dist` descends smoothly as the car approaches the stop line, and `Target obj spd` goes to zero for a red/yellow light.

## 9. Inspect Collision Points

Open another terminal while Lesson 7 is running.

```bash
rostopic echo -n 1 /planning/collision_points
```

Expected while stopped for a red/yellow light: a collision point exists with category `2`.

Check the publish rate.

```bash
rostopic hz /planning/collision_points
```

Expected: collision points publish steadily while the local path is active.

In RViz, enable:

```text
Planning / Collision points
```

Expected: stop line collision points are visible while the light requires stopping.

## 10. Check Parameters

Check detector parameters.

```bash
rosparam get /detection/camera1/yolo_traffic_light_detector/iou_threshold
rosparam get /detection/camera1/yolo_traffic_light_detector/min_roi_width
rosparam get /detection/camera1/yolo_traffic_light_detector/camera_delay_compensation
```

Expected: default values come from `shared/config/detection.yaml`.

Check planner stop line parameter.

```bash
rosparam get /planning/simple_collision_checker/braking_safety_distance_stopline
```

Expected: default value comes from `shared/config/planning.yaml`.

## 11. Syntax Check

Run this after editing the Lesson 7 detector and Lesson 6 collision checker.

```bash
python3 -m py_compile lesson7/nodes/yolo_traffic_light_detector.py lesson6/nodes/simple_collision_checker.py
```

Expected: no output.

## 12. Git Check

Remove temporary debug prints before committing.

Check which files changed.

```bash
git status --short
```

Expected: changed files show.

Check the Lesson 7 changes.

```bash
git diff -- lesson7/nodes/yolo_traffic_light_detector.py lesson6/nodes/simple_collision_checker.py lesson7/cmd.md
```

Expected: Lesson 7 TODO code changes and this command file are shown.

Commit and push.

```bash
git add lesson7/nodes/yolo_traffic_light_detector.py lesson6/nodes/simple_collision_checker.py lesson7/cmd.md
git commit -m "Complete lesson 7 traffic light detection"
git push
```

