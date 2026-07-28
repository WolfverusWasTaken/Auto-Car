# Lesson 2 Commands

## 1. Check Bag File

Make sure the rosbag exists.

```bash
ls shared/data/bags/2023-05-25-14-21-10_sensors_Raekoda.bag
```

Expected: file path is printed.

## 2. Run Lesson 2

Start rosbag, localizer, waypoint saver, and RViz.

```bash
roslaunch autoware_mini_tutorial lesson2.launch
```

Expected: RViz opens and terminal prints latitude/longitude values.

Example:

```text
58.37951071076978 26.72648964125517
58.37951153102539 26.726488117933275
```

## 3. Check Current Pose

Open another terminal.

```bash
rostopic echo /localization/current_pose
```

Expected: pose messages in `map` frame.

Example:

```yaml
header:
  frame_id: "map"
pose:
  position:
    x: -13.349363245186396
    y: -622.8975134063512
    z: 36.22197324112058
  orientation:
    z: 0.9080059543110942
    w: 0.4189572614666073
```

## 4. Check Current Velocity

```bash
rostopic echo /localization/current_velocity
```

Expected: velocity messages in `base_link` frame.

Example:

```yaml
header:
  frame_id: "base_link"
twist:
  linear:
    x: 9.374128787160322
```

## 5. Check TF

```bash
rosrun tf tf_echo map base_link
```

Expected: translation and rotation update while the bag plays.

Example:

```text
At time ...
- Translation: [x, y, z]
- Rotation: in Quaternion [x, y, z, w]
```

## 6. Check RViz

```bash
roslaunch autoware_mini_tutorial lesson2.launch
```

Expected:

```text
Red arrow = current_pose
Small arrows = saved waypoints
White numbers = speed in km/h
```

## 7. Check Waypoint CSV

```bash
ls $(rospack find autoware_mini)/data/trajectories/waypoints_1m.csv
```

Expected: CSV file exists.

```bash
head $(rospack find autoware_mini)/data/trajectories/waypoints_1m.csv
```

Expected columns:

```text
x,y,z,yaw,velocity,change_flag,steering_flag,accel_flag,stop_flag,event_flag
```

## 8. Run With Custom Arguments

```bash
roslaunch autoware_mini_tutorial lesson2.launch interval:=5 waypoints_file:=waypoints_5m.csv
```

Expected: new waypoint file is created.

```bash
ls $(rospack find autoware_mini)/data/trajectories/waypoints_5m.csv
```

## 9. Final Cleanup

Remove or comment this debug line before final submission.

```python
print(msg.latitude, msg.longitude)
```

Expected: node still publishes pose, velocity, and TF without printing coordinates.

## 10. Syntax Check

```bash
python3 -m py_compile lesson2/nodes/localizer.py
```

Expected: no output.

## 11. Git Check

```bash
git status --short
```

Expected: changed files show.

```bash
git diff -- lesson2/nodes/localizer.py lesson2/cmd.md
```

Expected: localizer TODO code and this command file are shown.
