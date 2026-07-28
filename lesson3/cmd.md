# Lesson 3 Commands

## 1. Create Waypoints First

Lesson 3 uses the waypoint file created in Lesson 2. Run this first if the waypoint CSV does not exist yet.

```bash
roslaunch autoware_mini_tutorial lesson2.launch
```

Expected: RViz opens and a waypoint file is created from the Lesson 2 recording.

## 2. Run Lesson 3

Start the path loader, pure pursuit follower, bicycle simulation, and RViz.

```bash
roslaunch autoware_mini_tutorial lesson3.launch
```

Expected: RViz opens and the recorded path is visible.

## 3. Check Vehicle Commands

Open another terminal while Lesson 3 is running.

```bash
rostopic echo /control/vehicle_cmd
```

Expected: steering angle, speed, and acceleration commands are printed.

## 4. Check Node Graph

Open the ROS graph viewer.

```bash
rqt_graph
```

Expected: with `Nodes only` selected, you should see the loop:

```text
/control/pure_pursuit_follower
-> /control/vehicle_cmd
-> /vehicle/bicycle_simulation
-> /localization/current_pose
-> /control/pure_pursuit_follower
```

## 5. Check Generated Waypoint File

Use this if Lesson 3 cannot load a path.

```bash
ls $(rospack find autoware_mini)/data/trajectories/waypoints_1m.csv
```

Expected: the waypoint CSV path is printed.

## 6. Change Lookahead Distance

Edit the control configuration if you want to test different Pure Pursuit behavior.

```bash
nano shared/config/control.yaml
```

Expected: changing `lookahead_distance` changes how sharply or smoothly the car follows the path.

## 7. Syntax Check

Run this after editing the follower node.

```bash
python3 -m py_compile lesson3/nodes/pure_pursuit_follower.py
```

Expected: no output.

## 8. Git Check

Check which files changed.

```bash
git status --short
```

Expected: changed files show.

Check the Lesson 3 changes.

```bash
git diff -- lesson3/nodes/pure_pursuit_follower.py lesson3/cmd.md
```

Expected: pure pursuit TODO code and this command file are shown.
