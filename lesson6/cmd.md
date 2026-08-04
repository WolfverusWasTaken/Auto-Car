# Lesson 6 Commands

## Point of Lesson 6

Lesson 6 is about local planning. The global planner creates a route, but the local planner decides how fast the vehicle should drive on the next short section of that route.

The main idea is to convert obstacles and the goal point into collision points. The speed planner then uses those collision points to slow down or stop before the vehicle reaches them.

## What Needs To Be Done

1. Understand how `local_path_extractor.py` extracts a short local path from the global path.
2. Create a buffered local path corridor in `simple_collision_checker.py`.
3. Check detected object convex hulls against the buffered path.
4. Publish obstacle collision points.
5. Calculate target velocity in `simple_speed_planner.py`.
6. Add braking safety distance for the car front and obstacle buffer.
7. Project moving-object velocity onto the path direction.
8. Use collision point speed and reaction time in the target velocity calculation.
9. Add the goal point as a collision point.
10. Remove temporary debug prints and commit the finished work.

## 1. Run Lesson 6 In Simulation

Start the simulator, global planner, local planner, obstacle simulator, controller, dashboard, and RViz.

```bash
roslaunch autoware_mini_tutorial lesson6_sim.launch
```

Expected: RViz opens, the map is visible, and there are no node crashes.

Set a destination with `2D Nav Goal`.

Expected: the global path and extracted local path appear, and the vehicle starts driving.

## 2. Check Extracted Local Path

Open another terminal while Lesson 6 is running.

```bash
rostopic hz /planning/extracted_local_path
```

Expected: the topic publishes at about 10 Hz.

Inspect one local path message.

```bash
rostopic echo -n 1 /planning/extracted_local_path
```

Expected: the message contains a short list of waypoints from the global path ahead of the ego vehicle.

## 3. Create Obstacle Collision Points

Edit the collision checker.

```bash
nano lesson6/nodes/simple_collision_checker.py
```

Implement `TODO 1`.

Expected behavior:

```text
local path waypoints
-> Shapely LineString
-> buffered path corridor
-> detected object convex hull intersection
-> collision point
```

Run Lesson 6 again.

```bash
roslaunch autoware_mini_tutorial lesson6_sim.launch
```

Use RViz:

```text
2D Nav Goal
Publish Point
```

Expected: after placing an obstacle on the path, collision points are created where the object intersects the local path buffer.

Enable this RViz display:

```text
Planning / Collision points
```

Expected: colored collision point spheres appear near obstacles on the path.

## 4. Inspect Collision Points

Open another terminal while Lesson 6 is running.

```bash
rostopic echo -n 1 /planning/collision_points
```

Expected: the message contains point fields such as `x`, `y`, `z`, `vx`, `vy`, `vz`, `distance_to_stop`, and `category`.

Check the publish rate.

```bash
rostopic hz /planning/collision_points
```

Expected: collision points publish steadily while the local path is active.

## 5. Create Basic Speed Planner

Edit the speed planner.

```bash
nano lesson6/nodes/simple_speed_planner.py
```

Implement `TODO 2`.

Expected behavior:

```text
collision point distance on local path
-> target velocity from braking formula
-> all waypoint speeds capped to target velocity
-> modified local path published
```

Run Lesson 6.

```bash
roslaunch autoware_mini_tutorial lesson6_sim.launch
```

Place a destination and then place an obstacle on the path.

Expected: the target velocity decreases as the vehicle approaches the obstacle.

## 6. Add Braking Safety Distance

In `simple_speed_planner.py`, implement `TODO 3`.

The planner should subtract:

```text
distance_to_car_front
distance_to_stop
```

Run the simulation again.

```bash
roslaunch autoware_mini_tutorial lesson6_sim.launch
```

Expected: the vehicle stops before the obstacle instead of stopping when `base_link` reaches the collision point.

Check the relevant planning parameters.

```bash
rosparam get /planning/distance_to_car_front
rosparam get /planning/simple_collision_checker/braking_safety_distance_obstacle
rosparam get /planning/simple_collision_checker/braking_safety_distance_goal
```

Expected: default values come from `shared/config/planning.yaml`.

## 7. Test With Recorded Bag

Run Lesson 6 using recorded sensor data.

```bash
roslaunch autoware_mini_tutorial lesson6_bag.launch
```

Set a goal further ahead in RViz.

Expected: detected objects from the bag appear, and the local planner reacts to collision points.

## 8. Calculate Collision Point Speed

In `simple_speed_planner.py`, implement `TODO 4`.

Run the bag with tracking enabled.

```bash
roslaunch autoware_mini_tutorial lesson6_bag.launch use_tracking:=true
```

Expected: tracked objects have non-zero velocities, and projected speed should make sense compared with object movement direction.

Useful topic to inspect:

```bash
rostopic echo -n 1 /detection/final_objects
```

Expected: some detected objects contain velocity data when tracking is enabled.

## 9. Account For Moving Objects

In `simple_speed_planner.py`, implement `TODO 5`.

Run without tracking.

```bash
roslaunch autoware_mini_tutorial lesson6_bag.launch
```

Then run with tracking.

```bash
roslaunch autoware_mini_tutorial lesson6_bag.launch use_tracking:=true
```

Expected: with tracking enabled, the target velocity changes based on the speed of the blocking object.

Watch the dashboard values:

```text
Target obj dist
Target obj spd
Stopping point dist
Target velocity
```

## 10. Add Reaction Time

In `simple_speed_planner.py`, implement `TODO 6`.

Run with tracking enabled.

```bash
roslaunch autoware_mini_tutorial lesson6_bag.launch use_tracking:=true
```

Expected: the planner keeps a larger following distance when the collision point has speed.

Check the reaction time parameter.

```bash
rosparam get /planning/braking_reaction_time
```

Expected: the default value comes from `shared/config/planning.yaml`.

## 11. Add Goal Point Collision

In `simple_collision_checker.py`, implement `TODO 7`.

Run the simulation.

```bash
roslaunch autoware_mini_tutorial lesson6_sim.launch
```

Set a destination with `2D Nav Goal`.

Expected: as the vehicle approaches the destination, the goal becomes a collision point and the vehicle slows down smoothly.

The final stopping point should appear near the end of the local path.

## 12. Stress Test Obstacles

Run the simulation.

```bash
roslaunch autoware_mini_tutorial lesson6_sim.launch
```

Test these cases in RViz:

```text
place one obstacle on the path
place multiple obstacles on the path
place an obstacle near the edge of the path buffer
remove obstacles
place obstacles again
set a new goal
```

Expected: there are no exceptions, the closest or most restrictive collision point controls speed, and the vehicle stops safely.

## 13. Check Node Graph

Open the ROS graph viewer while Lesson 6 is running.

```bash
rqt_graph
```

Expected: the local planning pipeline connects like this:

```text
/planning/lanelet2_global_planner
-> /planning/global_path
-> /planning/local_path_extractor
-> /planning/extracted_local_path
-> /planning/simple_collision_checker
-> /planning/collision_points
-> /planning/simple_speed_planner
-> /planning/local_path
-> /control/pure_pursuit_follower
```

Detected objects also feed into the collision checker:

```text
/detection/final_objects
-> /planning/simple_collision_checker
```

## 14. Syntax Check

Run this after editing the Lesson 6 nodes.

```bash
python3 -m py_compile lesson6/nodes/local_path_extractor.py lesson6/nodes/simple_collision_checker.py lesson6/nodes/simple_speed_planner.py
```

Expected: no output.

## 15. Git Check

Check which files changed.

```bash
git status --short
```

Expected: changed files show.

Check the Lesson 6 changes.

```bash
git diff -- lesson6/nodes/simple_collision_checker.py lesson6/nodes/simple_speed_planner.py lesson6/cmd.md
```

Expected: Lesson 6 TODO code changes and this command file are shown.
