# Lesson 4 Commands

## Point of Lesson 4

Lesson 4 is about building a global planner. Instead of following a saved waypoint CSV, the car should use the Lanelet2 map to create a route from its current pose to a goal selected in RViz, convert that route into waypoints, and let the Lesson 3 Pure Pursuit follower drive the path.

## What Needs To Be Done

1. Add goal-position logging in `goal_callback`.
2. Create Lanelet2 traffic rules and a routing graph.
3. Find the start lanelet, goal lanelet, route, shortest path, and lane-change-free lane sequence.
4. Convert the lanelet sequence into `Waypoint` messages.
5. Publish the generated global path.
6. Clear the path when the vehicle reaches the goal.
7. Align the path end with the selected goal point.
8. Remove temporary debug prints and commit the finished work.

## 1. Run Lesson 4

Start the simulator, Lanelet2 map visualizer, global planner, Pure Pursuit follower, and RViz.

```bash
roslaunch autoware_mini_tutorial lesson4.launch
```

Expected: RViz opens, the map is visible, and the console has no error messages.

## 2. Test Goal Logging

In RViz, click the `2D Nav Goal` button and place a goal on the map.

Expected: the terminal prints a log message with the goal coordinates.

Example:

```text
/planning/global_planner - goal position (x, y, z) in map frame
```

## 3. Test Routing Output

Add a temporary debug print after the route logic.

```python
print(path_no_lane_change)
```

Then run Lesson 4 again.

```bash
roslaunch autoware_mini_tutorial lesson4.launch
```

Set the vehicle start location with `2D Pose Estimate`, then set a destination with `2D Nav Goal`.

Expected: the terminal prints the lanelet sequence for the route.

Also test an unreachable goal.

Expected: the planner logs a warning similar to:

```text
No route found to goal position
```

Remove the temporary print before final submission.

## 4. Check Published Global Path

Open another terminal while Lesson 4 is running.

```bash
rostopic echo /planning/global_path
```

Expected: after setting a valid goal, a `Path` message with waypoints is printed.

## 5. Check Node Graph

Open the ROS graph viewer.

```bash
rqt_graph
```

Expected: with `Nodes only` selected, the planner publishes `/planning/global_path`, and the Pure Pursuit follower subscribes to it.

Important connection:

```text
/planning/global_planner
-> /planning/global_path
-> /control/pure_pursuit_follower
```

## 6. Test Speed Limit

Run Lesson 4 with a lower speed limit.

```bash
roslaunch autoware_mini_tutorial lesson4.launch speed_limit:=10
```

In another terminal, echo the current velocity.

```bash
rostopic echo /localization/current_velocity
```

Expected: the vehicle speed is limited by the launch argument. The planner receives the limit in km/h, but waypoint speeds must be stored in m/s.

## 7. Test Goal Reached Behavior

Run Lesson 4.

```bash
roslaunch autoware_mini_tutorial lesson4.launch
```

Set a start position with `2D Pose Estimate`, then set a reachable goal with `2D Nav Goal`.

Expected: when the vehicle gets close to the goal, the planner logs that the goal was reached, publishes an empty path, clears the route in RViz, and the car decelerates to a stop.

## 8. Test Path End Alignment

Run Lesson 4 and place goals at different positions on a lanelet.

```bash
roslaunch autoware_mini_tutorial lesson4.launch
```

Try goals near:

```text
beginning of a lanelet
middle of a lanelet
end of a lanelet
```

Expected: the final waypoint and goal point are aligned closely, so the car stops near the selected goal instead of driving to the full lanelet endpoint.

## 9. Syntax Check

Run this after editing the global planner node.

```bash
python3 -m py_compile lesson4/nodes/lanelet2_global_planner.py
```

Expected: no output.

## 10. Git Check

Check which files changed.

```bash
git status --short
```

Expected: changed files show.

Check the Lesson 4 changes.

```bash
git diff -- lesson4/nodes/lanelet2_global_planner.py lesson4/cmd.md
```

Expected: global planner TODO code and this command file are shown.
