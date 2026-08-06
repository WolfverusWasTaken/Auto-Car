# Lesson 8 Commands

## Point of Lesson 8

Lesson 8 is about closed-loop testing in the CARLA simulator. Instead of replaying a bag, the full stack drives an ego vehicle in a simulated world where traffic, pedestrians, traffic lights, sensors, planning, and control all interact.

The final task is to prove that the stack works in the prepared demo scenario, then create three scenario files where the simplified framework fails in different ways.

## What Needs To Be Done

1. Start CARLA with the Tartu simulator map.
2. Launch the full tutorial stack with `lesson8.launch`.
3. Verify that the vehicle can drive to a manually selected RViz goal.
4. Run the prepared VSE demo scenario.
5. Check the VSE evaluation result after the run.
6. Create three different failure scenarios in VSE.
7. Save the three scenario JSON files in `lesson8/scenarios/`.
8. Fill in the three failure-case descriptions in `lesson8/README.md`.
9. Commit and push the final Lesson 8 files.

## 1. Start CARLA

Start the CARLA simulator first.

```bash
$CARLA_ROOT/CarlaUE4.sh -prefernvidia -RenderOffScreen
```

Expected: CARLA starts and waits for ROS/CARLA bridge clients. Keep this terminal open.

If rendering fails on the machine, try starting without the Nvidia flag.

```bash
$CARLA_ROOT/CarlaUE4.sh -RenderOffScreen
```

## 2. Launch Lesson 8

Open another terminal and start the full stack.

```bash
roslaunch autoware_mini_tutorial lesson8.launch
```

Expected:

```text
RViz opens
Tartu map is visible
ego vehicle appears on the map
Carla image view shows the third-person vehicle camera
no required nodes crash
```

The default launch uses CARLA ground-truth obstacle detection and traffic light detection.

```text
detector:=carla
tfl_detector:=carla
```

## 3. Drive To A Manual Goal

In RViz, place a goal with:

```text
2D Nav Goal
```

Expected: the global path appears, the local path appears, and the ego vehicle drives toward the selected goal in CARLA.

Useful dashboard checks:

```text
Target velocity
Target obj dist
Stopping point dist
```

Expected: target velocity changes as the planner reacts to the path, goal, obstacles, or stop lines.

## 4. Inspect Core Topics

Open another terminal while Lesson 8 is running.

```bash
rostopic hz /localization/current_pose
rostopic hz /planning/global_path
rostopic hz /planning/local_path
rostopic hz /control/vehicle_cmd
```

Expected: localization, planning, and control topics publish while the simulation is active.

Inspect one command message.

```bash
rostopic echo -n 1 /control/vehicle_cmd
```

Expected: the command contains speed and steering values from `pure_pursuit_follower.py`.

## 5. Optional: Test Own Perception

Use the Lesson 5 lidar obstacle detector instead of CARLA ground truth.

```bash
roslaunch autoware_mini_tutorial lesson8.launch detector:=cluster
```

Expected: simulated lidar topics are active, Lesson 5 clustering nodes run, and `/detection/final_objects` publishes tracked detected objects.

Use the Lesson 7 YOLO traffic light detector instead of CARLA ground truth.

```bash
roslaunch autoware_mini_tutorial lesson8.launch tfl_detector:=yolo
```

Expected: simulated camera topics are active, YOLO detector nodes run, and `/detection/traffic_light_status` publishes merged stop line statuses.

These options are heavier than the default setup.

## 6. Run The Demo Scenario In VSE

Keep CARLA and `lesson8.launch` running. Start Visual Scenario Editor and open the `tartu_demo` map.

When VSE asks for the agent behavior logic, select:

```text
autoware_mini/nodes/platform/carla/carla_minimal_agent.py
```

Open the prepared scenario:

```text
Scenario menu -> Open
shared/data/scenarios/tartu_demo_route_simplified.json
```

Press:

```text
Play
```

Expected:

```text
goal appears in RViz automatically
ego vehicle starts driving the demo route
NPC vehicles and pedestrians act around the ego vehicle
traffic lights change according to the scenario
```

## 7. Check Demo Scenario Result

When the VSE run finishes, check the result window.

Expected: VSE reports route completion, collisions, red light violations, and other evaluation results.

Also check for the saved result text file next to the scenario JSON.

```bash
ls -l shared/data/scenarios
```

Expected: the folder contains the demo scenario JSON and the generated result file after the VSE run.

## 8. Create Failure Scenario Folder

Create the folder for final Lesson 8 scenarios.

```bash
mkdir -p lesson8/scenarios
```

Copy the demo scenario as a starting point.

```bash
cp shared/data/scenarios/tartu_demo_route_simplified.json lesson8/scenarios/failure_case_1.json
cp shared/data/scenarios/tartu_demo_route_simplified.json lesson8/scenarios/failure_case_2.json
cp shared/data/scenarios/tartu_demo_route_simplified.json lesson8/scenarios/failure_case_3.json
```

Expected: the three editable scenario files exist.

```bash
ls -l lesson8/scenarios
```

## 9. Edit Three Failure Cases In VSE

Open each copied scenario in VSE and modify actors, routes, triggers, timing, or traffic light behavior until the stack fails in a way a careful human driver could avoid.

Possible failure directions:

```text
Failure case 1: pedestrian or vehicle appears late from occlusion
Failure case 2: another actor cuts in or blocks the lane awkwardly
Failure case 3: traffic light or intersection timing exposes planner limitations
```

Expected: each case has a different failure reason and a different proposed framework fix.

Save each edited scenario back into:

```text
lesson8/scenarios/failure_case_1.json
lesson8/scenarios/failure_case_2.json
lesson8/scenarios/failure_case_3.json
```

## 10. Validate Failure Cases

For each failure scenario:

1. Start CARLA.
2. Start Lesson 8.
3. Open the scenario in VSE.
4. Press `Play`.
5. Observe and record what fails.

Useful topics during validation:

```bash
rostopic echo -n 1 /planning/collision_points
rostopic echo -n 1 /planning/local_path
rostopic echo -n 1 /detection/final_objects
rostopic echo -n 1 /detection/traffic_light_status
```

Expected: the failure is reproducible enough to demonstrate in the practice session.

## 11. Fill In README Failure Descriptions

Edit the Lesson 8 README.

```bash
nano lesson8/README.md
```

Fill in:

```text
Failure case 1
Failure case 2
Failure case 3
```

For each case, include:

```text
what happens in the scenario
how the framework fails
what framework change would fix it
```

Expected: the `...` placeholders are replaced with real descriptions.

## 12. Final Git Check

Check which files changed.

```bash
git status --short
```

Expected: Lesson 8 README, command file, and the three scenario JSON files show.

Check the Lesson 8 changes.

```bash
git diff -- lesson8/README.md lesson8/cmd.md lesson8/scenarios
```

Expected: failure-case descriptions, this command file, and scenario JSON changes are shown.

Commit and push.

```bash
git add lesson8/README.md lesson8/cmd.md lesson8/scenarios
git commit -m "Complete lesson 8 CARLA scenarios"
git push
```
