[< Previous lesson](../lesson7/) -- [**Main Readme**](../README.md)

# Lesson 8 - Testing in the CARLA simulator

In this final lesson, you will run the whole framework from the previous lessons in closed loop inside the CARLA simulator: the simulated world reacts to your vehicle, and your vehicle must react to the world.

Two tools are used for the closed-loop validation:
* [**CARLA**](https://carla.org/) - an open-source autonomous driving simulator. It renders the world via provided map files (and we will use our own Tartu map), simulates the physics and the sensors (lidar, cameras), and feeds them to your nodes through ROS topics.
* **Visual Scenario Editor (VSE)** - a graphical tool for creating and re-playing driving scenarios in CARLA: NPC vehicles and pedestrians with routes and triggers, traffic light sequences and weather. See the [VSE repository](https://github.com/UT-ADL/visual-scenario-editor) and [how to use the editor](https://github.com/UT-ADL/visual-scenario-editor/blob/main/tutorial.md).

You will first verify that your framework can drive in CARLA, then run it through a prepared VSE scenario, and finally design scenarios yourself where your framework fails.

### Expected outcome
* Understanding how the full autonomous driving stack behaves in a closed-loop simulation
* Exploring the limits of the framework you built


## 1. Run your stack in CARLA

The launch file [lesson8.launch](launch/lesson8.launch) connects your nodes from the previous lessons to CARLA. There is no bag playback: the localization comes from the simulator, and the vehicle commands from your `pure_pursuit_follower` steer the car in the simulation.

By default the detected objects and traffic light statuses come from the simulator's ground truth instead of your perception nodes - simulating the lidar and the cameras is very heavy, and running the perception pipeline on them can slow the simulation down to a crawl. Your planner and controller are still the ones driving. If your machine can afford it, you can enable your own perception with `detector:=cluster` (lesson 5 nodes on the simulated lidar) and/or `tfl_detector:=yolo` (lesson 7 nodes on the simulated cameras).

##### Instructions
1. Start the CARLA simulator:
    ```
    $CARLA_ROOT/CarlaUE4.sh -prefernvidia -RenderOffScreen
    ```
2. In another terminal, launch your stack:
    ```
    roslaunch autoware_mini_tutorial lesson8.launch
    ```

##### Validation
* RViz opens with the Tartu map and the ego vehicle placed in the simulated city
* The `Carla image view` panel shows the third-person view of the ego vehicle in the simulated world
* Place a goal on the map - the vehicle drives to it


## 2. Run the demo scenario

A driving scenario adds actors to the otherwise empty world: NPC vehicles and pedestrians that spawn, move and react when triggered, and traffic lights that switch according to the scenario triggers. You will run the prepared demo lap scenario and see whether your framework survives traffic.

When your stack is running, VSE automatically detects your ego vehicle and hands the driving over to it - the scenario provides the destination, the other actors and the evaluation.

##### Instructions
1. With `lesson8.launch` running, start VSE and open the `tartu_demo` map. When VSE first launches, it will ask to select the agent's behavior logic. Navigate to `autoware_mini/nodes/platform/carla/` and select `carla_minimal_agent.py`.
2. Open the scenario (`Scenario` menu -> `Open`): `shared/scenarios/tartu_demo_route_simplified.json` from the tutorial folder
3. Press **Play**. Note: if your machine has less than 10 Gb VRAM slowdowns are expected.

##### Validation
* The goal appears in RViz automatically and the vehicle starts driving the demo lap
* NPC vehicles and pedestrians act out the scenario around the ego vehicle
* When the run ends, VSE shows a results window scoring the drive (collisions, red light violations, route completion); the same results are also saved as a text file next to the scenario JSON


## 3. Create three failure cases

Your framework from the previous lessons is a simplified one. Remember all limitations that were discussed through the lessons. In this final task you will demonstrate these limits: create three scenarios where your framework fails.

##### Instructions
1. Copy `tartu_demo_route_simplified.json` (e.g. to `failure_case_1.json`) and modify it in VSE - move, add, retime or reroute actors and triggers until your stack demonstrably fails, while a careful human driver would still manage
2. For every failure case, think of a specific change to the framework that would fix it. You do not need implement the fix. The three cases should have three different proposed fixes.
3. Create a `lesson8/scenarios/` folder in your repository and commit the three scenario JSONs there
4. Fill in the three descriptions below: what happens in the scenario, how your framework fails, and what change to the framework would fix it. Add screenshots if needed.
5. Commit and push everything, and be ready to demonstrate your failure cases at the practice session

##### Failure case 1
Lane cutting.

Target criterion: `CollisionTest`, with possible `CheckKeepLane` or `InRouteTest` side effects depending on the ego reaction.

In `lesson8/scenarios/failure_case_1.json`, another vehicle cuts into the ego vehicle's lane close to the ego vehicle. A careful human driver would notice the neighboring vehicle's motion, anticipate the merge, and slow down or create space before the vehicle enters the lane.

The framework fails because it has limited behavior prediction for nearby vehicles and mostly reacts after an object becomes a direct collision point on the local path. When the other vehicle cuts in late, the ego vehicle may brake too late, swerve awkwardly, leave the planned route, or collide.

A fix would be to add lane-change and cut-in prediction for nearby vehicles. The planner should estimate whether adjacent vehicles are likely to enter the ego lane and adjust speed before the cut-in becomes an emergency.

##### Failure case 2
Late pedestrian crossing.

Target criterion: `CollisionTest`.

In `lesson8/scenarios/failure_case_2.json`, all traffic lights are set to green and all NPC vehicles are removed, leaving the original pedestrians from the demo scenario at their original valid spawn positions. The pedestrian triggers are widened or retimed so pedestrian activity is easier to observe near the ego route.

The framework fails because it reacts mostly to detected collision points on the local path and has very limited detection pedestrians coming from the corner. When pedestrians enter or approach the lane near the ego route, the vehicle may brake too late or collide with one of them.

A fix would be to add pedestrian trajectory prediction and increasing the buffer rate/range of angle of the sensor. The planner should slow down earlier near pedestrians and crossings when a pedestrian could reasonably enter the ego lane.

##### Failure case 3
Blind spot jaywalking.

Target criterion: `CollisionTest`.

In `lesson8/scenarios/failure_case_3.json`, a pedestrian enters the ego lane from a blind spot or occluded area when the ego vehicle is already close. A careful human driver would slow down near the occlusion, cover the brake, and leave extra room for a pedestrian who might step into the road.

The framework fails because it only reacts to currently detected objects and has limited reasoning about occlusions or hidden pedestrians. If the pedestrian appears suddenly from the blind spot, the ego vehicle may not have enough distance to brake smoothly and may collide or stop too late.

A fix would be to add occlusion-aware planning and pedestrian risk prediction. The planner should slow down near blind spots, parked vehicles, crossings, and other areas where pedestrians may appear suddenly.
