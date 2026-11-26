# Five-minute autonomous rover interview demo

Run the prepared six-view Foxglove layout before starting the demo. Keep it
connected to `ws://localhost:9091`. From a WSL terminal at the repository root,
run:

```bash
./scripts/interview_demo.sh
```

The runner starts Rosbridge when it is absent, launches the headless Gazebo
Humble stack in ROS domain 46, and sets a 4:45 mission-process timeout so the
full run, safe shutdown, and result check fit inside five minutes. It uses the
previously qualified local route envelope, with a maximum 1.0 m goal. Before publishing a goal, the demo
checks candidate destinations against the current terrain and obstacle maps,
using the planner's 0.35 m obstacle inflation and weighted-A* reachability
policy. It chooses the farthest reachable clear candidate between 0.75 m and
1.0 m along the rover's heading. The follower remains capped at 0.20 m/s and
the controller at 0.45 m/s with its acceleration limits. A longer 4 m goal
was blocked by the live map, so the interview profile no longer selects it
implicitly.

## What the interviewer should see

1. The RGB-D camera and LiDAR streams become live; the segmentation overlay
   classifies the observed lunar terrain.
2. The semantic and cost maps update from those sensor observations.
3. A terrain-aware path is published for the autonomous goal.
4. The rover moves only through the safety controller. The obstacle layer
   changes during motion, the planner publishes a changed path, and the
   follower stops or resumes according to its live path and obstacle checks.
5. The evaluator and mission supervisor report goal completion; the launcher
   saves the final SLAM map and shuts the simulation down cleanly.

The demo's pass is fail-closed: segmentation, semantic map, cost map, terrain
plan, changed-path replanning, integrated goal completion, evaluation, mission
lifecycle, nonzero command input and controller output, nonzero live-map
updates, and a saved map are required. A typed graph inventory and all
subsystem logs are retained under `evidence/runs/<run-id>/`.

## Verification

Two consecutive runs passed with the 1.0 m maximum safe-goal profile:

| Run | Wall time | Result |
| --- | ---: | --- |
| `20261117T195720Z-interview-demo` | 221 s | Goal, replanning, evaluation, mission, map save, and clean shutdown passed |
| `20261117T200119Z-interview-demo` | 234 s | Same live gates passed; clean shutdown |

Both runs retained the typed graph, mission/replan/evaluation logs, odometry,
and saved SLAM map. The reported obstacle-to-changed-plan latencies were about
25 s and 21 s respectively. These are two repeat trials, not a claim of formal
statistical reliability or P9/P11 acceptance.

## Scope

The fast presentation profile removes repeated one-shot ROS CLI dumps of large
image and grid topics. It retains the live subsystem and mission gates above;
it is not a P9-03 dashboard benchmark or P11-01 resource qualification. The
rover and obstacle perception are simulated; physical hardware is not
qualified. Dynamic replanning is triggered by changes in the sensor-derived
obstacle/terrain map while the rover moves. This demo does not claim prediction
or tracking of a moving obstacle. Four P10 evaluation scenarios remain
planner-only, and the separate Goal Master Plan referenced by the project was
not found.
