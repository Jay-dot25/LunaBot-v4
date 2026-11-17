# Phase D visual navigation checks

From Ubuntu/WSL, run:

```bash
cd ~/lunabot-v4
bash ./launch-d-view
```

This starts headless Gazebo and initializes RViz while motion remains startup
inhibited. It arms only after manual-mode diagnostics finish. RViz uses
simulation time and a project-local Mesa sampler-binding workaround.
No installed ROS shader files are modified. The upstream sampler issue is
tracked at https://github.com/ros2/rviz/issues/463.
Use RViz's 2D Goal Pose tool on connected mapped free space. Grey unknown
space and disconnected regions cannot be used as navigable routes. Close
RViz normally or press Ctrl+C in the launcher terminal to stop the stack.
The launcher prints its unique evidence directory, containing navigation.log,
control.log, rviz.log, and last_run.log. Startup PASS is separate from goal
completion, which requires GOAL_REACHED and zero controller output.

## Corrected Phase D profile

The shared baseline model is unchanged. Phase D derives an ignored model at
`.build/phase_d_navigation/model.sdf`: its LiDAR pose relative to sensor_head
is `0.35 0 -0.60 0 0 0`, matching both LiDAR TF aliases. This provides a level
planar scan at chassis height +0.25m, intersecting the low presentation obstacle.
The previous pitched-down scan made the ground part of the planar map and
produced a free-space gap around the start. This derived profile has its own
verification scope; historical sensor/localization acceptance does not transfer
automatically. Mast movement and wider terrain still require qualification.

The planner uses ROS simulation timestamps but steady-clock scheduling, so
slow simulation does not prevent command heartbeats or path renewal. Speed
is limited to 0.08m/s and 0.30rad/s. Failed plans are rate-limited and clear
the previous path; motion waits for path health. Reached goals remain stopped.
Software rendering defaults to two llvmpipe threads. Safety timeouts and
latched fault behavior are unchanged; a fault must not be hidden or bypassed.

## Recorded debugging evidence

`evidence/runs/20261107-gui-debug/` retains unsuccessful trials as well as
acceptance results. In `acceptance/`, the first short goal reached the 0.35m
tolerance. A subsequent idle map-TF freshness fault was explicitly reset only
after a current-position stop goal and healthy inputs. The longer goal in
`longer-after-reset.json` reached tolerance with RViz open and zero final output.
The sampler error did not recur in the corrected RViz log. These bounded
results are not a full mission, hardware or long-duration reliability claim.

The final cold-start run `combined-verified/` passed all startup checks without
a safety reset, reached the goals `(0.93695, 0.01409)` and `(1.10970, 0.01409)`,
and retained `GOAL_REACHED`. `final-control.txt` records health=OK and zero
target/output velocity. No safety latch or RViz shader error appears in that
run. Initial TF-cache message drops are informational and remain in rviz.log.
The first probe's time budget expired before its goal completed; the log and
final status capture establish completion afterward. The early post-startup
probe incorrectly saw a retained previous GOAL_REACHED and is not acceptance
evidence for the second goal; the probe now requires acknowledgment of its
own selected goal. Use navigation.log and final-navigation.txt for this run.
