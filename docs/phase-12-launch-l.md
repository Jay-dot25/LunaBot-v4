# Phase L — Final Mission Demonstration (`launch-l`)

## 1. Revised presentation target

The final project demonstration is a GUI mission, not only a headless
regression. It must show a Gazebo lunar habitat scene beside RViz and let the
operator select the rover's destination. The rover must sense a real obstacle
in front of it, update the obstacle/cost map, replan around it, and reach the
selected goal.

The final screen should contain:

- Gazebo: lunar terrain, habitat structures, rover, and physical obstacles;
- RViz: SLAM map, LiDAR returns, sensed obstacle map, cost map, goal marker,
  terrain-aware path, and active replanned path;
- visible status/evidence for goal selection, obstacle detection, replanning,
  and goal completion.

Phase L preserves the approved Phase A-K control, SLAM, mapping, planning,
replanning, evaluation, and shutdown infrastructure. It adds a real LiDAR
obstacle detector and a retained mission supervisor with lifecycle commands
and a durable event journal. The supervisor does not publish velocity or goals.

## 2. Final modes

### Manual presentation mode

This is the acceptance mode shown to a person:

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
FINAL_DEMO=1 AUTO_GOAL=false EVIDENCE=1 ~/launch-l
```

Gazebo and RViz open. In RViz, select the `Set Goal` tool, click a destination
on the map, and drag to choose the desired final heading. The rover then
plans and drives to that manually selected goal.

For software-rendered WSL GUI runs, use the bounded scheduler profile that was
qualified during P8 round-trip trials while retaining manual goal selection:

```bash
WSL_SOFTWARE_PROFILE=1 FINAL_DEMO=1 AUTO_GOAL=false EVIDENCE=1 \
  bash ./launch-l
```

This profile allows bounded WSL scheduling gaps for the simulation clock,
sensor freshness, path renewal, and obstacle-map updates. The controller's
0.5-second command watchdog remains enabled. Treat this as a WSL presentation
profile; it does not replace the final target-machine acceptance benchmark.

### Automated regression mode

This mode is retained for repeatable infrastructure testing. It deliberately
uses an automatic test goal and does not replace the manual presentation gate:

```bash
DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false \
  EVIDENCE=1 HEADLESS=1 ~/launch-l
```

## 3. Data flow

```text
Gazebo lunar habitat + physical obstacle
              │
              ├── /lunabot/lidar/scan
              ├── /lunabot/depth/image_raw
              └── /lunabot/camera/image_raw
                       │
                       ▼
              LiDAR obstacle detector
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
 /lunabot/obstacles/status  /lunabot/obstacles/map
          │                         │
          └────────────┬────────────┘
                       ▼
              terrain cost map overlay
                       │
                       ▼
              terrain-aware planner
                       │
                       ▼
       /cmd_vel_in -> controller -> /cmd_vel

RViz Set Goal -> /goal_pose -> planner/follower
Operator -> /lunabot/mission/command -> mission lifecycle supervisor
mission lifecycle supervisor -> /lunabot/mission/status -> follower motion gate
mission lifecycle supervisor -> SQLite event journal + /lunabot/mission/events
```

The physical forward obstacle is part of the Gazebo world. It is not a
synthetic status message. The detector observes real LaserScan returns,
publishes a map-frame obstacle overlay, and reports distance/bearing evidence.
The cost mapper incorporates those cells and inflates them before planning.

The follower checks the remaining route against each fresh obstacle grid before
publishing a motion command. If the route is blocked, it publishes a zero
command, clears the local route, and rejects repeated copies of that same plan
until the planner supplies a changed route. Its status reports
`detection_to_stop_ms` from obstacle-grid receipt to the blocking check and
`replan_latency_ms` from local invalidation to acceptance of a changed path.
The dynamic replan monitor separately reports
`obstacle_replan_latency_ms` from its first `OBSTACLE_DETECTED` status to the
next changed terrain path; the obstacle-gated monitor PASS requires this value.
These status timings use process monotonic clocks and do not include sensor
acquisition or transport delay before the relevant message is received.
An obstacle-triggered path invalidation holds the follower at zero while a
changed route is planned; this transient stop keeps the mission active. Local
obstacle intrusion and stale path or obstacle-map stops block motion until an
operator performs explicit recovery.

## 4. Mission acceptance contract

The retained `/lunabot/mission/status` publisher requires:

- a non-empty terrain plan;
- `DYNAMIC_REPLAN_PASS`;
- `INTEGRATION_GOAL_REACHED`;
- retained Phase K `EVALUATION_PASS`;
- a real `/goal_pose` sample;
- a real `/map` sample;
- `OBSTACLE_DETECTED` from the LiDAR detector;
- `MANUAL_GOAL_SELECTED` in manual presentation mode.

A successful result contains:

```text
MISSION_DEMO_PASS plan=1 replan=1 goal=1 evaluation=1 \
  goal_pose=1 map=1 obstacle=1 manual_goal=1
```

The automated regression mode sets `REQUIRE_MANUAL_GOAL=false` only so it can
exercise the rest of the stack without an operator. The final presentation
must use manual mode.

## 5. Mission lifecycle interface

The mission supervisor reports lifecycle on the retained
`/lunabot/mission/status` topic. Valid inputs move it through
`WAITING_FOR_MAP` → `WAITING_FOR_GOAL` → `READY` → `ACTIVE` → `COMPLETED`.
An invalid occupancy grid or goal pose moves the current mission to `FAILED`;
motion remains inhibited until a valid map and goal arrive and an explicit
`RECOVER` command is sent. Restart also creates a fresh `WAITING_FOR_MAP`
instance. A new valid goal after completion starts a new goal-scoped attempt.

Before counting a map, the supervisor requires the configured map frame,
positive width/height and finite positive resolution, exact data length,
occupancy values in -1..100, and a finite, nonzero origin quaternion. Goal
poses require a frame, finite coordinates, and a nonzero orientation. Invalid
inputs are rejected with `MISSION_MAP_REJECTED` or `MISSION_GOAL_REJECTED` and a
reason code; they do not satisfy mission completion criteria.

Each accepted `PoseStamped` goal receives a stable opaque ID derived from its
frame, timestamp, and pose. A changed goal ID clears plan, replan, goal-reached,
and evaluation evidence from the previous goal. Lifecycle and goal ID are
included in waiting and final status text; `MISSION_DEMO_PASS` also carries the
completed goal ID.

Commands arrive as `std_msgs/String` on `/lunabot/mission/command`:

| Command | Effect |
|---|---|
| `PAUSE` / `RESUME` | Pause or resume a READY/ACTIVE mission. |
| `CANCEL` | Enter terminal CANCELLED; a process restart is required to begin a new run. |
| `RECOVER` | Recover from BLOCKED or FAILED only after a valid map and goal are present; return to READY and require fresh planning evidence. |
| `ESTOP` / `RESET_ESTOP` | Latch E_STOPPED; reset requires valid map and goal and returns to READY. The lower-level safety controller's emergency-stop latch remains independently enforced. |
| `FAIL` | Enter FAILED for an explicit mission fault. |
| `DELIVER <delivery-id>` | Record delivery only after COMPLETED. The delivery ID is the persistent idempotency key. |

The follower listens to retained `/lunabot/mission/status` during Phase L and
publishes motion only in ACTIVE. PAUSED, BLOCKED, FAILED, CANCELLED, E_STOPPED,
COMPLETED, unknown state, and stale supervisor heartbeat all force zero at
`/cmd_vel_in`. The supervisor repeats its retained status every 0.5 s; the
follower stops if no status arrives within 1.5 s. Phase I leaves this gate
disabled because it has no Phase L mission supervisor.

Lifecycle transitions and delivery events are stored in SQLite at
`$EVIDENCE_DIR/mission_events.sqlite3` using WAL and synchronous commits.
`/lunabot/mission/events` emits delivery records. Repeating
`DELIVER <delivery-id>`—including after restarting the mission process—returns
the original stored goal ID with `duplicate=1` and does not add another
`DELIVERY_COMPLETED` row. The event journal survives the process restart; the
in-memory mission lifecycle restarts at `WAITING_FOR_MAP`.

## 6. RViz presentation configuration

`rviz/phase_l.rviz` provides:

- `map` fixed frame;
- SLAM map;
- LiDAR scan;
- sensed obstacle map;
- forward obstacle markers;
- terrain semantic and cost maps;
- selected goal pose;
- diagnostic A* path;
- terrain-aware and active paths;
- camera and terrain overlay views;
- RViz `Set Goal` tool publishing `/goal_pose`.

For WSL software rendering, the LaserScan display starts disabled so the first
occupancy-map texture can initialize before the scan display is enabled. Once
the map is visible, enable `LaserScan` from RViz's Displays panel if the scan
overlay is needed. TF frame names are hidden by default to keep the rover view
legible; the TF tree and axes remain available.

## 7. Validation

Static validation:

```bash
python3 tools/validate_phase_l.py
```

Manual final mission gate:

```bash
FINAL_DEMO=1 AUTO_GOAL=false EVIDENCE=1 ~/launch-l
```

The run must contain:

```text
manual RViz goal selection: PASS
obstacle detector content: PASS
OBSTACLE_DETECTED
DYNAMIC_REPLAN_PASS
final mission demonstration: PASS
MISSION_DEMO_PASS
saved map evidence (YAML + PGM): PASS
PHASE L RUN COMPLETE - overall result: PASS
Launch L environment cleanly closed.
```

The final gate must be repeated after a clean shutdown. Two independent GUI
runs are required for final presentation approval. Automated headless runs are
useful regression checks but do not substitute for the manual-goal gate.

## 8. Evidence

## 9. P8-03 round-trip mode

`P8_ROUNDTRIP=1` disables the diagnostic single-goal auto-target, uses the
software-rendering profile needed by headless WSL Gazebo, and starts the
fail-closed itinerary runner. The runner sends Habitat, Science, and Habitat
as three separate map-frame goals. It requires the supervisor's completion
message to carry the exact goal ID it published for each leg, then records a
single durable delivery after the return leg. A new goal must produce a new
non-empty plan and a new reached-goal signal; the launch-wide obstacle,
replanning, and evaluator gates stay qualified for the trial.

First run a map-qualification pilot with configured world sites. It records the
measured startup `map -> odom` transform and saves the final SLAM map, but it is
not an acceptance trial:

```bash
P8_ROUNDTRIP=1 P8_MAP_QUALIFICATION=1 P8_RUN_ID=p803-map-qualification \
  HEADLESS=1 DEMO=1 EVIDENCE=1 \
  EVIDENCE_DIR=evidence/runs/p803-map-qualification/phase-l \
  ./scripts/launch-l.sh
python3 tools/bind_p8_waypoints.py \
  --map-yaml evidence/runs/p803-map-qualification/phase-l/phase_l_map.yaml \
  --qualification evidence/runs/p803-map-qualification/phase-l/p8_roundtrip.json
```

The binder refuses a pilot without all three completed site legs, a saved-map
free cell at both staging poses, or a measured transform. Once the waypoint
registry is bound to that map, run the three independent acceptance trials:

```bash
./tools/run_p8_acceptance.sh
```

Each trial gets a fresh Gazebo world and mission journal under one evidence
directory. The aggregate verifier requires nine exact goal completions and
three unique durable delivery rows, each tied to its own return goal.

Runtime evidence is written to `evidence/phase-l-launch-l/`:

| File | Meaning |
|---|---|
| `obstacles.log` | real LiDAR obstacle detector diagnostics |
| `obstacle_status.txt` | detected/clear obstacle status |
| `obstacle_map.txt` | sensed obstacle map sample |
| `mission.log` | mission lifecycle supervisor diagnostics |
| `mission_events.sqlite3` | durable lifecycle and idempotent delivery event journal |
| `mission_status.txt` | retained mission status sample |
| `mission_wait_status.txt` | live `MISSION_DEMO_PASS` stream |
| `goal_selection_wait_status.txt` | real manual goal sample |
| `evaluation_status.txt` | aggregate Phase K evaluation |
| `replan_status.txt` | dynamic replanning result |
| `phase_l_map.yaml` / `phase_l_map.pgm` | final map evidence |
| `static_validation.txt` | static Phase L report |
| `last_run.log` | ordered launcher result |

Do not hand-edit runtime evidence. Inspect `obstacles.log`, `mission.log`,
`evaluation.log`, `replan.log`, `integration.log`, and `last_run.log` when a
gate fails.

## 9. Acceptance checklist

- [ ] Habitat scene, rover, physical obstacles, and lunar terrain are visible in Gazebo.
- [ ] RViz provides a working Set Goal tool and displays the selected goal.
- [ ] Manual goal is selected with `AUTO_GOAL=false`.
- [ ] Real LiDAR obstacle detection reports a forward obstacle.
- [ ] Obstacle cells appear in the obstacle/cost map.
- [ ] The terrain path changes around the sensed obstacle.
- [ ] Rover reaches the manually selected goal.
- [ ] Controller boundary and odometry evidence pass.
- [ ] Final YAML/PGM map evidence passes.
- [ ] Two independent GUI runs and clean relaunch pass.

Phase L is the final phase; no later phase is defined.
