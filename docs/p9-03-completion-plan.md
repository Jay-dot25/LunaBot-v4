# P9-03 completion plan

This plan closes the six-view dashboard performance gate with repeatable,
reviewable evidence. P9-03 is complete only when every required workload,
measurement, and target-profile gate below passes. A smoke layout or a
diagnostic run does not count as acceptance.

## Current starting point

- The Foxglove camera render probe v0.1.1 is installed and active.
- Existing probe trials are diagnostic: the layout is incomplete, viewport
  overrides are not a native target session, and the simulator ran near
  0.02x real time.
- Previous Phase L runs reported missing synchronized slope input and an
  occupancy-blocked plan.
- `config/profiling/resources.json` has no numeric release limits.
- The latest follow-up recovered WSL and ROS access through the authorized
  elevated WSL path; no host security control was changed.
- A recheck on 2026-11-14 confirmed the Windows host version (10.0.26340.0),
  12 host logical CPUs, 1536x864 primary display, and NVIDIA RTX 3050 Laptop
  GPU with 4096 MiB VRAM (driver 621.14). The earlier Ubuntu/WSL profile still
  records 8 logical CPUs and 4920 MiB; WSL could not be started to revalidate
  those values. Windows CIM inventory was also denied. The profile now records
  this distinction; it remains provisional and the release target has not
  been confirmed.
- The local Rosbridge source is connected again. The live Foxglove tab renders
  camera frames and receives mission/planner messages. This remains an
  exploratory unsaved layout, not a six-view acceptance layout.

Keep the existing raw evidence. New acceptance evidence should be stored in a
new dated run directory under `evidence/runs/` and must record the exact code,
configuration, dashboard layout, and profile revisions used.

## Ordered work

### 1. Freeze the target profile and acceptance limits

Record the target laptop/OS/WSL and graphics mode, native display resolution
and scaling, Foxglove client/version, ROS and simulator profile, and whether
hardware or software rendering is expected. Confirm this is the intended
release target rather than a temporary diagnostic host.

Before collecting release trials, fill in the CPU, RSS, GPU/VRAM, minimum RTF,
latency, and reconnect limits in `config/profiling/resources.json`. Derive the
limits from the product requirement and available target capacity; do not
derive them from the measurements after the fact. Record who approved the
limits and freeze the profile revision in the run manifest. If any metric has
no applicable limit, mark it `NOT_GATED` with a reason and approval; do not
silently omit it.

**Exit:** target profile and all applicable numeric thresholds are frozen and
versioned before acceptance data is collected.

### 2. Restore a usable, repeatable ROS test environment

Resolve WSL `E_ACCESSDENIED` through the normal authorized WSL/desktop path.
First verify distro availability and that the user session can launch it. If
WSL remains unavailable, move the same pinned test workspace and profile to an
approved ROS host; do not bypass host security controls. Record the chosen
host and environment.

Build and launch the Phase L stack with the P9-02 telemetry adapter and P9-03
control gateway. Verify the expected ROS graph, message types, frames, source
stamps, and publishers against `config/dashboard/contracts.json` and
`config/dashboard/adapter.json`. Save a topic/type inventory and startup log.

**Exit:** the graph is stable and all six view groups' required sources are
available or have an explicit, visible `unknown`/unavailable state.

### 3. Make the Phase L workload valid

Diagnose the `COST_MAP_WAITING_FOR_SYNCED_SLOPE` and
`MAP_REVISION_BLOCKED_PATH` messages using source stamps, frame IDs, map
revision, occupancy values, and planner/follower logs. Correct the source
configuration, synchronization, or deterministic test-map geometry so the
planner receives a fresh, compatible slope map and a safe, reachable route.
Do not weaken collision or freshness safety checks to force a pass.

Run the mission through its expected evaluation lifecycle and retain the
evaluator result. Verify source rates and simulation RTF are within the frozen
test profile before starting any dashboard timing trial. If the real simulator
cannot supply doubled traffic, use a documented staging/replay workload with
the same message types and representative payload sizes.

**Exit:** the workload is repeatable, the mission reaches its defined
evaluation outcome, and no prerequisite map/synchronization error invalidates
the dashboard measurements.

### 4. Build and verify the complete six-view dashboard

Create a native-size Foxglove layout with one clearly identified view for each
contract group in `config/dashboard/contracts.json`:

1. System and mission health.
2. Sensors and perception, including camera and depth.
3. Map and localization.
4. Terrain and risk layers.
5. Planning and evaluation.
6. Mission and safety controls.

For each group, include every required topic in the contract, using multiple
panels within a group when needed. Show source stamp, receive age, and
fresh/stale/unknown state. Keep the controls request-only through
`/lunabot/dashboard/command_request`; do not expose velocity or simulator-truth
writes. Save the layout and capture a screenshot plus a topic-to-view coverage
matrix. Start from `docs/dashboard-six-view-matrix.md`. Verify that no required
topic is absent and that missing sources are visibly unknown rather than
silently treated as fresh.

**Exit:** all six groups and every contract source are present, readable at the
frozen target resolution, and the control boundary matches the write contract.

### 5. Complete and validate measurement instrumentation

Use the installed v0.1.1 probe for its supported camera-panel receive-to-paint
diagnostics only. It does not measure Foxglove's built-in Image panel,
map/plan/status rendering, or transport-level drops. Add or select suitable
client-side instrumentation to record, for image, map, plan, and status:

- source message stamp;
- client receive monotonic time;
- actual display/update (render) monotonic time;
- derived source-to-render and receive-to-render age.

Record camera rendered FPS and a real stale/drop counter where available. If a
counter is unavailable, report it as unavailable; do not label cadence-derived
gap estimates as transport drops. Collect ROS bridge, adapter, simulator, and
dashboard-client CPU/RSS; dashboard GPU/VRAM and graphics mode; event-loop or
client latency; and startup/reconnect times. Validate clock alignment and
instrumentation with a controlled known-delay/single-frame check before the
release run.

**Exit:** every required metric has a defined source and unit, and unavailable
metrics are explicitly named before trial collection.

### 6. Run the baseline acceptance trials

Use the native Foxglove desktop client at the frozen target resolution. Start
all six views and all required telemetry sources. Warm up for 60 seconds, then
run three independent five-minute trials without changing configuration.
Capture one-second resource samples and per-frame render samples throughout.
Record exact start/end times, app versions, layout/profile hashes, source rates,
RTF, and any interruptions for each trial.

Also test a controlled disconnect and reconnect, recording connection startup,
disconnect detection, recovery duration, subscription recovery, and whether
stale data is visibly marked during the outage. Perform the guarded e-stop
request only in the isolated simulation/safety-approved procedure; record
dashboard request-to-controller acknowledgment separately from existing
controller/watchdog reaction measurements. Do not send motion commands to a
physical rover as part of this dashboard benchmark.

**Exit:** all three baseline trials are valid, complete, and have raw data for
every required metric; reconnect and safety acknowledgment evidence is saved.

### 7. Run the doubled-traffic acceptance trials

Repeat the same warm-up and three five-minute trials with camera, depth, and
map traffic doubled using the frozen profile or documented staging replay.
Keep target resolution, layout, client, and other stack settings unchanged.
Record the same render and resource metrics. Confirm that telemetry freshness
and guarded control status remain correct under load.

**Exit:** doubled-load evidence is complete and comparable to the baseline;
any inability to generate the specified load is recorded as a failed
prerequisite, not a pass.

### 8. Evaluate, repair if needed, and close the gates

Create a result table with one row per frozen threshold and functional check.
Use the three-trial p95/worst-case rule defined by the frozen profile; report
all trial values, not only an aggregate. Mark each row `PASS`, `FAIL`, or
approved `NOT_GATED`, with a link to raw evidence. If any row fails, fix the
identified cause and repeat the affected baseline and doubled-load trials
after freezing a new configuration revision. Never change a limit after seeing
the result to convert a failure into a pass.

Update `docs/dashboard-performance.md`, the P9-03 task-board row, and the
P11-01 resource-gate record. Mark P9-03 complete only when all mandatory
functional checks and applicable performance limits pass, both workloads have
valid evidence, and no release-blocking unknown remains. Mark dependent P11-01
complete only after its own resource thresholds and overload behavior are
evaluated against the same frozen target profile.

**Exit:** a reviewer can reproduce the setup from the recorded revisions and
verify every gate from retained raw measurements and the signed-off result
table.

## Execution status (2026-11-14, follow-up)

- Step 1 is partially prepared: host version, CPU count, primary display, GPU,
  and driver are recorded in `config/profiling/resources.json`. Scaling and
  release-target confirmation are missing; acceptance limits remain unset.
- Step 2 recovered through the authorized elevated WSL path. The Humble Phase L
  graph and Rosbridge on port 9090 are live with `ROS_DOMAIN_ID=42` and
  `ROS_LOCALHOST_ONLY=1`. The P9-02 telemetry adapter and P9-03 guarded control
  gateway were started. `ros2 topic list -t` showed every source in
  `contracts.json`, including both camera/depth camera-info topics and joint
  states. Live graph inspection confirmed the adapter publishes
  `/lunabot/dashboard/telemetry` as `std_msgs/msg/String`, the gateway publishes
  `/lunabot/dashboard/control_status` as `std_msgs/msg/String`, the semantic
  slope layer is `nav_msgs/msg/OccupancyGrid`, and `/map` is
  `nav_msgs/msg/OccupancyGrid`. The adapter and gateway each have one publisher;
  their telemetry/status connections are live. The telemetry sample includes
  adapter monotonic time and ROS clock metadata.
  A contract-to-graph comparison retained
  `evidence/runs/20261114-p903/P9-03/followup/live-ros-topic-inventory.csv`:
  36/36 configured read-only topics are present, and all 27 topics also typed
  by the adapter match their configured ROS message types. The current snapshot
  correctly marks the no-sample `/tf_static`, `/goal_pose`, `/plan`, and
  evaluation streams unknown and marks timed-out autonomy/planning/terrain
  streams stale; graph presence alone is not treated as fresh data.
- Step 3 remains invalid for acceptance. The mission is waiting for a goal;
  this Phase L launch is in its documented manual-goal mode, and the dashboard
  contract intentionally reports `/goal_pose` and `/plan` as unknown until a
  real goal is selected. The live planner's last status is
  `MAP_REVISION_BLOCKED_PATH`; logs show it invalidating plans after map
  revisions, while the mission has no goal ID. Semantic mapping has only about
  104 observed terrain cells in the 160x160 grid, and its source skew recently
  exceeded the configured 250 ms limit (274 ms). Cost-map logs show many
  unknown cells and recurring synced-layer waits. This makes simulation
  scheduling/synchronization and sparse observed terrain the current workload
  blockers; the last planner status may be retained from an earlier goal and
  must be re-evaluated on a fresh controlled regression mission. The
  occupancy/inflation checks remain unchanged. No goal, safety, or motion
  command was sent.
- Step 4 remains incomplete. Foxglove is connected. The unsaved exploratory
  draft now has tabs titled “System and mission health,” “Sensors and
  perception,” and “Depth and perception”; the first holds a safety-health
  message view and the sensor tabs show image panels. These tabs do not yet
  cover every required topic, the depth panel is not yet verified against its
  intended topic, and the complete six-group layout is not saved or accepted.
- Steps 1, 5, 6, and 7 remain open: display scaling/target confirmation and
  approved numeric limits are missing; supported render/resource instrumentation
  is incomplete; and no valid baseline or doubled-traffic acceptance trials
  have been collected. The active camera probe is diagnostic at roughly 0.02x
  source real-time and cannot establish acceptance performance.
- No acceptance trials or safety requests were performed during this plan
  execution.

## Completion checklist

- [ ] Target host, native resolution, software, and graphics mode verified for acceptance (engineering profile recorded; WSL allocation and display scaling unverified).
- [x] Numeric engineering limits frozen before acceptance measurements; target runtime verification remains open.
- [x] WSL access restored or an approved equivalent ROS host selected.
- [x] Phase L topics, stamps, frames, map synchronization, and planner workload valid for the recorded 2026-11-16 run.
- [ ] All six contract views and every required topic verified in the live layout (extension installed; draft unsaved and disconnected).
- [ ] Render and resource instrumentation validated; limitations explicit.
- [ ] Three valid five-minute baseline trials collected after warm-up (one mixed-domain diagnostic failed CPU/RTF gates).
- [ ] Three valid doubled-traffic trials collected after warm-up.
- [ ] Reconnect recovery and simulation-only guarded e-stop acknowledgment recorded.
- [ ] Every functional and performance gate evaluated against frozen limits.
- [ ] Evidence, dashboard docs, task board, and P11-01 status updated.

## Execution update — 2026-11-16

The headless Phase L workload prerequisite now passes with the active P5-03
three-class model. The accepted retry2 run processed 365 three-class frames,
updated semantic and cost maps, exercised planning and dynamic replanning,
completed the mission evaluator and mission demo, saved the final map, and shut
down cleanly. See
`evidence/runs/20261116-threeclass-phase-l-retry2/README.md`. This closes the
previously open Phase L topics, map synchronization, and planner-workload item
for this recorded run.

P9-03 remains **IN PROGRESS**. The six-view Foxglove layout, target-host render
and resource instrumentation, reconnect recovery, doubled-traffic trials, and
frozen numeric limits have not been accepted. Computer-use approval timed out
when attempting to inspect the desktop session, so no dashboard changes or
acceptance trials were performed. P11-01 resource limits also remain open.

## Execution update — 2026-11-17

The extension `lunabot.lunabot-p903-render-probe-0.2.0.foxe` is installed in
the signed-in Foxglove client. The hosted Personal layout `Coffee Dragonfly` is
saved with all six named views and connected to the live domain46 adapter at
`ws://localhost:9091`. Its topic-health panels show live samples, but
`/lunabot/mission/events` and `/lunabot/dashboard/control_status` had no
samples. Observed receive-to-render p95 ranged 83–202 ms, exceeding the frozen
100 ms limit for some rows. Native image/map/plan rendering is not yet measured.

The frozen profile `wsl2-hpvictus15-20261117-v1` has two isolated five-minute
baselines. Both pass CPU/RSS/VRAM and fail RTF (0.174 and 0.165); client render
p95 is absent. Windows inventory confirms the documented Victus 15 family,
Ryzen 5 5600H, RTX 3050 Laptop GPU, and about 8 GB RAM; the measured simulator
is still a WSL software-rendered workload. An idle-adapter reconnect restored
custom panel subscriptions in 2.94 seconds, with safety-health receive-to-
render p95 at 33 ms after recovery. It did not carry simulator or camera/map
traffic. No valid three-trial baseline/doubled-traffic set, full-workload
reconnect, target-size client CPU/VRAM sample, or overload result exists.
P9-03 and P11-01 remain open.

P11-03 run11 passed the simulated mission/evaluator gates. Restart run13 failed
goal completion. Run14 passed most startup/topic checks but failed its retained
terrain-plan sample and dynamic-replan gate, then latched `HEALTH_STALE_SCAN`
and entered mission `FAILED`; see
`evidence/runs/20261117-p1103-restart/P11-03/run-14/README.md`. A passing
restart is still required. The four planner-only P10 scenarios remain outside
live-mission acceptance; physical hardware is not qualified; and the separate
Goal Master Plan referenced by the project is still not found.

Run15 D3D12 renderer diagnostic also failed: RTF was 0.2066 against the frozen
0.5 minimum, and the mission failed with `HEALTH_CLOCK_STALLED` and
`NONPOSITIVE_DIMENSIONS`. The initial depth sample timed out even though
downstream perception later read depth frames. The launch probe now retries
header-only receives for up to 90 seconds. This diagnostic did not complete
the live acceptance suite; run details are in
`evidence/runs/20261117-p1103-restart/P11-03/run-15-renderer-d3d12/README.md`.

Run16 exercised a targeted mission-supervisor fix: zero-sized SLAM placeholder
maps now leave the mission in `WAITING_FOR_MAP` instead of forcing `FAILED`.
The trial confirmed that behavior, but `/clock`, `/lunabot/odom`, and
`/lunabot/control/status` live-sample checks failed, and the mission/evaluator
did not pass. See
`evidence/runs/20261117-p1103-restart/P11-03/run-16-map-startup-fix/README.md`.

Run17 on the software-rendered profile passed the complete live topic/content
startup contract and dynamic replanning, but the terrain follower did not
reach the goal before the outer 900-second trial cap, which fired during the goal wait; the full inner wait interval may not have elapsed. The mission and aggregate
evaluator did not pass. Its concurrent five-minute profile measured RTF
0.2555, below the frozen 0.5 minimum, with dashboard latency unmeasured. See
`evidence/runs/20261117-p1103-restart/P11-03/run-17-software-restart/README.md`
and `evidence/runs/20261117-p1103-restart/P11-01/run17-restart-full-mission/`.

Run18 aligned the WSL demo goal with the documented 4 m route but latched
`HEALTH_STAMP_RGB_INFO` before reaching the goal. Its five-minute RTF was
0.3336, still below the frozen 0.5 minimum. Run19 added safety timestamp
diagnostics; the CameraInfo latch did not recur, but the follower again failed
to reach the goal before the outer 900-second cap. The evaluator did not pass.
See the run reports under
`evidence/runs/20261117-p1103-restart/P11-03/run-18-goal-distance-fix/` and
`evidence/runs/20261117-p1103-restart/P11-03/run-19-health-diagnostic/`.

### Live six-view diagnostic — 2026-11-17

The hosted `Coffee Dragonfly` layout is saved with six named topic-health views
and was connected to domain46 Rosbridge during run22. The live mission passed
startup topic/content checks, terrain planning, dynamic replanning, goal
completion, and evaluation, then latched `HEALTH_STAMP_IMU`; its launcher hit
the 900-second cap before clean shutdown. The browser showed current
fresh/stale/unknown states, but sensor render percentiles remained `n/a` and
the viewport was 1085x685 rather than the unverified target size. This is not
a valid baseline trial.

Run21 separately achieved RTF 0.5057 on a 2 ms diagnostic world and passed
mission/evaluator/clean-shutdown checks. Three P3 repetitions on that world
included one failed turn-accuracy trial (0.2912 rad against 0.10 rad), so it
cannot be promoted as the acceptance profile. Run22's resource sampler passed
CPU/RSS/VRAM, but could not observe `/clock` and has no RTF value. Its sampler
now subscribes with best-effort QoS; an isolated synthetic clock probe worked,
but the live-domain issue remains unresolved.

P9-03 remains **IN PROGRESS**. The required three five-minute baseline and
three doubled-traffic trials, valid render/resource instrumentation,
workload reconnect, overload behavior, and target-size verification remain
open. See the run reports and `docs/dashboard-performance.md` for the retained
evidence.
