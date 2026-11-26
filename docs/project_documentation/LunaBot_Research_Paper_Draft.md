# LunaBot V4: An Evidence-Tracked ROS 2 Simulation Stack for Terrain-Aware Lunar Rover Navigation

**Manuscript status:** research-paper draft based on repository state and dated simulation evidence; author affiliations, final frozen experiment protocol and target venue remain to be supplied.

## Abstract

Planetary rover navigation requires a coordinated pipeline for sensing, localization, traversability assessment, planning, control and fault response. This paper presents LunaBot V4, a ROS 2 Humble and Gazebo Sim research implementation of that pipeline using a simulated articulated six-wheel rover. A LiDAR/odometry mapping path based on `slam_toolbox` provides an occupancy map and map-to-odometry correction. A learned RGB terrain segmentation model, RGB-D projection and geometry-derived slope, roughness, crater-risk and uncertainty layers feed a configurable terrain cost grid. A simulated LiDAR obstacle detector adds a separate obstacle overlay. Weighted grid A* replans on changed map inputs, while a freshness- and mission-gated path follower routes commands through a single ROS safety controller and an independent simulator actuator watchdog. A mission supervisor records state transitions and delivery events. Evidence includes a 0.2299 m short-route localization error, held-out three-class mIoU 0.6057 on 749 records, four seeded simulation calibration captures, three habitat–science–habitat simulation round trips, 15 passing deterministic planner fixtures, and two 2026-11-19 manual GUI runs that passed all 11 presentation-specific checks plus final map save. In those GUI runs, the reported obstacle-to-changed-path interval was 56.8 s and 75.9 s; these profile-specific values show high variability and are not target-platform performance. Results have different scopes and are not combined as a real-world success rate. Five cross-trial metrics remain unavailable, including mission travel time, trajectory error, collision count, clearance and recovery. LunaBot contributes an inspectable integration and evidence workflow rather than a new SLAM or path-planning algorithm. It is a student simulation and does not establish flight readiness or lunar terrain performance.

**Keywords:** planetary robotics; autonomous navigation; ROS 2; Gazebo Sim; terrain traversability; semantic mapping; LiDAR SLAM; weighted A*; rover simulation.

## 1. Introduction

Rovers exploring remote planetary surfaces cannot rely on ordinary GNSS navigation and often operate with constrained communications and uncertain terrain. A useful autonomy stack must coordinate perception, map construction, route selection, motion and safety. Individual algorithms are not sufficient: interface timing, coordinate frames, sensor calibration, stop behavior and experiment evidence affect end-to-end behavior.

LunaBot V4 is a staged software/simulation project created to expose these interactions in a reproducible ROS 2/Gazebo environment. It models a six-wheel articulated rover, simulates RGB, depth, LiDAR, IMU and wheel odometry, and incrementally adds SLAM, perception, terrain cost mapping, planning, replanning and mission-state logic. The study's scope is navigation integration and engineering qualification within bounded simulated scenarios.

## 2. Problem statement and research gap

**Problem:** Given a locally mapped lunar-like environment, simulated rover pose, sensor observations and a user-selected map-frame goal, produce and execute a route that accounts for occupancy, semantic/geometry terrain risk and newly sensed obstacles, while stopping on stale or unsafe command conditions.

**Gap addressed:** Student mobile robot demonstrations often show only goal following or isolated algorithms. LunaBot places sensing, map layers, a command safety boundary, mission evidence and explicit evaluation scope into one inspected project. This is an engineering integration gap, not a claim that prior rover research lacks such architectures. Flight projects such as Curiosity and Perseverance have far more advanced, validated onboard autonomy; public NASA descriptions provide context [1–4].

## 3. Contributions

1. Phase-gated ROS 2/Gazebo integration from simulated sensing through terrain-aware planning and actuator command, with source-level interface ownership.
2. A fused traversability grid combining active semantic labels, RGB-D-derived geometry risks and an independently detected LiDAR obstacle overlay.
3. A stop-on-invalidated-path execution pattern, with stale path/map and mission-lifecycle gates and a single velocity authority.
4. Evidence accounting that separates deterministic planner fixtures, held-out perception, simulation calibration and live simulation mission runs, including explicit unavailable metrics.

No algorithmic novelty is claimed for A*, weighted A*, SLAM, CNN inference, ROS 2 or Gazebo.

## 4. System architecture and methods

### 4.1 Simulation and sensing

The Gazebo world uses lunar gravity of 1.62 m/s², generated mesh terrain, habitat/rock obstacles and an articulated six-wheel SDF model. DiffDrive groups wheels left/right and emits local odometry. Sensors are Gazebo simulated. Baseline SDF camera settings are 160×120 at 20/15 Hz; a Phase B live audit observed 640×480 streams at 19.71/14.48 Hz, which indicates launch/runtime model variation and must be pinned for a repeatable paper experiment. The LiDAR SDF is a 720-beam 2D scan at 10 Hz; IMU configured at 100 Hz.

### 4.2 Localization and maps

`slam_toolbox` consumes `/lunabot/lidar/scan` and odometry/TF, publishes `/map` and `map→odom`. DiffDrive owns `odom→chassis`. RGB-D camera coordinates are projected using intrinsics and timestamped camera/base/map transforms. Semantic mapping produces semantic, confidence, age, slope, roughness, crater-risk and uncertainty layers. A cost fusion uses normalized weights 0.25, 0.20, 0.35 and 0.20 for slope, roughness, crater risk and uncertainty; occupancy/obstacle semantics bound the output to 0–100. These are configurable heuristic costs.

### 4.3 Planning and motion

Occupancy A* is retained as a baseline. Terrain weighted A* searches aligned grid cells and adds traversability penalties. Planner output is a `nav_msgs/Path`; the terrain follower uses bounded heading/linear control and checks path/map freshness and obstacle intersection. An obstacle-triggered invalid path causes zero motion until a changed safe route arrives. D* Lite was evaluated as a candidate on small deterministic grid fixtures but rejected for the current workload; it is not in the live ROS command path.

### 4.4 Safety and mission state

The follower publishes `Twist` to `/cmd_vel_in`; `control_odometry.py` validates and constrains it, then publishes the sole ROS `/cmd_vel` stream. A native Gazebo watchdog provides a second actuator timeout. Phase L motion additionally requires fresh ACTIVE mission status. The supervisor validates map and goal inputs, checks plan/replan/goal/evaluation evidence and journals events. This is a ROS String command/status interface and SQLite event store, not a ROS action server or sample collection mechanism.

## 5. Experimental setup

The project qualification environment is Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress (a recorded qualification reports Gazebo 1.0.3, `slam_toolbox` 2.6.10, Ignition Transport 11.4.2 and Ignition Messages 8.7.1). Current launcher/runtime model details and all configurations should be frozen by hash for replication. Gazebo truth is kept outside the operational ROS graph for selected qualification comparisons.

Evaluation sources include: P3-02 short-route truth comparison; P5-02 held-out three-class semantic evaluation; P6 four seeded calibration captures; P8-03 three live round trips; P10-02 deterministic planner scenarios; and P10-03 cross-trial accounting. These are not a unified benchmark campaign: scenarios, map, metrics and timebases differ. See project `LunaBot_Experiments_and_Results.md` for paths and detailed scope.

## 6. Results

| Measure | Result | Interpretation |
|---|---:|---|
| P3-02 short-route localization | 0.229877 m position; 0.000053 rad yaw | Within 0.25 m / 0.15 rad simulation gate, with limited position margin and short route. |
| Active model held-out mIoU | 0.605734; n=749 | Class IoU: regolith 0.9181, rock 0.3691, shadow 0.5300. Dataset-specific, no safety rate inference. |
| P6 plane calibration | Horizontal p95 0.0219 m (1.5–3m); 0.0576m (3–5m) | Seeded simulation; vertical p95 0.0096m and 0.0152m. Four captures including crater geometry. |
| P8-03 mission events | 3 round trips, 9 completed legs, 3 durable deliveries | One bound simulation map/profile; not independent environments. |
| P10-02 planner fixtures | 15/15 pass; terrain path intersects configured hazard region 0/6 vs occupancy 6/6 | Grid paths only; no rover contacts/clearance outcomes. |
| P10-03 evaluation | Complete with limitations | 9 limited metrics, 5 unavailable; no mission time, trajectory error, collision count, clearance or recovery outcome. |
| Manual GUI mission trials (2026-11-19) | 2 full WSL presentation-gate passes | Both runs passed all 11 presentation-specific checks plus final map save: goal selection, terrain planning, replanning, follower goal, evaluation, mission, command motion, controller, live map update and typed graph. Replan interval: 56.8 s / 75.9 s; no target machine qualification. |

## 7. Discussion

The results show that the implemented interfaces can support a simulated perception-to-route-to-motion chain and that some narrow gates have quantitative evidence. Two new manual GUI trials passed the complete presentation gate set, strengthening mission integration evidence. Their obstacle-replan intervals of 56.8 s and 75.9 s were high and variable. Later startup-only runs do not add complete mission trials; same-day partial failures are retained. This is not a reliable mission success-rate estimate. Held-out segmentation IoU does not equal safe obstacle detection; map calibration against generated terrain does not establish physical-camera accuracy; planner hazard-cell avoidance does not demonstrate physical clearance. Sensor rate measurements are simulation-time observations.

The separate safety control boundary is a useful engineering property, but it is not a certified safety system. The remaining measured obstacle-to-changed-plan delay includes local monotonic-clock intervals after status reception and excludes earlier acquisition/transport; it should not be compared directly to offline planner timings. Terrain costs need calibration against actual mobility outcomes. No end-to-end energy/compute or physical rover data is available.

## 8. Limitations

The environment lacks a validated lunar soil model, vacuum, electrostatic dust, thermal/radiation environment and communication constraints. All sensors are simulated; live RGB-D projection error is not fully measured. Localization has a narrow short-route gate and wheel slip remains a problem outside its calibrated friction profile. The path planning representation is 2D and not fully footprint/rollover/energy aware. Five registered performance metrics are unavailable. No physical rover, sample detector, manipulator or collection hardware exists in this scope. Two WSL GUI mission gate passes do not qualify a target platform or whole-stack performance. The broader current A–L runtime regression remains open.

## 9. Future work

Priorities: (1) repeated clean Phase L qualification with complete input samples, (2) measure collision/contact, physical grid clearance, travel time, true trajectory and recovery, (3) long-route/slip/dropout localization qualification, (4) live camera/depth projection and uncertainty evaluation, (5) whole-stack resource and sensor-to-stop latency measurement, (6) calibrate traversability against physical mobility tests, (7) compare footprint-aware/Nav2 alternatives under matched seeded scenarios, and (8) instrumented hardware/sim-to-real work. AI or reinforcement learning should be considered only with a clear benchmark advantage and bounded safety assurance.

## 10. Conclusion

LunaBot V4 is an evidence-tracked student simulation integrating ROS 2, Gazebo, LiDAR SLAM, RGB-D semantic/geometry maps, cost-aware grid planning, obstacle replanning, bounded motion control and mission lifecycle evidence. Its useful contribution is an inspectable integration and validation architecture. Current results are promising within specific simulation and fixture scopes, but performance gaps and physical lunar limitations remain explicit. The implementation should be described as a research prototype, not as a flight-qualified autonomous lunar rover.

## References

See `references.md` for linked full citations. Key contextual references are NASA/JPL AutoNav overview [1], JPL Curiosity navigation [3], PDS Navcam material [4], NASA VIPER rover/instrument material [7–8], ROS 2 Humble [10], `slam_toolbox` [11], Gazebo [13], A* [15] and D* Lite [14]. Add venue-compliant bibliography formatting and verify exact citation metadata before submission.
