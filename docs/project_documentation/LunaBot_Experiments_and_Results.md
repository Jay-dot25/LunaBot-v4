# LunaBot V4 Experiments, Results and Evidence Ledger

## Evidence policy

Each result below is named by evidence run and scope. A passing unit/fixture test is not a live robot trial. A successful simulation is not real lunar validation. A missing/null metric is **NOT MEASURED**, never zero. Avoid combining unlike runs into one performance claim. Repository audit date: 2026-11-19.

## Experiment register

| Experiment | Objective | Method | Expected result | Actual result | Status |
|---|---|---|---|---|---|
| Phase B sensor contract P2-01 | Verify sensor payload/rate/frame and CameraInfo contracts | Live Phase B capture and ROS audit | Correct topic, type, advancing stamps, expected simulation rates and TF frames | RGB 19.71 Hz, depth 14.48 Hz, LiDAR 10 Hz, IMU 100 Hz, odom 29.41 Hz; message/frame checks recorded in `docs/sensor-contracts.md` and P2 evidence | PASS in named Phase B run; not every launcher runtime-regressed. |
| Wheel odometry P3-01 | Compare wheel-derived motion against simulator truth | Three synchronized straight/turn trials, calibrated effective track | Frozen position/yaw gates | Calibrated 1.03 m profile: straight error 0.0019–0.0038 m; turn error 0.0879–0.0965 rad. Original 0.80 m setting failed; low-friction discrepancy recorded. | PASS for frozen simulation profile; real rover/low-friction generalization unqualified. |
| LiDAR SLAM P3-02 | Validate mapping, saved graph/map and GPS-free short-route pose correction | Headless scan/odom mapping, map reload and independent Gazebo truth comparison | Nonempty valid map; bounded short-route position/yaw error | 403/720 valid returns; map reload preserved 38,860 known / 29,468 unknown cells; position error 0.229877 m, yaw 0.000053 rad vs bounds 0.25 m / 0.15 rad | PASS with limited position margin and route scope. |
| A* planner/follower P4 | Check grid paths, occupancy policy, follower behavior | Unit fixtures, route baselines, safety fixtures | Feasible paths; follower stops on blocked/stale inputs | A* fixtures and P4-03 3 routes × 3 trials pass; Phase L live obstacle stop/replan/goal run documented | Planner fixtures PASS; live integration only in named Phase L runs. |
| Perception P5-02 | Evaluate active three-class model | Frozen scene-isolated held-out analog corpus, 749 records | Report class IoU and gate outcome | mIoU 0.605734; regolith 0.918081, rock 0.369137, shadow 0.529984; report marks gate pass | PASS for this dataset/checkpoint; not generalization proof. |
| P6 known-plane/crater calibration | Bound projection/elevation-risk error in seeded simulation | Three independent plane captures + one crater capture | Frozen projection, flat-risk, bowl-risk checks | PASS, four captures. Horizontal p95 0.0219 m (1.5–3 m), 0.0576 m (3–5 m); vertical p95 0.0096 m and 0.0152 m. Crater region 157 valid cells and crater-risk max 100. | PASS simulator-only; hardware and live ROS registration tolerance not closed. |
| P7-03 planner replan comparison | Compare weighted A* and D* Lite | Nine matched deterministic occupancy-grid pairs | D* Lite adopt only if all pass, ≥20% faster and route/cost gates pass | P10 report snapshot: weighted A* median 3.971 ms, D* Lite 23.592 ms; adoption rejected. Another retry record differs, so quote run ID when reporting. | Weighted A* retained; fixture scope only, not ROS/Gazebo latency. |
| P8-03 round trip | Demonstrate habitat–science–habitat mission and durable event recording | Three independent live simulation round trips, three legs each | Nine legs and three durable delivery events | 3/3 round trips; 9/9 completed legs; 3 durable deliveries on bound map | PASS for this map/profile; not broad mission reliability statistics. |
| P10-02 scenarios | Evaluate five predefined scenarios | Three deterministic grid attempts per scenario (15 total) | Pass each planner fixture and compare terrain/occupancy paths | 15/15 attempts pass; hazard fixtures: occupancy paths intersected 6/6, terrain-aware planned paths 0/6 | Fixture planner result only; no simulated physical clearance/contact count. |
| P10-03 metric report | Account for registered metrics and provenance | Report builder, source hashes, plots and schema | All metrics either supported or explicitly unavailable | `COMPLETE_WITH_LIMITATIONS`: 9 limited-scope values and 5 unavailable; source says 4/5 scenarios lack live mission runs | PASS report completeness, not full project qualification. |
| Phase L headless integration (2026-11-16) | Exercise the full sensor/perception/planning/mission chain in WSL software-rendered profile | Fresh ROS 2 Humble/Gazebo run with active three-class CUDA model and bounded 1.5 m demo goal | All runtime stages, mission evidence and clean shutdown pass | Project issue log reports all 23 runtime gates passed, including dynamic replanning, integrated goal, evaluation, mission demo, map save and shutdown | PASS for this named WSL headless profile; GUI, target-machine resource/restart and broader site-map acceptance remain separate. |
| Phase L obstacle-to-goal run (2026-11-16 recheck) | Exercise live ROS/Gazebo perception-cost-plan-follow-mission path | Recorded topic list and status/log samples | Real scan detection, changed route, goal and mission evidence | `OBSTACLE_DETECTED distance=1.06 bearing=0.05 returns=138`; `DYNAMIC_REPLAN_PASS revisions=12 motion=1.64 obstacle=1 obstacle_replan_latency_ms=2239.4`; `GOAL_REACHED distance=0.35`; mission status text begins `MISSION_DEMO_PASS` | PASS reported in named run artifacts; stale/incomplete per-input artifact requires independent qualification capture for strongest claim. |
| Phase L manual RViz mission runs (2026-11-19) | Verify an operator-selected goal through terrain plan, dynamic obstacle replanning, motion, goal, evaluation and mission completion | Two full GUI validation runs with 16 typed presentation feeds and map-update evidence | All 11 presentation-specific checks pass per run; final map save also passes | `20261119-164616-phase-l-view-4738`: manual goal, terrain plan, replan, integrated goal, evaluation, mission, nonzero input/output motion, active controller, live SLAM map update and typed graph all PASS; mission journal reports `map_updates=57`. `20261119-171329-phase-l-view-6711`: same gate set PASS; `map_updates=65`. Both logs also report final map save PASS. | PASS in two named WSL GUI simulation trials. This is not a success-rate estimate; other same-day attempts failed or were incomplete. Target-machine resource/restart qualification remains open. |
| Phase L WSL GUI replan timing (2026-11-19) | Record obstacle-detection to changed-route monitor timing during the two successful manual runs | Read `DYNAMIC_REPLAN_PASS` status from the two run folders | Observe and retain timing; no frozen target-platform limit | 56,754.8 ms and 75,902.8 ms respectively | Measured, high and variable in these runs. Monotonic interval starts at detector status receipt and ends at changed terrain path; excludes earlier acquisition/transport. Not directly comparable to the 2,239.4 ms 2026-11-16 recheck or offline planner timings. |
| Later Phase L interactive startup (2026-11-19) | Check launch, topic graph, RViz readiness and shutdown in `interview_fast` profile | Runs `20261119-204622-phase-l-lunar-design-29034`, `205254-...-30483`, `205925-...-32028`, `211105-...-34687` | Startup/typed graph pass; manual mission checks require operator goal and enough capture time | 23-step launcher overall PASS; last run says 16 typed feeds PASS and manual mission/map-update validation NOT EVALUATED. `211105` mission log later records `MISSION_DEMO_PASS`, but launch acceptance had already marked the interactive mission not evaluated. | Startup pass only. Do not count these as full validated mission runs. |
| Phase L incomplete/failed same-day attempts (2026-11-19) | Preserve failures and partial manual runs alongside passes | GUI attempts in `20261119-*phase-l-*` | Full gate set | Some runs failed goal selection, goal completion, map update or typed-topic checks; e.g. `20261119-142130-phase-l-view-24958` records mission status PASS but five presentation checks FAIL, and `20261119-161029-phase-l-view-518` has manual goal/plan/replan/motion but follower did not reach goal. | FAIL/INCOMPLETE; they are not hidden by the two complete passes. |
| Phase A–L full current regression | Verify all independent launch combinations on target workstation | Current complete A–L runtime suite | Each launch passes current tests and acceptance | No single current end-to-end A–L qualification package was established in inspected evidence; verification matrix explicitly limits broad regression | OPEN / NOT ESTABLISHED. |

## Metrics summary

| Metric | Value | Evidence/scope |
|---|---|---|
| Localization position error | 0.229877 m | One P3-02 short route, bounded simulation truth comparison. |
| Localization heading error | 0.000053 rad | Same short route; does not imply global accuracy. |
| Planned grid path length | See `evidence/runs/20261113-p1003/P10-03/report.md` by scenario | Planner path length, not rover-traveled arc length. |
| Mission completion | 3 live round trips, 9 legs, 3 durable delivery events | P8-03; one bound map/profile. |
| Terrain hazard path intersection | 0/6 terrain paths vs 6/6 occupancy paths | Deterministic planned grid route fixture; not physical rover entry. |
| Perception mIoU | 0.605734 | Held-out three-class corpus, 749 records. |
| Replan latency | Run-dependent; P10 report has 3.971 ms weighted A* median, 23.592 ms D* Lite median | Python deterministic planner measurement only. Obstacle-specific live status reported 2239.4 ms from detector status to changed path in one recheck; timing excludes prior acquisition/transport. Do not compare these as same latency. |
| Manual Phase L mission gate | 2 full gate passes in 2 individually named runs | 2026-11-19 WSL GUI trials; not a statistically meaningful reliability estimate, and failed/partial attempts occurred on the same date. |
| Live obstacle replan interval | 56.7548 s and 75.9028 s in the two manual passes | Detector-status receipt to changed terrain path; process monotonic clocks; WSL GUI profile; substantial run-to-run variability. |
| Wheel odometry accumulated distance | 3.276425 m and 3.914826 m in the two manual run reports | Whole-run odometry monitor accumulation, not an independent ground-truth trajectory or path-length error. |
| Sensor rates | RGB 19.71, depth 14.48, LiDAR 10, IMU 100, odom 29.41 Hz | Simulation-time Phase B audit. |
| Mission travel time | NOT MEASURED in P10 schema | No common simulation start/end duration for scenario set. |
| Trajectory length error | NOT MEASURED | Grid paths are not rover trajectories. |
| Collision count / physical clearance | NOT MEASURED | Instrumentation absent in cross-trial evidence. |
| Mission recovery outcome/rate | NOT MEASURED | No attempts/outcomes instrumented in cited P10 inputs. |
| Whole stack CPU/GPU | NOT MEASURED | Existing resource values measure model-only inference, not whole ROS/Gazebo stack. |

## Build and tests

Repository contains CMake package metadata, native C++ watchdog build logs, Python unit/contract suites, per-phase static validation scripts and run logs. Historical project documentation gives a Humble command `python3 -m pytest -q`; this documentation task did not rerun the suite. P10-03 report states its focused P10 subset passed 13/13, task board integrity and source checks passed at that dated run. Current full test result: **NOT RE-RUN for this documentation work**. Static source existence is not presented as execution evidence.

## Evidence paths to preserve

* Phase L recheck: `evidence/runs/terrain-goal-20261116-recheck/` (topic dump, map, odometry report/CSV, semantic/obstacle/cost/planner/mission status, logs).
* P8: `evidence/runs/p803-acceptance-final2-20261113/P8-03/summary.json`.
* P10: `evidence/runs/20261113-p1003/P10-03/report.md`, `report.json`, SVG plots.
* P3 short localization: `evidence/runs/20261103-p302/localization-loop-final/localization_truth_comparison.txt`.
* P5 held-out: `evidence/runs/20261112-p502-threeclass/P5-02/heldout-three-class-evaluation.json`.
* P6 calibration: `evidence/runs/20261117-p6-final-v3/summary.json`.
* P7 comparison: `evidence/runs/20261112-p703/P7-03/benchmark.json`, retries in `evidence/runs/20261116-p703-retry/`.
* 2026-11-19 complete manual GUI missions: `evidence/runs/20261119-164616-phase-l-view-4738/last_run.log` and `evidence/runs/20261119-171329-phase-l-view-6711/last_run.log`; each folder also retains mission/replan/map/odometry logs and saved PGM/YAML.
* 2026-11-19 later interactive startup-only profile: `evidence/runs/20261119-211105-phase-l-lunar-design-34687/last_run.log` and `mission.log`; the launcher marked mission/map-update proof not evaluated, while the later mission log contains a pass record.
* 2026-11-19 representative partial/failure evidence: `evidence/runs/20261119-142130-phase-l-view-24958/last_run.log`, `evidence/runs/20261119-161029-phase-l-view-518/last_run.log`, and `evidence/runs/20261119-145117-phase-l-view-29294/last_run.log`.

## Experiments required to strengthen claims

1. Run the documented repeated Phase L qualification on a clean supported Ubuntu/Humble/Fortress target; retain three trial CSVs, ROS input samples and summary hashes.
2. Instrument collision/contact events, footprint clearance, planned-vs-traveled path length, mission start/end simulation time and recovery attempts/outcomes.
3. Repeat localization on long and loop routes and under slip, sensor dropout, map reload and obstacle occlusion, retaining independent Gazebo truth.
4. Measure RGB-D projection error on live ROS messages across depth/range and mast pose; quantify map registration uncertainty.
5. Report whole-stack resource use and end-to-end age/latency under nominal and overloaded sensor/compute conditions.
6. Validate terrain risk thresholds against physical mobility outcomes rather than planner path intersection alone.
