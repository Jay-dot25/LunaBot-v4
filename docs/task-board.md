# LunaBot V4 task board

Statuses: P0-02/03 and P1-01/02/03 completed; P2-01/02/03 completed. P3-01 now passes three fresh frozen WSL Gazebo trials after a measured 1.03 m effective-track calibration; slip sensitivity and physical-hardware accuracy remain limitations. P3-02 SLAM passed its bounded accuracy gate; P3-03 registry contract completed and P8-03 later bound the Habitat/Science staging poses. P4-01/02/03 completed. P5-01 source audit completed; historical five-class P5-02 FAILED (test mIoU 0.310), while active three-class P5-02 (regolith, rock, shadow) PASSED frozen validation/held-out gates and P5-03 package plus live ROS integration PASS (365 CUDA frames; see Phase L evidence). P6-01/03 now pass the seeded Gazebo calibration gate using three known-plane runs plus one crater geometry run; physical hardware remains separate. P7-01/02 live Phase L planning/replanning pass; P7-03 matched benchmark passes all attempts but correctly rejects D* Lite under its frozen speedup gate. P8-03 completed three live round trips. P9-03 dashboard acceptance and P11-01 target resource limits are IN PROGRESS; P10 and P11-03 retain declared scope limitations. See each task row for evidence and exact open gates. V(ID) is proposed command python3 tools/verify_task.py --task ID --run-id <run-id>, available after P0-03. Freeze thresholds before trials.

Active scope update (2026-11-17): runtime output is three-class (regolith/rock/shadow); Phase L P5-03 model integration passed on CUDA with 365 frames, live mapping/cost/planning, dynamic replanning, goal completion and evaluation. P3-01 now passes three calibrated WSL Gazebo trials; hardware/slip limits remain open. Seeded P6 calibration passes frozen gate v2 in three known-plane runs and a separate crater geometry run; see `docs/p6-calibration-plan.md` and `evidence/runs/20261117-p6-final-v3/`. P9-03 target dashboard acceptance and P11-01 target resource limits remain open.

This update supersedes the older summary-line statements that P5-03 ROS inference and P6/P7 live ROS exercise were unverified.

| ID | Priority | Dependencies | Affected files | Acceptance and verification |
|---|---|---|---|---|
| P0-01 | P0 | — | Read-only repo, prompt, environment | Map all supplied requirements; note missing separate plan. Audit review. |
| P0-02 | P0 | P0-01; START | Seven planning docs | 36 IDs/deps consistent. Cross-document validator. |
| P0-03 | P0 | P0-02 | tools/verify_task.py, tests, config/verification | Failing fixture detected; preserve evidence. V(P0-03). |
| P1-01 | P0 | P0-03 | Gazebo metadata, bringup package/shared launch | colcon build, rosdep; headless no RViz. V(P1-01). |
| P1-02 | P0 | P1-01 | Launchers/reset scenarios | 3 headless drive-stop-reset cycles; no orphan. V(P1-02). |
| P1-03 | P1 | P1-02 | SDF/config/simulation | Physics, wheels, suspension, collisions, landmarks verified. V(P1-03). |
| P2-01 | P0 | P1-02 | Model/bridge/sensor config/TF | Frame, calibration, timing and unique TF ownership pass. V(P2-01). |
| P2-02 | P0 | P2-01 | control_odometry.py/safety | Single cmd_vel publisher; e-stop deadline passes. V(P2-02). |
| P2-03 | P0 | P2-02 | Health adapters/fault fixtures | Stale sensor, clock jump, process failure stop; restart held. V(P2-03). |
| P3-01 | P0 | P2-01 | scripts/launch-b.sh, scripts/wasd_teleop.py, tools/capture_gazebo_truth.cc, tools/analyze_odom_truth.py, tools/run_odom_qualification.py, docs/odometry-qualification.md, evidence/runs/20261103-p301/, evidence/runs/20261116-p301-retry/ | Three timestamped (<0.02 s) trials; straight error <= max(0.10 m,10%); 1.5 rad turn yaw error <=0.10 rad; fresh origin <=0.02 m/rad with monotonic Odometry stamps; odom->chassis runtime TF lookup/source ownership; controlled low-friction discrepancy recorded. Frozen 2026-11-16 retry PASS: 3/3 straight and turn trials with 1.03 m effective track; slip sensitivity remains disclosed and physical-hardware accuracy is unqualified. WSL profile uses software rendering, a 2.0 s health-report scheduling timeout, and the 0.5 s actuator watchdog. Evidence: evidence/runs/20261116-p301-retry/qualification-fixed/. V(P3-01). |
| P3-02 | P0 | P3-01,P2-03 | config/slam_toolbox_phase_c.yaml; config/slam_toolbox_localization.yaml; scripts/launch-c.sh; src/lunabot_gazebo/models/lunabot_v4/model.sdf; src/lunabot_gazebo/worlds/lunar_world.sdf; docs/slam-qualification.md; evidence/runs/20261103-p302/ | COMPLETED. Headless mapping: 403/720 valid LiDAR returns; map 117x584 at 0.05 m/cell. Saved map reload preserves 38,860 known / 29,468 unknown cells; saved graph reloads fresh with match_type=3. Truth error 0.230 m / 0.000053 rad meets <=0.25 m / <=0.15 rad; 0.10 m initial-pose offset scan-matches back. Verify: bash -n scripts/launch-c.sh; from /tmp run ign sdf -k on the absolute world/model SDF paths; inspect evidence/runs/20261103-p302/scan_summary.txt, saved_map_reload.txt, localization_truth_comparison.txt, and localization-loop-final/last_run.log. See docs/slam-qualification.md. |
| P3-03 | P1 | P3-02,P1-03 | config/waypoints.json, scripts/waypoint_registry.py, tests/test_waypoint_registry.py, docs/waypoints.md | Habitat/Science IDs registered against pinned map; invalid map/frame and unbound-goal dispatch rejected. V(P3-03). |
| P4-02 | P0 | P4-01,P2-03 | scripts/terrain_path_follower.py, scripts/launch-l.sh, tests/test_terrain_follower_safety.py, evidence/runs/20261104-p402/ | Eight fixtures: free path remains clear; occupied route and clearance stop; rotated grid lookup; empty/stale plan and missing/stale required obstacle map publish zero; five blocked ticks never reverse. Focused tests, Python compile and bash syntax pass. Live Phase L now passes bounded goal completion and obstacle stop/replan on the active three-class stack. Evidence: evidence/runs/20261116-threeclass-phase-l-retry2/. |
| P4-03 | P1 | P4-02 | config/navigation_baselines.json, tools/run_navigation_baselines.py, tests/test_navigation_baselines.py, evidence/runs/20261104-p403/P4-03/ | Three deterministic occupancy routes; 3 attempts per route; retain all 9 attempt records including failures. Report success, path length, path cells, estimated clearance and planning time. Verify unittest, evaluator exit/result, output JSON integrity and board verifier. Planner-level only; Gazebo motion trials remain later runtime work. V(P4-03). |
| P5-01 | P1 | P0-02 | config/perception/classes.json, config/perception/sources.json, config/perception/splits.json, docs/perception-dataset.md, tools/validate_perception_manifest.py, tests/test_perception_manifest.py, evidence/runs/20261104-p501/P5-01/ | Five canonical classes plus ignore ID; sources with CC0/CC-BY basis, attribution, and acquisition notice checks; 8/2/2 complete-terrain split policy with shared namespace preventing cross-source leakage; annotation precedence and ignore rules. Validator and 5 unit tests pass. No external dataset files or new masks are copied; materialization/class-frequency audit is a P5-02 prerequisite. V(P5-01). |
| P5-02 | P1 | P5-01 | tools/materialize_perception.py, tools/project_crater_candidates.py, tools/materialize_marsseg_train.py, tools/train_perception.py, tools/evaluate_perception_checkpoint.py, tools/perception_metrics.py, tests/test_materialize_perception.py, tests/test_project_crater_candidates.py, tests/test_materialize_marsseg_train.py, tests/test_perception_training.py, docs/perception-dataset.md, docs/verification-matrix.md, docs/known-issues.md, evidence/runs/20261107-p502/P5-02/ | Pair NASA PNGs to POLAR-Sim polygons; remap observed labels and preserve unknown as ignore; audit complete split manifest; project explicit lidar-XY crater ROIs to left-camera candidate masks without mutating source masks or trainer manifest; materialize approved MarsSeg MSL training rows only with CC-BY revision/file SHA provenance and unknown classes ignored; seeded five-class training, frozen gates before held-out test, and SHA-256/per-class IoU/mIoU/confusion. Verify materializer/projector/trainer/manifest suites and py_compile; audit source class support, pairs, dimensions, and leakage; review crater overlays. Result (2026-11-08): corpus audit PASS, 30-epoch CUDA training completed with best epoch 23, validation mIoU 0.302 (gate FAIL), test mIoU 0.310 (gate FAIL). Test per-class IoU: regolith 0.898, bedrock 0.001, rock 0.228, crater 0.003, shadow 0.420. Frozen gates remain mIoU >=0.50 and every class IoU >=0.25; they were not weakened. The model is not qualified. Validation and test confusion show poor transfer for bedrock/crater; do not integrate this model. Evidence and full provenance: evidence/runs/20261107-p502/P5-02/. V(P5-02). |
| P5-03 | P1 | P5-02,P2-01,P1-01 | tools/export_verify_p502_threeclass.py, scripts/p502_threeclass_inference.py, evidence/runs/20261112-p503/P5-03/ | Package acceptance PASS: exact export parity, stale-to-unknown behavior, confidence, CPU/CUDA measures. Phase L ROS integration PASS: 365 live three-class CUDA frames with inference/input-age metrics, semantic confidence reached mapper, terrain/cost/planner pipeline passed, dynamic replanning and bounded goal/evaluation passed. Evidence: evidence/runs/20261116-threeclass-phase-l-retry2/. V(P5-03). |
| P6-01 | P0 | P2-01,P3-02 | scripts/semantic_terrain_mapper.py, scripts/semantic_projection.py, tests/test_semantic_projection.py, docs/p6-calibration-plan.md | Fixture, Phase L integration, and frozen seeded Gazebo qualification PASS. Three independent known-plane captures pass both configured bands; worst horizontal p95 is 0.0219 m near / 0.0576 m far, and worst vertical p95 is 0.0096 m near / 0.0152 m far. Recorder aligns masks to the nearest depth frame and registers with Gazebo's settled full 3D pose. Evidence: evidence/runs/20261117-p6-final-v3/. V(P6-01). |
| P6-02 | P1 | P6-01,P5-03 | scripts/semantic_layers.py, scripts/semantic_terrain_mapper.py, scripts/terrain_segmentation.py | Rolling map, confidence, age, TTL and recenter fixtures pass. Active three-class confidence is now used; regolith maps to traversable, rock to obstacle, shadow/unknown to unobserved. Phase L live mapping and 22 map updates passed. Mission-wide retained coverage remains intentionally outside the bounded rolling grid. Evidence: evidence/runs/20261112-p602/P6-02/ and evidence/runs/20261116-threeclass-phase-l-retry2/. V(P6-02). |
| P6-03 | P0 | P6-02 | scripts/terrain_geometry.py, scripts/terrain_costs.py, scripts/terrain_cost_mapper.py, scripts/semantic_terrain_mapper.py, docs/p6-calibration-plan.md | Geometry/cost fixtures and Phase L live elevation/cost-path integration pass. Seeded Gazebo geometry qualification PASS: flat risk p95 is 2 slope points / 0 crater points; crater-center coverage is 157 cells, risk max 100, and slope/roughness/crater p95 errors are 9/14.2/0 points. Slope uses a 1 m local plane fit; crater risk also samples a sparse 3 m rim for broad bowls. Evidence: evidence/runs/20261117-p6-final-v3/. V(P6-03). |
| P7-01 | P0 | P6-03,P4-01 | scripts/terrain_aware_planner.py, scripts/terrain_plan.py, tests/test_terrain_plan.py | Weighted A* and deterministic contracts pass: prefer lower-cost routes, reject blocked/out-of-bounds endpoints, prevent diagonal corner cutting, configure unknown-cell policy, rotated-grid conversion, and content/geometry map revisions. 8/8 focused tests pass; Phase H static checks and Phase L live terrain-plan revision updates pass. Evidence: evidence/runs/20261112-p701/P7-01/ and evidence/runs/20261116-threeclass-phase-l-retry2/. V(P7-01). |
| P7-02 | P0 | P7-01,P4-02 | scripts/terrain_path_follower.py, scripts/dynamic_replan_monitor.py, tests/test_terrain_follower_safety.py, tests/test_dynamic_replan_monitor.py | Route safety and timing instrumentation pass; obstacle-intersecting routes stop before motion resumes on a changed path. Phase L live gate passed with 2 revisions, obstacle observed, and 20,707.4 ms obstacle-to-changed-plan timing; this is measured performance, with no frozen latency limit. Evidence: evidence/runs/20261112-p702/P7-02/ and evidence/runs/20261116-threeclass-phase-l-retry2/. V(P7-02). |
| P7-03 | P1 | P7-02,P4-03 | config/p7_replan_benchmark.json, scripts/dstar_lite.py, tools/run_replan_benchmark.py, tests/test_dstar_lite.py, docs/p7-replan-benchmark.md | COMPLETED: original and fresh 2026-11-16 matched runs each retained 18 successful attempts with equal route costs. Fresh weighted A* median replan 3.105 ms; D* Lite 20.659 ms, so the frozen 20% speedup adoption gate fails and weighted A* remains in production. D* Lite is benchmark-only. V(P7-03). Evidence: evidence/runs/20261112-p703/P7-03/ and evidence/runs/20261116-p703-retry/benchmark.json. |
| P8-01 | P0 | P3-03,P4-02 | scripts/phase_l_mission.py, scripts/mission_lifecycle.py, tests/test_phase_l_mission.py, docs/phase-12-launch-l.md, tools/validate_phase_l.py | COMPLETED (2026-11-12) for mission lifecycle fixtures: valid map/goal/path transitions WAITING_FOR_MAP → WAITING_FOR_GOAL → READY → ACTIVE → COMPLETED; malformed map/frame/resolution/value and goal poses are rejected into FAILED; stable goal IDs appear in status and changed goals clear prior goal-scoped completion evidence. Focused lifecycle tests and Phase L P8-01 static checks pass. Live Phase L mission runtime remains unverified. V(P8-01). Evidence: evidence/runs/20261112-p801/P8-01/. |
| P8-02 | P0 | P8-01 | scripts/mission_event_store.py, scripts/mission_lifecycle.py, scripts/phase_l_mission.py, scripts/terrain_path_follower.py, scripts/launch-l.sh, tests/test_phase_l_mission.py, tests/test_terrain_follower_safety.py, docs/phase-12-launch-l.md, tools/validate_phase_l.py | COMPLETED (2026-11-12) for event/recovery fixtures: SQLite WAL/FULL-sync event journal deduplicates delivery IDs across a reopened process; pause/resume/cancel, blocked/failure recovery, E-stop/reset, and invalid-map recovery are explicit; Phase L follower emits zero unless mission state is fresh and ACTIVE. 7/7 P8-02-focused lifecycle/follower cases pass. Live ROS service and rover stop timing remain unverified. V(P8-02). Evidence: evidence/runs/20261112-p802/P8-02/. |
| P8-03 | P0 | P8-02,P7-02 | scripts/p8_roundtrip.py, scripts/terrain_path_follower.py, tools/bind_p8_waypoints.py, tools/run_p8_acceptance.sh | COMPLETED (2026-11-13): qualified and bound Habitat/Science staging poses; three independent live Habitat–Science–Habitat trials passed, with nine exact goal completions and three durable SQLite delivery rows attached to their return goals. Runner endpoint preflight now applies the planner's obstacle inflation, goals stay in the SLAM map frame, and the follower heartbeat uses steady time against the wall-clock watchdog. V(P8-03). Evidence: evidence/runs/p803-fixed-r64/phase-l/ and evidence/runs/p803-acceptance-final2-20261113/P8-03/. |
| P9-01 | P1 | P0-02,P2-01 | docs/dashboard-adr.md, config/dashboard/options.json, config/dashboard/contracts.json, tools/validate_dashboard_contract.py, tests/test_dashboard_contract.py | COMPLETED (2026-11-13) for evidence-based comparison of three stacks and six-view topic/control contract. Foxglove Bridge/client selected for prototype trials, not release. Latency/CPU/RAM/VRAM are explicitly unmeasured here and remain P9-03 gates. Focused validator and contract tests pass. Evidence: evidence/runs/20261113-p901/P9-01/. V(P9-01). |
| P9-02 | P1 | P9-01,P5-03,P6-03,P7-02 | scripts/dashboard_telemetry_adapter.py, scripts/dashboard_telemetry.py, config/dashboard/adapter.json, src/lunabot_bringup/launch/dashboard_telemetry.launch.py, docs/dashboard-adapter.md, tests/test_dashboard_telemetry.py | COMPLETED (2026-11-14) for optional read-only ROS metadata adapter covering every views 1-5 source exactly once. It preserves original feed stamps/frames, reports publisher provenance and source/receive ages, and marks absent or type-mismatched sources unknown. 12 focused tests pass. Live ROS/Gazebo launch could not be checked because WSL returned Wsl/E_ACCESSDENIED; runtime/UI coverage remains unverified. Evidence: evidence/runs/20261114-p902/P9-02/. V(P9-02). |
| P9-03 | P1 | P9-02,P8-02 | scripts/dashboard_control_gateway.py, scripts/dashboard_control_policy.py, config/dashboard/controls.json, src/lunabot_bringup/launch/dashboard_controls.launch.py, docs/dashboard-controls.md, docs/dashboard-performance.md | IN PROGRESS (2026-11-14): guarded view 6 gateway and request-only contract are implemented. ROS build passed; telemetry adapter published a live snapshot; gateway dispatched PAUSE/RESUME (mission lifecycle was already FAILED and rejected those transitions) and confirmed ESTOP/RESET_ESTOP reached safety services (96 ms and 10 ms request-to-service acknowledgments). Headless Phase L source graph started, but terrain planning failed its occupancy/synchronized-slope gates. User-authorized hosted Foxglove sign-in and local Rosbridge telemetry connection work. On continuation, the authorized WSL path restored the ROS graph; 36/36 required read-only topics were present and all 27 adapter-typed topics matched, recorded in `evidence/runs/20261114-p903/P9-03/followup/live-ros-topic-inventory.csv`. Telemetry exposes explicit fresh/stale/unknown states. The unsaved personal Foxglove draft now has three exploratory tabs, but it still does not cover the six groups or all required topics, and the depth view is not verified. No goal, safety, or control command was sent in this continuation. Four 300-second resource trials recorded host CPU/memory, stack CPU/RSS and RTF. A separate five-minute Foxglove Performance-sidebar capture measured app-level main-thread FPS (p50 30, p95 37), pipeline rate (p50 17.8, p95 18.9), event-loop latency (p50 66.9 ms, p95 85.6 ms), and JS heap (p50 254.2 MB, p95 314.5 MB). The actual release target resolution is unspecified, so this is diagnostic rather than target acceptance. Three camera render-probe 0.1.1 diagnostic trials ran for 300–335 seconds each: one at 480×667 and two at 1536×864 CSS viewport overrides. Painted FPS was 4.10, 4.30, and 4.16; receive-to-paint p95 was 66.0, 58.3, and 61.9 ms; estimated source-to-paint wall-age p95 was 1,525.6, 1,513.0, and 1,378.3 ms. Source ran near 0.02× real time; these are not native desktop or target acceptance results. The layout still lacks full contract coverage; client CPU/VRAM, reconnect timing, doubled traffic, and frozen target limits remain open. Evidence: `evidence/runs/20261114-p903/P9-03/followup/summary.md`, live topic inventory, and three camera-render-probe-samples-v0.1.1*.csv files. V(P9-03). |
| P10-01 | P0 | P0-03,P3-01 | config/evaluation/metrics.json, tools/validate_evaluation_contract.py, tests/test_evaluation_contract.py, docs/evaluation-contract.md | COMPLETED (2026-11-13): metric contracts cover known references, aggregation, gate state and evidence source; Gazebo truth is isolated to standalone capture/analysis tools. Validator scans operational source paths and rejects truth-interface markers. P10-03 added path-length, travel-time, and terrain-exposure coverage to the contract. Focused tests, py_compile, git diff --check, and board-integrity validation pass. Generic task-verifier evidence write is blocked by Windows Python access to the WSL UNC path; task-specific evidence is recorded. Evidence: evidence/runs/20261113-p1001/P10-01/ and evidence/runs/20261113-p1003/P10-03/. V(P10-01). |
| P10-02 | P1 | P10-01,P7-02,P8-03 | config/evaluation/scenarios.json, tools/run_scenarios.py, tests/test_evaluation_scenarios.py, docs/evaluation-scenarios.md | COMPLETED (2026-11-13) for reproducible scenario/planner fixtures: five frozen scenarios x three seeded trials (15/15 pass); occupancy and terrain planners are matched, changed safe replanning is required for the unexpected-obstacle case, and every attempt is retained. P8-03 separately supplies three live outbound-return missions (9 legs, 3 durable deliveries); not rescored by P10 runner. Four non-roundtrip scenarios remain planner-level only; motion/sensor/runtime metrics are explicitly unavailable. Evidence: evidence/runs/20261113-p1002/P10-02/. V(P10-02). |
| P10-03 | P1 | P10-02,P5-02,P7-03 | tools/build_evaluation_report.py, config/evaluation/report.schema.json, tests/test_evaluation_report.py, evidence/runs/20261113-p1003/P10-03/ | COMPLETED (2026-11-13): report accounts for all 14 metric-catalog entries, with 9 limited-scope results and 5 unavailable values with explicit reasons. Cross-trial inputs have SHA-256 provenance; four SVG plots cover scenario pass rate, planner path length, held-out IoU, and P7 replan time. 4/4 focused report tests pass. Status is COMPLETE_WITH_LIMITATIONS; the report does not claim live acceptance for four planner-only scenarios. Evidence: evidence/runs/20261113-p1003/P10-03/. V(P10-03). |
| P11-01 | P0 | P9-03,P10-02 | config/profiling/resources.json, tools/profile_runtime.py, docs/resource-profiling.md | IN PROGRESS (2026-11-14): added a CPU/RSS/GPU/RTF profiler and provisional WSL2 machine profile. A 60-second headless stack sample recorded stack CPU p95 60.35% of host, RSS p95 1,325 MiB, RTF 0.316, GPU use 0; no latency samples. Target limits remain null, so thresholds and safe-overload behavior were not evaluated. P9-03's six-view target-machine benchmark is still a dependency. Evidence: evidence/runs/20261114-p1101/P11-01/. V(P11-01). |
| P11-02 | P1 | P1-01,P4-03,P8-03 | A-L launchers/package launch | COMPLETED (2026-11-14): full colcon build passed for lunabot_gazebo and lunabot_bringup; registered ament package/launcher test passes and colcon reports 5 tests, 0 failures. Full Python suite: 191 tests, 0 failures, 2 environment skips (optional pyarrow and training runtime). Aggregate Phase A-L validator passes 125/125 checks. Installed reusable package launch started the world-state service headless, launched no GUI/RViz processes, and shut down cleanly. Evidence: evidence/runs/20261114-p1102/P11-02/. V(P11-02). |
| P11-03 | P0 | P10-03,P11-01,P11-02 | Acceptance/docs/demo | All requirements evidenced; restart/full mission pass. V(P11-03). |

Runtime gates require real ROS/Gazebo evidence. Static checks do not complete runtime acceptance. GUI tests never gate headless acceptance.
P1-01 evidence: evidence/runs/20261103-p101/P1-01/verification.txt. Runtime remains unverified. P0-03 corrected JSON evidence: evidence/runs/20261103-p003-jsonfix/P0-03/manifest.json.

P1-02 Completed: three headless cycles passed startup, move/turn, settled stop, fresh-process reset and shutdown. Evidence: evidence/runs/20261103-p102/P1-02/summary.json. In-process reset is not tested.

P2-01 evidence: evidence/runs/20261103-p201/P2-01/summary.json. Live Phase B, A/B/C static gates and A-L source/syntax pass. See docs/sensor-contracts.md for rates/ownership and P6-01 registration boundary.

P2-02 Completed: isolated DDS + live Phase B safety tests, Phase A safety-routing smoke and A/B/C static checks PASS. Evidence: evidence/runs/20261103-p202/P2-02/summary.json. P2-03 sensor/localization/process-failure gating remains open; see docs/safety-control.md.

P2-03 Completed: 26 health/protocol fixtures, live process/bridge/clock/localization fault gates, native watchdog package build, A-L static gates and bounded clean teardown PASS. Evidence: evidence/runs/20261103-p203/P2-03/summary.json. See docs/safety-faults.md for scope and earlier failures. P1-03 and P3-02 are complete. P3-03 registry/frame validation is complete; authoritative Habitat/Science goal poses remain required before mission dispatch.


Historical P3-01 qualification using the 0.80 m track failed the frozen
accuracy gate: turn accuracy failed 3/3, straight accuracy failed 2/3, and
controlled low friction exposed 0.947 m wheel-ground error. A measured 1.03 m
effective track then passed three fresh trials against the same gates. Keep
the original failure and slip evidence; simulated pass does not qualify
physical hardware. See docs/odometry-qualification.md and
evidence/runs/20261116-p301-retry/qualification-fixed/.


P3-02 Completed with bounded localization accuracy. OGRE2/software-rendered headless GPU LiDAR produces valid scans; mapping, occupancy save/reload including unknown-cell preservation, pose-graph serialize/fresh reload, map TF, and isolated-truth localization checks passed. Measured truth error 0.230 m / 0.000053 rad is within the qualification bound (0.25 m / 0.15 rad) with limited position margin. See docs/slam-qualification.md and evidence/runs/20261103-p302/. P1-03 and P3-02 are complete; P3-03 registry/frame validation is complete with destination binding limitations recorded.

P1-03 Completed (2026-11-04): static SDF, physics, wheel, collision-mesh,
contact-system, and four landmark collision-geometry audit passed. Headless
runtime verified all six wheel joints rotate, valid LiDAR returns, and
wheel-driven truth movement. The original passive hinge settings limited
articulation to less than 0.0007 rad. A controlled candidate comparison
identified refined hinge dynamics; the final source-model headless run measured
0.27-0.28 rad rocker and 0.15-0.17 rad bogie spans over about 0.28 m of terrain
elevation change, with joints inside limits. A bounded approach stopped just
short of the forward obstacle collision face while wheels continued turning;
no penetration occurred, though no direct contact sensor was installed.
Evidence: evidence/runs/20261104-p103/. No RViz process was required.

P1-03 final qualification: static physics, wheel, suspension joint, collision
geometry, and four collision-landmark checks pass. Baseline rocker/bogie
resistance yielded less than 0.0007 rad joint span on the rough-transect trial.
The refined passive profile (rocker damping/friction 10/1; bogie 5/0.5) was
applied to the existing model and reproduced headlessly: 0.27-0.28 rad rocker
span, 0.15-0.17 rad bogie span, and no joint reached its limit on a route over
a roughly 0.28 m collision-mesh elevation change. A separate low-speed forward
approach stopped at the existing obstacle boundary while drive wheels continued
rotating, with no rover pose penetration. The unchanged world contains four
static landmarks with collision shapes; the forward obstacle also produced a
1.513 m LiDAR return. RViz was not required. Evidence: evidence/runs/20261104-p103/.

P3-03 registry contract completed: Habitat and Science IDs are registered against
the pinned P3-02 map identity and map frame. Six unit tests pass. Valid identity
loads; wrong caller map ID/frame, mismatched waypoint frame, and attempts to
require/dispatch unbound goals reject. At that phase the Habitat landmark was
outside the qualified map and both IDs remained unbound. P8-03 later qualified a
map covering both simulation staging sites and bound their verified poses.
Evidence: docs/waypoints.md, evidence/runs/20261104-p303/verification.json, and
evidence/runs/p803-fixed-r64/phase-l/.

P4-01 Completed (2026-11-04): six behavioral fixtures pass for wall-gap
reachability, rotated-grid conversions, circular footprint inflation, diagonal
corner clearance, out-of-bounds snapping rejection, and boundary goal rejection.
The existing planner was preserved with two targeted fixes: diagonal steps now
require both adjacent cardinal cells safe; out-of-bounds cells cannot be
reinterpreted as in-map goals. The original Phase D report was 151/152 because
an inherited Phase C check overmatched bounded diagnostic pipe/head commands;
the latest static report is 152/152 and supersedes that historical result. See
evidence/phase-d-launch-d/static_validation.txt and
evidence/runs/20261104-p401/summary.json. P4-02 implementation and unit-fixture
acceptance is recorded below; live timing remains P7-02.


P4-02 implementation and fixture acceptance Completed (2026-11-04): the existing
terrain follower now stops on empty or stale plans, waits for a fresh required
obstacle map in Phase L, stops if that map becomes stale, and holds zero command
when the remaining path intersects occupied cells plus a conservative footprint
margin. It has no reverse or spin recovery; motion resumes only after the cause
clears and fresh inputs arrive. Eight follower fixtures and six preserved A*
fixtures pass; Python and launch syntax checks pass. This is implementation and
unit verification only: no Phase L headless mission or detection-to-stop latency
run was performed; P7-02 adds deterministic latency instrumentation, while live
timing remains unverified. Evidence:
evidence/runs/20261104-p402/P4-02/summary.json.


P7-02 implementation and deterministic fixture acceptance Completed
(2026-11-12): when the follower sees that its active route intersects a fresh
obstacle grid, it invalidates the local route and publishes zero in that control
tick before it can issue another motion command. Repeated copies of the
invalidated route remain rejected until a changed route arrives. Status records
map-receipt-to-stop and invalidation-to-new-plan latency; the dynamic replan
monitor records first detected-obstacle-to-changed-plan latency and requires
that measurement before its obstacle-gated PASS. These are deterministic
instrumentation/fixture results; a live Phase L obstacle-placement trial is
still needed to qualify ROS/Gazebo timing. Evidence: evidence/runs/20261112-p702/P7-02/.


P4-03 planner-baseline implementation and trials Completed (2026-11-04):
wall-gap detour, bounded corridor, and alternating-barrier routes were each
evaluated three times using the existing A* implementation. All 9/9 attempts
passed; every attempt record is retained with outcome, path length, path cells,
minimum grid clearance estimate, and planning time. A failed initial wall-gap
geometry trial is also retained, then the route was widened to respect the
planner footprint inflation and map-edge margin. This is an occupancy-grid
planner baseline, not a Gazebo motion or mission result. See
evidence/runs/20261104-p403/P4-03/.


P7-03 planner comparison passed its benchmark execution on both the original
and fresh 2026-11-16 matched runs: all 18 attempts succeeded with equal path
costs. The fresh retry measured weighted A* at 3.105 ms median replan and D*
Lite at 20.659 ms; D* Lite failed the frozen 20% adoption gate, so production
remains weighted A*. This is a Python grid benchmark, not a ROS/Gazebo result.
Decision and limits: docs/p7-replan-benchmark.md; fresh evidence:
evidence/runs/20261116-p703-retry/benchmark.json.


P5-01 dataset audit and protocol Completed (2026-11-04): the five-class mask
ontology, source rights register, annotation policy, and terrain-grouped split
plan are versioned. The validator checks class IDs, allowed license identifiers,
attribution, file-notice gates, split completeness, and scene leakage across
shared source namespaces. Five unit tests pass. No external dataset has been
downloaded and no new pixel masks have been created; actual source-file notice
review, sample inventory, per-class counts, and annotations remain before P5-02
training. Evidence: evidence/runs/20261104-p501/P5-01/summary.json.

### P5-02 resumed audit (2026-11-06)

This update supersedes the earlier statement that no train crater labels exist. The delegated Terrain05 PosA annotations were merged into 68 separate training masks, adding 850,772 crater pixels while retaining known rock/shadow/unknown pixels. All validation/test masks remain unchanged; the 131 held-out crater candidates remain proposals. The merge tool now rebases image and unchanged-mask paths to absolute paths so the relocated output manifest resolves correctly. The earlier external merged CSV had broken relative references and should not be used. The regenerated manifest is at evidence/runs/20261106-p502/P5-02/rebased-merge/manifest.csv (local derived data, ignored by Git).

All 29 perception materializer/projector/merger/trainer/manifest fixtures pass, including the relative-path regression. Compilation and git diff --check pass. The reproducible audit and measured counts are in evidence/runs/20261106-p502/P5-02/audit_continuation.py and continuation-audit.json. Five-class qualification remains open: validation/test have no accepted bedrock or crater support. Do not invent bedrock from discrete rocks or move Mars training frames into held-out splits. No qualification training, frozen gates, or held-out score is claimed.

### P5-02 annotation repair retry (2026-11-06)

Full-resolution review produced two conservative partial crater annotations: Terrain11 PosA `11_A_off_180_L_0032.png` has 551 geometry-supported reviewed crater pixels; Terrain12 PosA `12_A_off_180_L_0032.png` has 2,201. The corresponding unreviewed crater rectangles are ignore, preventing known ambiguous rock/shadow regions from being promoted to crater. The review applies only to those exact frames and hashes; other illumination/exposure variants remain unaccepted. These small patches are partial annotations and do not establish representative crater accuracy. The new manifest is a diagnostic retry corpus, not a completed qualification corpus. Review definitions, hashes and limitations are in `evidence/runs/20261106-p502/P5-02/heldout-patch-review.json`; reproducible materialization and retry scripts accompany it.

Bedrock remains a data-acquisition dependency. Reinspection of the staged held-out scenes did not establish continuous exposed consolidated substrate. An external-source search found the Concordia/Mission Control Lunar Analogue publication with exposed-bedrock examples, but no accessible dataset download in its linked public code repository; the user has no copy. MADMAX requires registration and describes academic use; Devon Island did not establish a compatible licensed bedrock mask corpus. No alternative source was silently approved. See `bedrock-source-search.json` for exact references and findings. No bedrock labels were fabricated and no qualification training was run. The retried preflight result is retained separately in `retry-preflight.json`.


### P5-02 independent corpus review (2026-11-07; supersedes readiness claims)

The inherited Terrain05 crater merge is quarantined pending correction. Inspection of full-resolution 05_A_off_180_L_0032.png and the 12-frame training montage shows accepted regions overlapping prominent positive rocks, with masks propagated through saturated and dark exposures. Successful geometric projection and unit tests do not establish semantic correctness. The restarted CPU run was stopped before held-out testing; its outputs must not be used as qualification evidence. NASA documents small negative features in Terrain05, but that does not validate these particular regions.

The frozen 256px cache also contains 276 identical completely white RGB frames across train (161), validation (59), and test (56). Remove these uninformative frames from all splits rather than relocating them. A first audit additionally keyed lineage by image content and therefore conflated same-image/different-mask entries; that auditor bug is corrected to use the cache key derived from the original path. The original audit report is retained for traceability and will be superseded by the corrected report.

The previously acquired Apollo and BASEPROD additions provide sparse bedrock evaluation support; claims that bedrock is wholly absent are stale for the combined corpus. They remain limited in domain, scale and scene count. A separate NASA POLAR Traverse training-only crater acquisition is being prepared; existing held-out Stereo terrain assignments are preserved.

### P5-02 training and qualification result (2026-11-08)

The corrected scene-separated corpus audit passes with 5,417 train, 746 validation and 749 test rows; five-class support exists in every split and no identical RGB image crosses splits. Frozen gates remain mIoU >=0.50 and per-class IoU >=0.25. Source, split, data and checkpoint hashes plus acquisition/annotation lineage are retained under `evidence/runs/20261107-p502/P5-02/`.

Thirty CUDA epochs completed on the RTX 3050; validation selected epoch 23 (mIoU 0.302). The held-out result is mIoU 0.310: regolith 0.898, bedrock 0.001, rock 0.228, crater 0.003, shadow 0.420. The frozen qualification gate FAILED and the model is NOT QUALIFIED. Do not integrate this model. Full confusion matrices, per-class support, run identities, hashes and training history are in `training-reviewed-v2-gpu/metrics.json` and `validation-metrics.json`.

The first held-out scoring attempt ran inference but hit an `UnboundLocalError` before writing metrics because `summarize_confusion` was imported only inside the training loop. No model, gate or data changes followed. The import was moved to module scope, and a checkpoint-only evaluator verified the saved run identity, selected checkpoint and frozen input/data hashes before writing the result. The failed attempt and recovery command/log are retained as `training-heldout.log`, `training-command-heldout-recovery.json` and `training-heldout-recovery.log`.

### P5-02 supplied checkpoint evaluation (2026-11-12)

Imported the 40-epoch ResNet34 U-Net EMA export from `full_output.zip` and evaluated it on all 749 held-out rows with its archived grayscale preprocessing, 448px input, flip-averaged inference, and shadow logit bias. Validation mIoU was 0.3480. Held-out mIoU was 0.3737; per-class IoU was regolith 0.9142, bedrock 0.1004, rock 0.3572, crater 0.0000, shadow 0.4966. Frozen gates failed for mIoU, bedrock, and crater. The run does not qualify for P5-03. Evaluation script, confusion matrix, and checkpoint hashes are in `evidence/runs/20261112-p502/P5-02/`.

### P5-02 active three-class checkpoint qualification (2026-11-12)

Evaluated the trained ResNet34 U-Net EMA export from `all_outputs.zip` against the frozen three-class gates (`p502-three-class-v1-20261112`). The checkpoint loaded strictly and was scored with the archived grayscale-to-RGB preprocessing, 448px input, ImageNet normalization, identity/horizontal/vertical flip probability averaging, and exported logit bias. Validation on 746 rows passed at mIoU 0.5926 (regolith 0.9469, rock 0.3148, shadow 0.5162). The single held-out pass on all 749 test rows also passed at mIoU 0.6057: regolith IoU 0.9181, rock 0.3691, shadow 0.5300. Each exceeds its frozen 0.25 class minimum; mIoU exceeds 0.50. Held-out class support was 9,262,739 regolith, 660,626 rock, and 615,255 shadow pixels. No threshold or split was changed.

Checkpoint SHA-256: `16276d1375f023d04d3f60557a3c6a84f3d5dbfc97c51d4aea3deeec8c5a8210`. Full confusion matrices, gate calculations, frozen input hashes, and reproducible evaluator are in `evidence/runs/20261112-p502-threeclass/P5-02/`. P5-02 is complete for the active three-class engineering scope and may feed P5-03. The historical five-class model remains failed and this analog-corpus result does not establish deployment performance.
