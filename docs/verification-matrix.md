# Verification matrix

Planning, documentation, implementation and runtime are distinct. Historical reports do not prove current runtime.

| Requirement | Tasks | Verification | State |
|---|---|---|---|
| Simulation/reset/headless | P1-01–03 | Build/dependencies and repeated lifecycle | Planned |
| Sensors/TF/calibration/safety | P2-01–03 | Timing/frame contracts and fault injection | Planned |
| Wheel odometry/GPS-free SLAM/persistence/truth | P3-01–03 | P3-01 truth/odometry gates; P3-02 valid scan, non-empty map, occupancy and pose-graph persistence, saved-map localization, bounded truth comparison | P3-01 passes three fresh frozen simulated trials after the measured 1.03 m effective-track calibration: straight errors 0.0019–0.0038 m and turn errors 0.0879–0.0965 rad. Physical hardware and low-friction behavior remain unqualified. P3-02 PASS: 403/720 valid returns, map reload preserves 38,860 known/29,468 unknown cells, pose graph reloads fresh, truth error 0.230 m / 0.000053 rad within 0.25 m / 0.15 rad; P3-03 registry contract PASS. P8-03 later supplies qualified map coverage and bound simulation staging poses. Evidence: evidence/runs/20261116-p301-retry/qualification-fixed/. |
| Occupancy A*/avoidance/recovery | P4-01-03 | P4-01 planner fixtures; P4-02 follower fixtures and syntax; P4-03 repeated occupancy route baselines | P4-01 implementation/unit PASS: 6/6; latest Phase D static validator PASS 152/152, superseding the historical 151/152 Phase C false positive. P4-02 implementation/unit PASS: 8/8 follower fixtures, 6/6 A* regression fixtures, Python compile, Phase L bash syntax; live Phase L WSL obstacle stop/replan/goal run PASS. P4-03 planner baseline PASS: 3 routes x 3 attempts (9/9); records include outcome, path length, path cells, clearance estimate and plan time. |
| Semantic ML | P5-01-03 | P5-01 source/split policy; P5-02 train/eval; P5-03 export/inference | Active three-class regolith/rock/shadow model PASSED frozen P5-02 gates: validation mIoU 0.5926, held-out mIoU 0.6057; class IoU 0.9181/0.3691/0.5300. TorchScript parity, stale-input behavior, confidence and CPU/CUDA provider checks PASS. Phase L ROS inference PASS: 365 CUDA frames fed live semantic mapping/cost/planning; obstacle replanning, integrated goal, evaluator, mission and clean shutdown PASS. Historical five-class run (mIoU 0.310) is archived and superseded for active output scope. Evidence: evidence/runs/20261112-p502-threeclass/P5-02/, evidence/runs/20261112-p503/P5-03/, evidence/runs/20261116-threeclass-phase-l-retry2/. |
| RGB-D semantic/geometric mapping | P6-01–03 | Projection, aging, coverage, aligned layers | Fixtures, Phase L integration, and seeded Gazebo gate v2 PASS. Three independent known-plane captures passed both depth bands with worst horizontal p95 0.0219 m near / 0.0576 m far and vertical p95 0.0096 m near / 0.0152 m far. Flat slope/crater-risk p95 was 2/0. The separate crater capture covered 157 cells within the 0.7 m center region; crater risk max was 100 and layer p95 errors were slope 9, roughness 14.2, crater 0. Evidence: evidence/runs/20261117-p6-final-v3/summary.json and docs/p6-calibration-plan.md. |
| Terrain planning/replanning | P7-01–03 | Route/obstacle tests and latency/comparison | P7-01 weighted A* fixtures PASS 8/8; occupancy endpoint/corner constraints, unknown policy, rotated origins, and deterministic map revision behavior verified. P7-02 deterministic obstacle-stop/replan latch and latency instrumentation PASS (9/9 follower tests plus monitor timing fixture); one live ROS/Gazebo WSL mission PASS captured 4 revisions and 19,965.5 ms obstacle-to-changed-plan latency. This is measured runtime evidence, not a target-profile performance qualification. P7-03 retry PASS: 3 routes x 3 pairs, all 18 attempts succeeded with equal path costs. Weighted A* median replan 3.105 ms vs D* Lite 20.659 ms; D* Lite fails the frozen adoption gate, so weighted A* remains selected. Evidence: evidence/runs/20261112-p701/P7-01/, evidence/runs/20261112-p702/P7-02/, evidence/runs/20261112-p703/P7-03/, evidence/runs/20261116-p703-retry/benchmark.json, evidence/runs/20261116-phase-l-acceptance/. |
| Mission lifecycle/delivery | P8-01–03 | Transitions/idempotency/roundtrips | P8-01 map/goal validation and lifecycle PASS. P8-02 SQLite delivery idempotency across reopen, pause/resume/cancel, blocked/failure recovery, e-stop/reset, and fresh ACTIVE-only follower gate fixtures PASS (7 focused cases); Phase L P8-specific static checks pass. P8-03 live acceptance PASS: three independent round trips, nine exact goal completions, three durable delivery proofs. Evidence: evidence/runs/20261112-p801/P8-01/, evidence/runs/20261112-p802/P8-02/, evidence/runs/p803-fixed-r64/phase-l/, and evidence/runs/p803-acceptance-final2-20261113/P8-03/. |
| Six dashboard views | P9-01–03 | Stack comparison, timestamped telemetry/control integration, loaded six-view benchmark | P9-01 comparison and safety boundary PASS. P9-02 adapter and P9-03 gateway built and launched against a live Phase L ROS graph. Telemetry snapshot and PAUSE/RESUME/ESTOP/RESET_ESTOP acknowledgments recorded; terrain-plan acceptance failed occupancy/synchronized-slope gates. A six-panel hosted Foxglove smoke layout displays live status and camera at 1536×864, but does not yet cover all six contract view groups/topics. Five-minute client diagnostics measured app-level main-thread FPS, data-pipeline rate, event-loop latency, and JS heap; four five-minute host/stack profiler trials are retained. P9-03 remains open for full contract layout coverage, camera-specific source-to-render age/FPS/drop, browser CPU/VRAM, reconnect timing, doubled-traffic workload, release target identification and frozen budgets. Evidence: evidence/runs/20261113-p901/P9-01/, evidence/runs/20261114-p902/P9-02/, and evidence/runs/20261114-p903/P9-03/. |
| Five scenarios and all metrics | P10-01–03 | Seeded trials, isolated truth and reports | P10-01 metric/truth contract PASS; P10-02 five planner-fixture scenarios x three seeds PASS; P10-03 all 14 metric definitions accounted with provenance, five unavailable reasons, and four plots. P8-03 is the separate live full round-trip reference; four scenarios remain planner-level, not live acceptance. Evidence: evidence/runs/20261113-p1001/P10-01/, evidence/runs/20261113-p1002/P10-02/, and evidence/runs/20261113-p1003/P10-03/. |
| Resources/restart/final acceptance | P11-01–03 | Soak, compatibility and mission | Latest local verification: full pytest 202 passed/2 environment skips, Phase K 105/105, Phase L 126/126, and two-package colcon test passed 1/1. Phase L headless WSL mission acceptance and clean shutdown PASS. P11-01 target limits remain unset pending P9-03's target-machine dashboard benchmark; target resource/overload and formal restart acceptance remain open. Evidence: evidence/runs/20261116-phase-l-acceptance/ and prior P11 evidence at evidence/runs/20261114-p1102/P11-02/ and evidence/runs/20261114-p1101/P11-01/. |
| Separate Goal Master Plan | P0-01 | Locate/reconcile | Not found; unverified |

P1-03 PASS: static physics/wheel/suspension/collision/landmark contracts; headless wheel motion; forward LiDAR return; low-speed obstacle boundary stop; refined passive suspension response reproduced in the actual source model over a roughly 0.28 m collision-terrain elevation change with rocker span 0.27-0.28 rad and bogie span 0.15-0.17 rad, inside limits. Evidence: evidence/runs/20261104-p103/report.md. Record commands, exit codes, commit/config/model hashes, seed, timestamps and evidence. P0-03 creates runner; runtime requires actual ROS/Gazebo evidence.
P1-02 runtime evidence: three corrected Phase B headless cycles PASS; startup, drive/turn, settled stop, restart initial pose and final process cleanup verified. See evidence/runs/20261103-p102/P1-02/summary.json. This does not complete P1-03 physics/landmarks or P2 sensor calibration gates.

P2-01 update: corrected camera physical pitch, scoped optical/IMU/LiDAR frames, CameraInfo bridges and JointState type. Sensor-head TF follows measured mast joints. Phase B live acceptance PASS; A/B/C static gates PASS in isolated copy; full A-L runtime remains unverified. Measured contracts, QoS evidence, failure behavior and registration limits: docs/sensor-contracts.md; evidence/runs/20261103-p201/P2-01/summary.json.

P2-02 update: sole configured controller routing for A-L; latched emergency stop/reset services, finite-command validation, steady-clock watchdog. DDS and Phase B live acceptance PASS; Phase A smoke PASS. Interfaces, evidence and remaining process/restart/freshness limits: docs/safety-control.md and evidence/runs/20261103-p202/P2-02/summary.json.

P2-03 evidence: real monitor/bridge/controller fault injection, restart inhibition,
clock pause/resume and missing-map gating; explicit fixtures cover invalid/stale/
replayed data and clock discontinuities. Native package build, all A-L static
gates, bounded cleanup and separated-process SLAM drive regression are recorded.
See evidence/runs/20261103-p203/P2-03/summary.json and safety-faults.md. This does
not establish localization accuracy, every A-L runtime combination or a full
mission. No synthetic sensor/localization fixtures were used as production data.

Historical P3-01 runs with the original 0.80 m effective track failed turn
accuracy (3/3) and straight accuracy (2/3); controlled low-friction testing
measured 0.947 m error. These runs are retained as provenance and superseded
for the frozen simulated accuracy gate by the three calibrated 1.03 m trials
documented in docs/odometry-qualification.md. Physical hardware and
low-friction accuracy remain unqualified.


P3-02 runtime verification: bash -n scripts/launch-c.sh; ign sdf -k on world and rover model; headless mapping and fresh localization launches without RViz; direct rclpy scan capture; independent transient-local nav2_map_server reload; evaluator-only Gazebo truth comparison. Evidence and limits: docs/slam-qualification.md and evidence/runs/20261103-p302/.

P4-01 behavioral verification: python3 -m unittest discover -s tests -p
test_astar_qualification.py (with ROS 2 Humble sourced) passes 6/6. Python
compile passes. The original Phase D static report was 151/152 because of an
inherited Phase C diagnostic substring check. The latest Phase D report passes
152/152 and supersedes that historical failure. Evidence:
evidence/phase-d-launch-d/static_validation.txt and evidence/runs/20261104-p401/.


P4-02 implementation verification: with ROS 2 Humble sourced, python3 -m
unittest tests.test_terrain_follower_safety -v passes 8/8, and
python3 -m unittest tests.test_astar_qualification -v passes 6/6.
python3 -m py_compile scripts/terrain_path_follower.py
tests/test_terrain_follower_safety.py and bash -n scripts/launch-l.sh pass.
Evidence: evidence/runs/20261104-p402/P4-02/summary.json. This is unit/static
verification; live Phase L collision-stop timing and mission behavior remain
runtime gates for P7-02. RViz was not used or required.


P4-03 planner baseline verification: source ROS 2 Humble, then run
python3 -m unittest tests.test_navigation_baselines -v and
python3 tools/run_navigation_baselines.py --trials-per-route 3 --output
evidence/runs/20261104-p403/P4-03/navigation-baselines.json.
Three route scenarios produce nine retained attempts. The corrected wall-gap
geometry passes after an initial failed design iteration was preserved separately.
Metrics are planner path length, path-cell count, grid clearance estimate and
planning time. These are deterministic occupancy-grid planner measurements,
not physical travel-time or rover-motion measurements. See
evidence/runs/20261104-p403/P4-03/.


P5-01 verification: python3 tools/validate_perception_manifest.py
--classes config/perception/classes.json --sources config/perception/sources.json
--splits config/perception/splits.json passes; python3 -m unittest
tests.test_perception_manifest -v passes 5/5; all three JSON manifests parse.
This validates the protocol and planned group split, not acquired-file inventories
or trained-model metrics. External data remain outside the repository.


P5-02 implementation verification (2026-11-04): `python3 -m unittest tests.test_perception_training tests.test_perception_manifest -v` passes 10/10; `python3 -m py_compile tools/perception_metrics.py tools/train_perception.py tests/test_perception_training.py`, P5-01 manifest validation and `git diff --check` pass. This verifies metric semantics, ignore-label handling, manifest split enforcement, frozen gate schema and regression compatibility; the isolated CPU smoke exercises training/checkpoint/metrics/gate wiring only. No real dataset acquisition or held-out evaluation occurred; see docs/perception-dataset.md and evidence/runs/20261104-p502/P5-02/summary.json.

P5-02 full-source verification (2026-11-05): POLAR-Sim/NASA materialization has 3,879/3,879 paired images/masks; zero missing pairs or dimension mismatches. Split counts: train 2,319, validation 803, test 757. Pixel IDs 0=3,808,319,983; 2=200,897,482; 4=224,681,292; 255=4,897,949,947; bedrock 1 and crater 3 absent. NASA Terrain05/11/12 contain XYZ point clouds and stereo calibration; Terrain11 PosA inverse transform puts 349,998/411,672 sampled points in frame. Crater projector fixtures PASS 6/6; its archive member discovery now supports real dataset root names. An unreviewed PosA XY proposal for Terrain11 (validation) and Terrain12 (test) generated 131 separate candidate masks; outputs are review-only. Terrain05 was omitted because its dark gaps are missing point returns, not supported craters. Source JSON: /home/jay/lunabot-p502-persistent/annotations/crater-rois-proposed-20261105.json; candidate report: /home/jay/lunabot-p502-persistent/crater-candidates-proposed-20261105-v2/candidate-report.json. The Terrain11 and Terrain12 overlays were visually inspected; both contours cover the main bowl, but remain unreviewed and may include occluded/rock pixels.

P5-02 bedrock training-source verification (2026-11-05): pinned repository gremlin97/mars-seg_msl revision 35b03e48bcf9ab2e26980ccb58e4419d290e06c5, CC-BY-4.0 card, train shard SHA-256 041d3c77a2bfbdfecc74e8fd370b945521e5e5e204362530a06827162038822d. The external converter produced 2,893/2,893 paired PNGs, matching pixel histogram, canonical IDs only, bedrock 266,581,689 pixels and zero crater pixels. Background, sky/mountains and track map to ignore. All rows are split=train. Combined trainer loader PASS: 5,212 train / 803 validation / 757 test; Mars validation/test count zero. Converter tests PASS 3/3 and existing P5-02 suites PASS 21/21. External output/report: /home/jay/lunabot-p502-persistent/materialized-marsseg-train-20261105/. This supplies bedrock pretraining only; it is not lunar/analog evaluation and does not close held-out class-support requirements. No crater ROI review, frozen gates, trained qualification checkpoint, or real held-out metrics exist; P5-02 remains open.

P5-02 continuation verification (2026-11-06):  passes; focused trainer and crater-projector suites pass 15/15; projector report checks pass for all 131 review-only masks and both Terrain11/12 overlays; targeted  passes. The projector now preserves known non-regolith and unknown pixels, and the trainer preflights class pixel support before model creation. Real qualification remains blocked because accepted train crater labels and held-out analogue bedrock/crater support are absent; no training or IoU gates were run. Evidence: evidence/runs/20261106-p502/P5-02/summary.json.

### P5-02 resumed audit (2026-11-06)

This update supersedes the earlier statement that no train crater labels exist. The delegated Terrain05 PosA annotations were merged into 68 separate training masks, adding 850,772 crater pixels while retaining known rock/shadow/unknown pixels. All validation/test masks remain unchanged; the 131 held-out crater candidates remain proposals. The merge tool now rebases image and unchanged-mask paths to absolute paths so the relocated output manifest resolves correctly. The earlier external merged CSV had broken relative references and should not be used. The regenerated manifest is at evidence/runs/20261106-p502/P5-02/rebased-merge/manifest.csv (local derived data, ignored by Git).

All 29 perception materializer/projector/merger/trainer/manifest fixtures pass, including the relative-path regression. Compilation and git diff --check pass. The reproducible audit and measured counts are in evidence/runs/20261106-p502/P5-02/audit_continuation.py and continuation-audit.json. Five-class qualification remains open: validation/test have no accepted bedrock or crater support. Do not invent bedrock from discrete rocks or move Mars training frames into held-out splits. No qualification training, frozen gates, or held-out score is claimed.

Full resumed audit PASS for dimensions, canonical IDs and split policy on 6,772 records (train 5,212 / validation 803 / test 757). Training now supports all five classes, including 850,772 crater pixels and 266,581,689 bedrock pixels. Validation/test each still have zero pixels for IDs 1 and 3. Exact counts and input-manifest hashes: evidence/runs/20261106-p502/P5-02/continuation-audit.json.

### P5-02 annotation repair retry (2026-11-06)

Full-resolution review produced two conservative partial crater annotations: Terrain11 PosA `11_A_off_180_L_0032.png` has 551 geometry-supported reviewed crater pixels; Terrain12 PosA `12_A_off_180_L_0032.png` has 2,201. The corresponding unreviewed crater rectangles are ignore, preventing known ambiguous rock/shadow regions from being promoted to crater. The review applies only to those exact frames and hashes; other illumination/exposure variants remain unaccepted. These small patches are partial annotations and do not establish representative crater accuracy. The new manifest is a diagnostic retry corpus, not a completed qualification corpus. Review definitions, hashes and limitations are in `evidence/runs/20261106-p502/P5-02/heldout-patch-review.json`; reproducible materialization and retry scripts accompany it.

Bedrock remains a data-acquisition dependency. Reinspection of the staged held-out scenes did not establish continuous exposed consolidated substrate. An external-source search found the Concordia/Mission Control Lunar Analogue publication with exposed-bedrock examples, but no accessible dataset download in its linked public code repository; the user has no copy. MADMAX requires registration and describes academic use; Devon Island did not establish a compatible licensed bedrock mask corpus. No alternative source was silently approved. See `bedrock-source-search.json` for exact references and findings. No bedrock labels were fabricated and no qualification training was run. The retried preflight result is retained separately in `retry-preflight.json`.

Retry measured result: PREFLIGHT_REJECTED, missing class IDs validation=[1], test=[1]. Crater support is 551 / 2,201 pixels in the two partial annotations. Model creation and training did not occur. Focused regression suites pass 22/22; Python compilation and git diff --check pass.


### P5-02 independent corpus review (2026-11-07; supersedes readiness claims)

The inherited Terrain05 crater merge is quarantined pending correction. Inspection of full-resolution 05_A_off_180_L_0032.png and the 12-frame training montage shows accepted regions overlapping prominent positive rocks, with masks propagated through saturated and dark exposures. Successful geometric projection and unit tests do not establish semantic correctness. The restarted CPU run was stopped before held-out testing; its outputs must not be used as qualification evidence. NASA documents small negative features in Terrain05, but that does not validate these particular regions.

The frozen 256px cache also contains 276 identical completely white RGB frames across train (161), validation (59), and test (56). Remove these uninformative frames from all splits rather than relocating them. A first audit additionally keyed lineage by image content and therefore conflated same-image/different-mask entries; that auditor bug is corrected to use the cache key derived from the original path. The original audit report is retained for traceability and will be superseded by the corrected report.

The previously acquired Apollo and BASEPROD additions provide sparse bedrock evaluation support; claims that bedrock is wholly absent are stale for the combined corpus. They remain limited in domain, scale and scene count. A separate NASA POLAR Traverse training-only crater acquisition is being prepared; existing held-out Stereo terrain assignments are preserved.
