# Known issues and verification gaps

| Issue | Evidence | Resolution |
|---|---|---|
| Separate Goal Master Plan not found | Prompt reference; absent from inspected materials | P0-01; disclose limitation |
| RViz2 unavailable | Humble package lookup failed | Optional GUI; headless unblocked |
| Historical A/B logs say ROS missing | Old logs | Current WSL has Humble; collect new run evidence |
| Phase L headless acceptance | The isolated 2026-11-16 ROS 2 Humble/Gazebo WSL run passed all 23-stage runtime gates with the active three-class CUDA model, dynamic replanning, integrated goal, evaluation, mission demo, map save, and clean shutdown | Resolved for the recorded WSL software-rendered profile and 1.5 m bounded demo goal; evidence: evidence/runs/20261116-threeclass-phase-l-retry2/. GUI and target-machine resource/restart acceptance remain under P9/P11. |
| WSL headless Gazebo renderer and timing | Earlier attempts crashed in OGRE2 or stopped during obstacle replanning and mission completion | Resolved for the recorded headless WSL run by Mesa software rendering, planar navigation LiDAR, and a derived monitor-report timeout; the actuator command watchdog remains 0.5 s. All runtime gates pass with clean shutdown; evidence above. |
| P3-01 wheel odometry accuracy | Three fresh frozen trials with 1.03 m effective wheel separation: straight errors 0.0019–0.0038 m and turn errors 0.0879–0.0965 rad; historical 0.80 m runs and low-friction run retained | Accuracy gate PASSES for this WSL Gazebo profile. Wheel odometry remains slip-sensitive and physical hardware is not qualified. Evidence: `evidence/runs/20261116-p301-retry/qualification-fixed/`. |
| Live projection/risk calibration | P6 fixtures pass and Phase L exercised RGB-D projection, map->camera TF lookup, occlusion handling, map recentering, and geometry/cost publication | Extract truth from the seeded Gazebo terrain and SDF, freeze numeric gates before capture, then follow docs/p6-calibration-plan.md; use physical surveys for hardware qualification |
| Semantic map retains only rover-local 16m coverage | P6-02 rolling 160x160 grid follows the rover; cells confidence-decay and expire | Mission-wide archival coverage requires a persistent map layer; the bounded live layer deliberately discards cells outside its window |
| Semantic confidence calibration | Active Phase L segmentation exports P5-02 three-class calibrated class confidence; mapper confidence aging/thresholding is fixture-tested | Confidence is the model's maximum class probability, not a separately calibrated reject probability; shadow is preserved as unknown in the traversability map |
| Elevation-derived risk layers lack live ROS/Gazebo calibration | P6-03 implements aligned slope, roughness, bowl-depression, and uncertainty risk grids with deterministic fixtures; Phase L exercises the layers; independent scene truth and numeric accuracy limits are absent | Acquire a surveyed slope/bowl reference, freeze error/recall/false-positive limits before qualification, then run docs/p6-calibration-plan.md |
| Historical five-class model archived | Five-class held-out mIoU 0.310; the project scope is now three-class regolith/rock/shadow and its frozen gates passed | Keep the five-class run as provenance only; do not use it in active inference |
| Broader mission acceptance beyond staging | P8-03 passed three independent live Habitat-Science-Habitat staging trials with durable delivery proofs | Keep P8-03 as the qualified WSL staging scope; target hardware and alternate site-map acceptance remain separate. See docs/waypoints.md. |
| Six-view dashboard release acceptance incomplete | An exploratory Foxglove layout exists, but it lacks contract-wide topic coverage and target-profile performance evidence | Complete P9-03 against a frozen target profile; see docs/p9-03-completion-plan.md |
| Research evaluation scope is limited | P10-01–03 produce matched planner trials and an evidence report; four scenarios remain planner-only | Preserve the report's explicit limitations; live sensor/motion comparisons for the remaining scenarios are future acceptance work. |
| WSL 5GB RAM/GPU 4GB | Environment inspection | P11-01 |
| P5-03 ROS Python/CUDA compatibility | Phase L segmentation ran from `.build/p502-training-venv/bin/python` with Torch 2.11.0+cu128, CUDA, ROS 2 Humble, NumPy and Pillow; 365 live frames passed | Resolved in the current WSL environment. A clean machine must provide the P5 runtime venv and model archive before launch. See `config/perception/active-model.json` and `evidence/runs/20261116-threeclass-phase-l-retry2/`. |
| Phase L Bash syntax | `bash -n scripts/launch-l.sh` passes after active model and WSL profile changes | Resolved; rerun after launch edits |

| Habitat/Science waypoint poses unbound | P3-02 map covers x=-2.1..3.75 m; Habitat world landmark at x=-3.5 m lies outside it; current world/map has no Science site or authoritative coordinate | P3-03 registry rejects dispatch of unbound goals. Bind only after an aligned map covers Habitat and a Science site pose is supplied. See docs/waypoints.md. |

| Historical Phase C static-validator false positive | The original Phase D regression report was 151/152 because the inherited Phase C check matched a bounded diagnostic pipe/head command | Superseded by the latest Phase D static report at 152/152. Keep the earlier report for history; current static bundle is green. |

Do not close issues from source presence or historical static reports alone.
P1-02 resolved Phase B child-process cleanup: owned ROS wrapper process groups now stop together. Three corrected cycles passed with zero remaining simulation/bridge/TF/control/monitor processes. Evidence: evidence/runs/20261103-p102/P1-02/summary.json. Scope is Phase B; other launchers require separate migration. In-process world reset not tested.

P2-01 update: corrected camera physical pitch, scoped optical/IMU/LiDAR frames, CameraInfo bridges and JointState type. Sensor-head TF follows measured mast joints. Phase B live acceptance PASS; A/B/C static gates PASS in isolated copy; full A-L runtime remains unverified. Measured contracts, QoS evidence, failure behavior and registration limits: docs/sensor-contracts.md; evidence/runs/20261103-p201/P2-01/summary.json.

P2-02 update: sole configured controller routing for A-L; latched emergency stop/reset services, finite-command validation, steady-clock watchdog. DDS and Phase B live acceptance PASS; Phase A smoke PASS. Interfaces, evidence and remaining process/restart/freshness limits: docs/safety-control.md and evidence/runs/20261103-p202/P2-02/summary.json.

P2-03 revision: controller/bridge loss is now covered by the native actuator
watchdog, and controller restart is inhibited until explicitly armed. Raw health
validation moved out of the motion executor after an observed SLAM-load heartbeat
failure. Process cleanup now uses the shared bounded helper; the odometry monitor
does not call ROS shutdown from its signal handler. Earlier failed attempts and
their resolutions are preserved in the P2-03 evidence tree. Backward/forward time
jumps are fixture-tested; pause/resume and process loss are live-tested. Broad
A-L runtime/security/accuracy acceptance remains open. See safety-faults.md.

Historical P3-01 runs with 0.80 m effective track show turn error
1.298–1.563 rad and straight error above 0.10 m in two of three runs; controlled
low friction exposes 0.947 m wheel-ground error. The calibrated 1.03 m model
passes three fresh simulated trials against the same frozen gate. The earlier
failures remain evidence of slip sensitivity; hardware remains unqualified.
See docs/odometry-qualification.md and
evidence/runs/20261116-p301-retry/qualification-fixed/.

P3-02 completed in the qualified WSL software-rendered headless profile. Localization truth error is 0.230 m against the 0.25 m qualification limit, leaving limited margin; wider routes and higher-resolution camera/depth performance remain unverified. See docs/slam-qualification.md and evidence/runs/20261103-p302/.

P3-02 profile note: 160x120 camera/depth is a resource-bounded qualification setting, not a final dashboard/perception resolution. OGRE2 with hardware-selected D3D12 aborts in this environment; use the recorded software-rendered headless profile until another renderer path is tested.
| P1-03 suspension/collision fidelity | Initial model hinge resistance suppressed articulation (<0.0007 rad on the recorded route); the first route did not reach uneven terrain or obstacle contact | Resolved for this model profile: refined passive hinge dynamics are qualified in headless terrain and obstacle trials. Wider terrain, payload and alternative renderer profiles still need qualification. Evidence: evidence/runs/20261104-p103/. |


| P4-02 live avoidance timing | Headless Phase L live obstacle detection, stop, changed-plan, and goal completion passed on 2026-11-16; this is one software-rendered WSL trial | Evidence is recorded in evidence/runs/20261116-phase-l-acceptance/. Treat its 19,965.5 ms obstacle-to-changed-plan timing as measured, not a frozen cross-platform performance limit; repeat on the target profile before setting a limit. |


| P4-03 baseline is planner-only | Nine deterministic occupancy-grid A* attempts; no Gazebo motion, sensor variation, or travel-time measurement | Preserve as occupancy planner baseline. Add matched headless rover trials under later route/evaluation tasks; do not compare these planner times to physical navigation duration. |

| Archived five-class source support is incomplete | POLAR-Sim/NASA masks contain IDs 0,2,4,255 only; the separate MarsSeg train-only materialization supplies bedrock but no crater labels. The active project output is three-class and passed its frozen gates. | Keep this as a limitation of the archived five-class source/training path. Reactivate it only after reviewed held-out bedrock/crater support and qualification; do not block the active three-class model on it. See docs/perception-dataset.md. |


| Historical five-class P5-02 qualification blocker (superseded for active three-class scope) | The five-class ontology still lacks adequate crater and bedrock support for qualification; the historical five-class model failed. | Active three-class regolith/rock/shadow model passed its frozen validation and held-out gates on 2026-11-12. Retain the five-class limitation only if that output ontology is required. See `evidence/runs/20261112-p502-threeclass/P5-02/` and `docs/perception-three-class-plan.md`. The three-class result is specific to the existing analog corpus and is not deployment evidence. |

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

### P5-02 qualification failure (2026-11-08)

Training and evaluation evidence is complete, but the five-class model did not pass frozen gates. Best validation mIoU is 0.302 at epoch 23; held-out test mIoU is 0.310. Bedrock and crater test IoU are 0.001 and 0.003; rock is 0.228. Do not integrate this checkpoint. The low scores and sparse held-out support show that current training data do not establish bedrock/crater transfer. Add a provenance-reviewed, scene-independent representative training source, keep the existing gates frozen for a new run, and rerun before claiming qualification. See `evidence/runs/20261107-p502/P5-02/summary.json`.
