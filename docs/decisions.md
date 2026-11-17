# Architecture decisions

| ID | Decision | Rationale/revisit |
|---|---|---|
| ADR-001 | Preserve both A*, mapping, assets and A-L launchers | Change only after measured failure and verified replacement |
| ADR-002 | Headless autonomy independent of RViz/dashboard | GUI optional; RViz unavailable |
| ADR-003 | Qualify DiffDrive before another estimator | Wheel-position source; estimator only if accuracy gate fails |
| ADR-004 | Incremental packaging/dependencies/bringup from Phase 1 | Early discipline; avoid broad rewrite |
| ADR-005 | One final /cmd_vel authority | Avoid conflicting publishers |
| ADR-006 | Prototype Foxglove Bridge/client first; keep browser+rosbridge and native Qt/RViz as measured fallbacks | P9-01 comparison and six-view contract are in docs/dashboard-adr.md and config/dashboard/. This is not production approval; P9-03 must measure the six-view workload and confirm account/seat terms. |
| ADR-007 | Isolate truth to evaluation | Operational decisions remain sensor-derived. P10-01 contract forbids ROS bridging and checks operational source paths; Gazebo pose capture/comparison stays in standalone evaluation tools. See docs/evaluation-contract.md. |
| ADR-008 | Preserve weighted A*; assess D* Lite on measured benefit | Avoid unjustified planner duplication |
| ADR-013 | Keep DiffDrive as a local prior; use qualified slam_toolbox for global map correction | P3-01 passes the frozen simulated accuracy gate after effective-track calibration, while low-friction slip remains measurable and physical hardware is unqualified. P3-02 proves saved-map scan localization within 0.25 m / 0.15 rad on the short route. Do not add another estimator without evidence that SLAM plus wheel odometry is insufficient. See docs/odometry-qualification.md and docs/slam-qualification.md. |
| ADR-014 | Use OGRE2 plus software OpenGL for the qualified WSL headless LiDAR profile | OGRE gives all-minimum LiDAR returns; OGRE2/D3D12 aborts in GL3PlusTextureGpu::copyTo; OGRE2 with LIBGL_ALWAYS_SOFTWARE=1 produces scene returns and a non-empty map. GUI/RViz remains separate. |
| ADR-015 | Preserve PGM unknown cells by saving map YAML with free_thresh 0.19 | Default 0.25 reloads PGM value 205 as free; independent map_server reload with 0.19 retains 29,468 unknown cells. launch-c applies this correction. |

Open: dashboard stack, dataset/license/model, broader-route localization/resource thresholds, mission persistence and package split. P3-02 truth boundary is verified for the recorded path.
## P2-03 decisions
| ID | Decision | Evidence and trade-off |
|---|---|---|
| ADR-009 | Separate raw health validation from motion heartbeat | SLAM-load regressions exposed heartbeat gaps when large callbacks shared the controller executor; separated SLAM regression passes without relaxed stop deadlines |
| ADR-010 | Native 0.25s actuator watchdog behind remapped ROS requests | A dead ROS controller cannot publish zero; native loss test and wheel stop verified |
| ADR-011 | Explicit startup arm, latched runtime recovery, idempotent acknowledgment | Controller restart does not resume held input; delayed RPC success is safely acknowledged without clearing faults |
| ADR-012 | Shared bounded process-group teardown and flag-only signal handlers | Fault suspension exposed odometry shutdown deadlock; bounded cleanup and clean teardown now tested |

ADR-005 applies to ROS request ownership. ADR-010 adds a separate native final
actuator authority; no competing publishers are introduced within either transport.

| ADR-016 | Keep existing rocker-bogie geometry and use qualified passive hinge dynamics (rocker damping/friction 10/1; bogie 5/0.5) | Baseline measured less than 0.0007 rad articulation over changing terrain; candidate and source-model headless trials measured 0.27-0.28 rad rocker and 0.15-0.17 rad bogie span over roughly 0.28 m terrain elevation change without reaching joint limits. Revisit for payload or higher fidelity. Evidence: evidence/runs/20261104-p103/. |

| ADR-017 | Pin semantic waypoints to map ID, frame and map artifact hashes; fail closed for unbound poses | P3-02 map does not cover the Habitat world landmark, and no Science coordinate is present. Keep IDs registered, reject invalid identity/frame, and block dispatch until authoritative map-aligned poses exist. Evidence: docs/waypoints.md and evidence/runs/20261104-p303/. |

| ADR-018 | Preserve occupancy A* and disallow diagonal corner cutting or out-of-map snapping | P4-01 fixtures exposed a path through two blocked cardinal cells and silent snapping of an out-of-bounds goal. The existing planner now checks both cardinal neighbors for diagonal motion and rejects out-of-bounds cells. Six fixtures pass. Evidence: evidence/runs/20261104-p401/. |


| ADR-019 | Keep follower recovery stop-only; gate Phase L motion on a fresh obstacle map and fresh terrain path | The follower checks the remaining terrain path against occupied obstacle-grid cells with an inflated clearance, rejects empty/stale plans, and commands zero on obstacle or map staleness. It does not autonomously reverse or spin; this bounds recovery without introducing unverified maneuvers. Phase L requires map receipt; older detector-free launches keep the map optional. Unit acceptance is recorded at evidence/runs/20261104-p402/P4-02/summary.json; live latency remains P7-02. |


| ADR-020 | Establish occupancy baselines with deterministic grid routes before matched rover comparisons | Use three named routes and three repeated A* attempts per route to retain repeatable planner metrics now. Keep the output explicitly separate from physical motion, terrain-aware, and matched-method comparisons, which require later runtime gates. Evidence: evidence/runs/20261104-p403/P4-03/. |


| ADR-021 | Train the five-class ontology with source-scoped rights and scene-grouped evaluation | Use POLAR/POLAR-Sim lunar-analogue scenes for lunar-relevant evaluation, and permit CC-BY Mars bedrock imagery only as training pretraining. All related images, variants, masks, and crops inherit one terrain-scene split. Verify acquisition notices and attribution before importing files. Evidence: docs/perception-dataset.md and evidence/runs/20261104-p501/P5-01/. |

| ADR-022 | Use three semantic output classes for the active P5-02 model | Train regolith, rock, and shadow outputs on a separately derived full-corpus mask set. Merge source bedrock and discrete-rock labels into rock; map crater and existing ignore pixels to ignore, leaving crater risk to geometry. Preserve the five-class source ontology, original masks, and historical five-class evidence; keep the same scene-isolated validation/test membership and freeze separate three-class gates before the held-out run. Evidence: docs/perception-three-class-plan.md and evidence/runs/20261112-p502-three-class/P5-02/. |
