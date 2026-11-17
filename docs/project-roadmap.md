# LunaBot V4 roadmap

## Completion states
Planning completion maps requirements and gates. Documentation completion saves and cross-checks seven documents. Implementation completion passes applicable build/static/unit checks. Runtime verification requires real ROS/Gazebo integration and mission evidence. These states are distinct.

## Environment
Ubuntu 22.04.5 WSL, ROS Humble, Python 3.10.12, Gazebo Sim 6.18.0, bridge/slam_toolbox installed. RViz2 unavailable but optional for headless work. RTX 3050 4GB visible; WSL 5GB RAM, 8 processors, 8GB swap. Phase L model runtime was verified with the project training venv (Torch 2.11.0+cu128, NumPy 1.26.4, CUDA and ROS bindings); target-profile resource limits remain unqualified.

## Phases
0 audit/requirements/evidence; 1 simulation and incremental packaging/dependencies/reusable bringup; 2 sensor contracts/safety; 3 qualify wheel odometry and GPS-free SLAM; 4 occupancy-only baseline; 5 active three-class regolith/rock/shadow perception (five-class source masks retained); 6 synchronized RGB-D, semantic mapping and geometry; 7 terrain planning/replanning; 8 mission lifecycle; 9 dashboard selection/six views; 10 five-scenario evaluation; 11 resource/reliability/final acceptance. Preserve existing A*, mapping, assets, A-L launch entry points and working implementations. Replace only after measured failure and verified replacement.

## Requirements coverage
Covers supplied prompt: six-wheel rover/suspension, physics, Habitat/Science landmarks, reset/headless; RGB/depth/LiDAR/IMU/wheel odometry, calibration/timing/TF; GPS-free SLAM/maps/persistence/relocalization/waypoints; occupancy A* baseline, local avoidance, footprint/recovery; licensed terrain imagery and scene-separated splits; active P5-02 inference classes are regolith, rock (including merged bedrock labels), and shadow, with craters handled separately by geometry; annotation, training, IoU/mIoU/confusion, export parity and inference health; synchronized calibrated RGB-D/pose, invalid depth, occlusion, age/confidence, geometry/slope/crater risks; weighted A*, uncertainty, route validation, obstacle replanning/latency, conditional D* Lite; mission schema/map identity/collection/delivery/return/pause/cancel/blocked/recovery/failure/e-stop/restart/idempotency; six dashboard views; easy delivery, crater, unexpected obstacle, uncertain terrain, full round-trip scenarios; success/path/time/collision/clearance/terrain/localization/perception/replanning/recovery/resource metrics; matched occupancy-vs-terrain trials; truth isolation; laptop budgets, degradation, restart, demo. Separate Goal Master Plan referenced by prompt was not found; its coverage is unverified.

Headless acceptance has no RViz/dashboard dependency. Qualify DiffDrive before another estimator. Add packaging in Phase 1. Benchmark three dashboard approaches at P9-01. Isolate truth to evaluation. Critical path P0-01→02→03→P1→P2→P3→P4→P6/P7→P8→P10→P11; P5/P9 research can proceed when dependencies allow. P0-02 first; P0-03 runner next; P1-01 first engineering task.

Evaluation scenarios: easy sample delivery, crater-aware navigation, unexpected obstacle, uncertain terrain, and full outbound delivery plus return. Metrics include success rate, path length, travel time, collisions, clearance, terrain exposure, unsafe terrain entry, localization error, segmentation scores, replan count/latency, recovery, CPU, RAM, GPU memory and real-time factor. Compare occupancy-only and terrain-aware methods under matched repeated trials; reports include run/config/model versions, plots, timestamps, evidence and limitations.

## P3-01 qualification result

DiffDrive source is wheel-position derived. Three synchronized headless trials
confirm poor straight/turn accuracy (accuracy gate failed); fresh origin,
message monotonicity, TF lookup, and controlled-slip truth capture were verified.
Retain DiffDrive as a local motion prior only. P3-02 now tests GPS-independent
LiDAR SLAM correction; no estimator replacement is selected yet. See
docs/odometry-qualification.md and evidence/runs/20261103-p301/summary.json.
