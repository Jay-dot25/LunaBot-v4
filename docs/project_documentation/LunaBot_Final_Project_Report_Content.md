# LunaBot V4 Final-Year Project Report Content

## Front matter (complete before submission)

**Title:** Terrain-Aware Autonomous Navigation for a Simulated Lunar Rover Using ROS 2 and Gazebo Sim  
**Student(s):** [insert] · **Supervisor:** [insert] · **Department/Institution:** [insert] · **Academic year:** [insert]  
**Declaration/approval/acknowledgment:** insert institution-approved wording; do not copy a generic declaration.

## Abstract

This project develops LunaBot V4, a ROS 2 and Gazebo Sim research implementation of an autonomous rover navigation stack for lunar-like terrain. The rover uses simulated RGB, depth, LiDAR, IMU and wheel odometry streams. LiDAR SLAM provides a map and global-to-local correction; RGB-D terrain classification and geometry mapping produce semantic, slope, roughness, crater-risk and uncertainty layers; LiDAR obstacle detection adds an obstacle overlay; weighted grid A* plans over terrain cost; and a freshness-gated path follower sends bounded commands through a controller and simulator watchdog. A lifecycle supervisor records mission events. Evidence includes a short-route localization error of 0.2299 m, held-out mIoU of 0.6057 on 749 analog records, four seeded geometry calibration captures, three simulated round trips, 15 deterministic path-planner fixture attempts and two 2026-11-19 manual GUI mission runs passing all 11 presentation-specific checks plus final map save. Those GUI trials recorded obstacle-to-changed-path intervals of 56.8 s and 75.9 s in a WSL presentation profile. The evidence has different scopes and does not establish flight readiness or a statistical mission success rate. Mission travel time, trajectory error, collision count, clearance and recovery remain unmeasured in the cross-trial report. The system is a student simulation, not a physical lunar rover.

## Chapter 1 — Introduction

### 1.1 Background

Planetary mobility faces uncertain terrain and limited communication compared with terrestrial robots. Local autonomous mapping, hazard perception and route following can support operations. ROS 2 and Gazebo enable an educational platform for studying the software pipeline, while not replacing operational or flight qualification.

### 1.2 Problem statement

Connect simulated sensors, localization, semantic/geometric terrain assessment, occupancy and terrain-aware planning, obstacle-triggered replanning, safe command flow and mission status into an inspectable navigation demonstration.

### 1.3 Objectives

Use the objectives/status definitions in Section 2 of the main documentation. Include only the implemented and validated phases supported by the evidence ledger.

### 1.4 Scope

Software and simulated rover in ROS 2 Humble/Gazebo Fortress, including bounded scenario and perception evaluations. Excludes flight hardware, calibrated lunar soil, real sample collection, certified safety and general lunar performance.

## Chapter 2 — Literature and context

Summarize planetary rover navigation from official NASA/JPL sources and peer-reviewed foundations listed in `references.md`. Compare at system level: Curiosity combines stereo terrain assessment with odometry/visual methods; Perseverance AutoNav constructs 3D terrain hazard representations and routes between operator-defined destinations; VIPER technical materials emphasize lunar polar lighting and stereo sensing. State that these are operational mission systems and unlike LunaBot in maturity, hardware, validation and constraints. Review A*, D* Lite, ROS 2, Gazebo and `slam_toolbox` as established technology.

## Chapter 3 — Requirements and system design

### 3.1 Functional requirements

Receive operator goal; acquire sensor data; maintain map/pose frames; classify/project terrain; mark obstacles; plan/replan; stop on unsafe/stale state; report mission state. Every requirement should be linked to a source file and verification record.

### 3.2 Assumptions and constraints

Ubuntu 22.04/ROS 2 Humble/Gazebo Fortress profile; simulation timing; known frame tree; map connected around requested goal; no actual lunar environmental dynamics or hardware.

### 3.3 Architecture

Insert the `LunaBot_Architecture.md` Mermaid pipeline and data-flow table. Describe standard ROS messages, topics and transform authority. Include no unimplemented action/service claims.

## Chapter 4 — Rover and simulation model

Describe `model.sdf`: 24 kg chassis link, 0.95×0.68×0.22 m chassis collision, articulated rocker/bogie-like links, six wheel collisions of radius 0.17 m and width 0.13 m, sensor mast and DiffDrive. Note chassis mass is not full model mass. Include `assets/report_rover_schematic.png` only as an illustrative SDF-informed schematic. Describe lunar world gravity and generated mesh/obstacles/habitat. State simulation fidelity limits. Insert an actual Gazebo screenshot when captured; current package lacks one.

## Chapter 5 — Sensors and ROS data

Use the sensor table in main documentation. Explain Image/CameraInfo, LaserScan, Imu, Odometry, occupancy/path/pose/twist messages and frames. Explicitly note the SDF baseline camera dimensions (160×120) differ from the Phase B measured runtime contract (640×480). Explain simulator intrinsics are not physical calibration and sensor rates are simulation-time rates.

## Chapter 6 — Localization, perception and mapping

Describe LiDAR `slam_toolbox`, `map→odom` ownership, wheel odometry limitations, semantic model classes, RGB-D pinhole projection, map layers and cost fusion. Include map artifact metadata and image links. Explain why unknown/shadow are caution rather than a semantic hard obstacle, and separate occupancy from terrain risk.

## Chapter 7 — Planning, control and mission lifecycle

Explain occupancy A*, terrain weighted A*, obstacle grid merge/inflation, path following and changed-route logic. Include kinematic equations from the main documentation. Explain command safety chain and mission lifecycle/SQLite events. State D* Lite was benchmarked only and not retained in live route path; Nav2 is absent from the current custom planner implementation.

## Chapter 8 — Implementation

| Function | Code location |
|---|---|
| ROS package launch | `src/lunabot_bringup/launch/`, `scripts/launch-*.sh` |
| World/model | `src/lunabot_gazebo/worlds/lunar_world.sdf`, `models/lunabot_v4/model.sdf` |
| Occupancy planner | `scripts/astar_navigation.py`, `scripts/terrain_plan.py` |
| Terrain planner/cost | `scripts/terrain_aware_planner.py`, `scripts/terrain_cost_mapper.py`, `scripts/terrain_costs.py` |
| Perception/mapping | `scripts/p502_threeclass_inference.py`, `scripts/semantic_projection.py`, `scripts/semantic_terrain_mapper.py` |
| LiDAR obstacle | `scripts/obstacle_detector.py` |
| Follower/control/safety | `scripts/terrain_path_follower.py`, `scripts/control_odometry.py`, `src/lunabot_gazebo/src/command_watchdog.cc` |
| Mission/evaluation | `scripts/mission_lifecycle.py`, `scripts/phase_l_mission.py`, `scripts/phase_k_evaluator.py` |

## Chapter 9 — Testing and results

Use the experiment table in `LunaBot_Experiments_and_Results.md`, cite exact evidence directories and include run ID/provenance. Separate unit tests, deterministic planner fixtures, held-out model evaluation, seeded Gazebo calibration and live simulated mission results. Include both successful manual GUI gate runs and failed/partial same-day attempts. Do not relabel a fixture as a field test or treat two passes as a reliability estimate. Include unavailable metrics explicitly. The documentation author did not rerun the full test suite while preparing this package.

## Chapter 10 — Discussion, limitations and future work

Discuss simulation-to-real issues, heuristic costs, narrow localization evidence, calibration limits, absent collision/clearance/travel-time metrics, target machine/restart/whole-stack resource qualification, and no hardware/sample payload. Prioritize repeated live mission qualification and measurement, then calibrated mobility and hardware validation. Consider learning control only if it is justified by measurable research value and safety evidence.

## Chapter 11 — Conclusion

LunaBot V4 provides an integrated, evidence-tracked simulated navigation pipeline and a useful basis for robotics evaluation. Its contribution is system integration and bounded engineering validation. Current evidence supports only the scenarios and datasets named in the results; it does not establish a real lunar rover capability.

## Appendices to prepare

A. Complete launch/reproduction commands and environment package versions.  
B. ROS topic/type/frame contracts and TF captures.  
C. Test output, run metadata and input hashes.  
D. Selected actual Gazebo/RViz screenshots with source run IDs (currently missing).  
E. Configuration listings and model/world SDF excerpts.  
F. Institutional formatting, signatures and plagiarism/citation checks.
