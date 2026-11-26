# LunaBot V4 Technical Evaluation Presentation

18-slide outline. Use only project assets listed in `LunaBot_Image_Catalog.md`; GUI screenshots are still needed. Keep result scope labels visible on charts.

## Slide 1 — LunaBot V4: Terrain-Aware Rover Navigation

**Key points:** ROS 2 + Gazebo research prototype; autonomous navigation pipeline; simulation-only. **Visual:** actual rover/environment screenshot (capture required). **Speaker:** “This is a student simulation and integration study, not flight hardware.”

## Slide 2 — Why planetary rover autonomy?

**Key points:** no GPS; uncertain terrain; communication/operations constraints; autonomy supports science traverse. **Visual:** simple sourced concept diagram, with NASA reference in notes. **Speaker:** connect remote operations challenge to safe local navigation without implying LunaBot reproduces flight operations.

## Slide 3 — Problem and objectives

**Key points:** goal-to-route-to-motion with semantic terrain risk and obstacle replan; single command safety boundary; measurable evidence. **Visual:** one statement and scope labels. **Speaker:** define scope and exclusion of physical sample handling.

## Slide 4 — What is implemented?

**Key points:** simulated six-wheel rover; sensor bridge; SLAM; RGB-D model; layered terrain risks; weighted A*; follower/watchdog; mission journal. **Visual:** architecture overview. **Speaker:** separate actual components from future work.

## Slide 5 — Rover and mechanical model

**Key points:** six cylindrical wheels; rocker/bogie-like articulation; 0.17 m radius, 0.13 m width; 1.03 m effective track; chassis 0.95×0.68×0.22 m. **Visual:** `assets/report_rover_schematic.png` (illustrative) plus actual Gazebo screenshot still requested; SDF dimensions footnote. **Speaker:** all values are SDF simulation parameters, not built hardware specs.

## Slide 6 — Environment and simulation fidelity

**Key points:** lunar gravity configured 1.62 m/s²; generated mesh and obstacle/habitat geometry; synthetic sensors. **Visual:** full Gazebo scene screenshot required. **Speaker:** vacuum, dust, radiation, thermal cycles and validated soil mechanics absent.

## Slide 7 — Sensors and frames

**Key points:** RGB, depth, 720-beam 2D LiDAR, 100 Hz IMU, wheel odom; `map→odom→chassis→sensors`. **Visual:** SDF sensor placements or actual RViz RobotModel screenshot. **Speaker:** SDF camera baseline is 160×120; a measured runtime profile was 640×480—pin the profile when reproducing.

## Slide 8 — ROS 2 data flow

**Key points:** exact topics and message types; perception/map/planning/control chain. **Visual:** Mermaid architecture from `LunaBot_Architecture.md`. **Speaker:** planners request motion only via `/cmd_vel_in`.

## Slide 9 — Localization and mapping

**Key points:** `slam_toolbox` uses scan+odometry; occupancy `/map`; separate map/odom transforms; 5 cm map resolution in cited artifact. **Visual:** actual saved PGM map (`assets/phase_l_map.pgm`). **Speaker:** one map raster does not mean all cells are observed/free.

## Slide 10 — Semantic and geometric terrain understanding

**Key points:** regolith/rock/shadow inference; RGB-D projection; slope/roughness/crater/uncertainty layers. **Visual:** `assets/perception-iou.svg` and actual segmentation overlay screenshot (required). **Speaker:** mIoU 0.606 on one held-out analog corpus; not safety accuracy.

## Slide 11 — Terrain cost map and planning

**Key points:** weighted cost fusion; hard obstacle vs caution for unknown; occupancy A* baseline and terrain weighted A*. **Visual:** cost map and path RViz screenshot required. **Speaker:** cost score is a heuristic, not calibrated soil or rollover safety.

## Slide 12 — Obstacle detection and dynamic replanning

**Key points:** LiDAR obstacle grid; changed plan; stop while route invalid; freshness checks; two manual GUI passes reported 56.8 s and 75.9 s obstacle-replan intervals. **Visual:** `assets/replan-latency.svg` plus actual before/after RViz screenshot required. **Speaker:** offline planner milliseconds and live status latency use different scopes; WSL GUI intervals were high and variable.

## Slide 13 — Control safety and mission lifecycle

**Key points:** `/cmd_vel_in` → controller → `/cmd_vel` → native watchdog; mission pause/recover/estop/event store. **Visual:** safety chain diagram. **Speaker:** software safeguards, not certified flight/physical safety.

## Slide 14 — Mission demonstration

**Key points:** Habitat–Science–Habitat software lifecycle; manual RViz goal; obstacle event → replan → goal; no physical sample. **Visual:** actual Gazebo/RViz mission completion screenshots (required). **Speaker:** cite P8 three simulation round trips, nine legs, three durable events and the two 2026-11-19 full GUI gate passes as bounded evidence.

## Slide 15 — Experimental results

**Key points:** localization 0.2299 m short route; held-out mIoU 0.6057; four P6 captures; P8 round trips; 15 planner fixtures; two complete manual GUI gate passes. **Visual:** concise table and plots `scenario-pass-rates.svg`, `planner-path-lengths.svg`. **Speaker:** distinguish live missions from deterministic planner tests and mention the high, variable WSL replan timing.

## Slide 16 — What is not measured yet?

**Key points:** mission time; traveled trajectory error; collision count; clearance; recovery outcome; whole-stack resources. **Visual:** “NOT MEASURED” metric ledger. **Speaker:** absence is not zero/pass; name next instrumentation plan.

## Slide 17 — Limitations and research contribution

**Key points:** integration/evidence contribution; no novel A*/SLAM claim; simulation-to-real and lunar fidelity gaps; no sample arm. **Visual:** scope boundary graphic. **Speaker:** honest contribution improves technical credibility.

## Slide 18 — Roadmap, conclusion and questions

**Key points:** repeated end-to-end qualification; calibrated live projection and mobility; richer terrain-aware control; hardware sim-to-real. **Visual:** staged short/medium/long roadmap. **Speaker:** “LunaBot demonstrates an integrated research prototype; physical lunar capability remains future work.”

## Suggested timing

For a 12-minute evaluation, allow ~30 seconds on slides 1–3, ~45 seconds on slides 4–14, ~60 seconds on results/limitations, then 30 seconds on roadmap. Be ready to show source paths, run IDs, exact data scope and the difference between mission runs and grid fixtures.
