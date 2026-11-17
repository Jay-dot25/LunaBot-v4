# Phase F — Semantic Terrain Mapping (`launch-f`)

## 1. Scope and gate

Semantic terrain mapping is the Phase F addition to the approved Phase E
stack. It fuses the real Phase E segmentation mask with timestamped depth and
both cameras' `CameraInfo`, then projects visible observations into a fixed-size
rolling grid in the existing `map` frame. It does
not add SLAM, AMCL, Cartographer, a second localization source, a cost-map
planner, or a velocity controller.

The semantic, confidence, and age maps are aligned, auditable
`nav_msgs/OccupancyGrid` messages. The semantic values legend is:

```text
-1   unknown / not observed
25   terrain candidate
100  obstacle candidate
```

The mapper uses calibrated depth intrinsics, RGB/depth camera extrinsics, and
the existing timestamped `map -> depth optical frame` TF chain. It publishes
no `/cmd_vel` or `/cmd_vel_in`
messages. Terrain-aware cost fusion remains Phase G; semantic map output is
not silently connected to the approved A* planner.

Phase F is independently launchable as `~/launch-f`. Static validation is not
runtime acceptance. The workstation runtime produced real semantic-map and
status messages, preserved the approved Phase A-E gates, and cleanly relaunched;
Phase F is approved before Phase G implementation.

## 2. Data flow

```text
Phase E /lunabot/terrain/segmentation ─────┐
Phase E /lunabot/terrain/segmentation_confidence ─┤
/lunabot/depth/image_raw ──────────────────┼──> lunabot_semantic_terrain_mapper
/lunabot/camera/camera_info ───────────────┤        │
/lunabot/depth/camera_info ────────────────┘        ├─ /lunabot/terrain/semantic_map
map/depth/RGB optical TF ──────────────────────────└─ /lunabot/terrain/semantic_map/status
                                                           ├─ /lunabot/terrain/semantic_map/confidence
                                                           └─ /lunabot/terrain/semantic_map/age

Approved navigation path remains unchanged:
slam_toolbox -> /map -> A* -> /cmd_vel_in -> Phase B controller -> /cmd_vel
```

Phase E publishes an aligned mono8 decision-strength confidence image for each
mask (0 unknown, 1–255 known). This rule-segmenter score is a heuristic margin,
not a calibrated probability. The mapper requires confidence, RGB-mask, and
depth timestamps to be within 75 ms and drops inputs older than 0.5 s. It reads
focal lengths and principal points from each
camera's `CameraInfo`; it does not infer calibration from nominal FOV. Each
valid depth pixel is projected through the depth camera model, transformed from
the depth optical frame into the RGB optical frame using TF at both capture
times, and registered into the segmentation mask. An RGB-space z-buffer keeps
the nearest visible depth return so surfaces hidden by nearer returns do not
inherit their labels. A second timestamped TF projects depth points into `map`,
including the camera pitch and articulated mast transform. Obstacle evidence
dominates terrain evidence when both reach the same semantic cell.

The default live grid stays 160x160 cells at 0.1 m (16x16 m) and recenters on
the timestamped depth-camera pose in `map`, preserving overlap and clearing
cells that leave the window. Its label, confidence, observation time, elevation,
and elevation time arrays use 640,000 bytes regardless of mission distance.
This is a rover-local rolling
coverage layer; cells outside its window are discarded. Confidence has a 10 s
half-life, and observations expire after 30 s or when effective confidence
drops below 0.35. Confidence and age outputs share semantic-grid geometry:
confidence is 0–100; age is whole seconds capped at 100; -1 means unknown.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/segmentation` | `sensor_msgs/Image` | Phase E labels: 1 terrain, 2 obstacle |
| `/lunabot/terrain/segmentation_confidence` | `sensor_msgs/Image` | aligned mono8 decision-strength confidence |
| `/lunabot/depth/image_raw` | `sensor_msgs/Image` | real depth range for projection |
| `/lunabot/camera/camera_info` | `sensor_msgs/CameraInfo` | RGB intrinsics and frame identity |
| `/lunabot/depth/camera_info` | `sensor_msgs/CameraInfo` | depth intrinsics and frame identity |
| depth optical -> RGB optical TF | `tf2` | RGB/depth registration across capture timestamps |
| `map -> depth optical` TF | `tf2` | timestamped map projection with camera pitch |
| `/lunabot/terrain/semantic_map` | `nav_msgs/OccupancyGrid` | accumulated semantic grid |
| `/lunabot/terrain/semantic_map/confidence` | `nav_msgs/OccupancyGrid` | aligned effective confidence, 0–100 |
| `/lunabot/terrain/semantic_map/age` | `nav_msgs/OccupancyGrid` | aligned observation age in seconds |
| `/lunabot/terrain/semantic_map/slope` | `nav_msgs/OccupancyGrid` | aligned local elevation-gradient risk 0–100 |
| `/lunabot/terrain/semantic_map/roughness` | `nav_msgs/OccupancyGrid` | aligned local height-spread risk 0–100 |
| `/lunabot/terrain/semantic_map/crater_risk` | `nav_msgs/OccupancyGrid` | aligned bowl-depression candidate risk 0–100 |
| `/lunabot/terrain/semantic_map/uncertainty` | `nav_msgs/OccupancyGrid` | aligned confidence, age, and geometry uncertainty |
| `/lunabot/terrain/semantic_map/status` | `std_msgs/String` | frame, sample, and class-cell statistics |

The semantic map and status are Reliable + Transient Local for late-joining
RViz/runtime diagnostics. A successful status contains `SEMANTIC_MAP_PASS` and
reports accumulated observations, confidence rejects, cleared cells, rolling
window/memory, terrain cells, obstacle cells, resolution, and the map frame.

## 4. Installation

Phase F uses the approved Phase E workstation dependencies. No additional ML
or mapping package is required.

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-f" ~/launch-f
chmod +x launch-f scripts/launch-f.sh scripts/semantic_terrain_mapper.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_f.py
```

The validator checks the approved Phase A-E baseline, independent launch
behavior, semantic projection implementation, map/status QoS, real runtime
checks, RViz output, evidence paths, and honest runtime gating. It never claims
that a real semantic observation was mapped.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f
```

The run starts the approved world, bridge, static TF, controller, odometry
monitor, Phase E segmentation, `slam_toolbox`, semantic mapper, and A*
directly. It must validate the inherited Phase A-E interfaces and these Phase F
interfaces:

```text
/lunabot/terrain/semantic_map        nav_msgs/OccupancyGrid
/lunabot/terrain/semantic_map/status std_msgs/String
SEMANTIC_MAP_PASS
A* goal reached: PASS
live map updates during autonomous navigation: PASS
saved map evidence (YAML + PGM): PASS
PHASE F RUN COMPLETE - overall result: PASS
Launch F environment cleanly closed.
```

The semantic-map check requires real `ros2 topic type` and
`ros2 topic echo --once` messages. A mapper process being alive is not a pass.

## 7. GUI/RViz run

After the headless gate passes:

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-f
```

RViz retains the Phase E map, A* path, camera, LiDAR, odometry, segmentation
overlay, and TF displays. It adds `Semantic Terrain Map` on
`/lunabot/terrain/semantic_map` with a costmap color scheme. The semantic map
is visualization/evidence output only; it does not change A* behavior in Phase
F.

Confirm that the semantic grid updates as segmentation frames arrive, the
status reports increasing frame and observation counts, and the inherited
navigation outputs remain error-free. Press Ctrl+C for clean shutdown.

## 8. Evidence

Runtime evidence is written to `evidence/phase-f-launch-f/`:

| File | Meaning |
|---|---|
| `semantic_mapping.log` | semantic mapper diagnostics |
| `semantic_map_sample.txt` | real semantic `OccupancyGrid` sample |
| `semantic_map_status.txt` | real `SEMANTIC_MAP_PASS` status sample |
| `segmentation_confidence_sample.txt` | aligned Phase E confidence image |
| `semantic_confidence_sample.txt` / `semantic_age_sample.txt` | aligned map layers |
| `segmentation_sample.txt` | inherited Phase E mask sample |
| `segmentation_status.txt` | inherited Phase E status sample |
| `depth_sample.txt` | real depth image sample |
| `topics.txt` | runtime topic graph |
| `last_run.log` | ordered launcher and validation result |
| `map_before_navigation.txt` / `map_after_navigation.txt` | inherited A* map-motion gate |
| `phase_f_map.yaml` / `phase_f_map.pgm` | final SLAM map evidence, when available |
| `static_validation.txt` | generated static Phase F report |

Do not hand-edit runtime evidence. Inspect `semantic_mapping.log`,
`segmentation.log`, bridge/Gazebo logs, and `last_run.log` when a check fails.

## 9. Clean relaunch gate

After the autonomous or GUI run:

```text
Launch F environment cleanly closed.
```

Then repeat:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f
```

Both runs must start one semantic mapper, publish real semantic-map/status
messages, preserve the A* and Phase E gates, and stop all process groups. This
clean relaunch is part of the Phase F runtime gate.

## 10. Acceptance checklist

- [ ] Phase A-E static and runtime contracts remain passing.
- [ ] Segmentation and depth are real input messages.
- [ ] RGB/depth CameraInfo dimensions and intrinsics match their images.
- [ ] Mask/depth timestamp skew and input age are bounded.
- [ ] Timestamped map/depth-optical and depth-to-RGB-optical TF are used.
- [ ] RGB z-buffering rejects depth samples hidden behind nearer returns.
- [ ] Semantic map is a real non-empty `nav_msgs/OccupancyGrid`.
- [ ] Semantic map status contains `SEMANTIC_MAP_PASS`.
- [ ] Confidence and age layers match semantic geometry and recenter together.
- [ ] Low-confidence, decayed, and expired observations become unknown.
- [ ] Rolling coverage follows the rover while storage remains bounded.
- [ ] Terrain and obstacle cell counts are reported.
- [ ] RViz displays the semantic map without changing navigation.
- [ ] A* still reaches its goal through the Phase B controller boundary.
- [ ] Final map evidence is saved when the map-saver package is available.
- [ ] Clean shutdown and a second independent relaunch pass.

Phase F's runtime gate is explicitly approved. A separate Phase G request has
started terrain cost-map implementation; Phase G must complete its own runtime
gate before Phase H.
