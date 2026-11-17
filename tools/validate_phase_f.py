#!/usr/bin/env python3
from static_contracts import shared_shutdown
"""Static Phase F gate: semantic terrain mapping.

This validator checks the independent launch-f contract and the approved
Phase A-E baseline. It never claims that a real semantic observation was
mapped; that requires the workstation command documented in
 docs/phase-6-launch-f.md.
"""

from pathlib import Path
import ast
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
checks = []


def check(name, ok, detail=""):
    checks.append((name, bool(ok), detail))


def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8")


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)


required = [
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh",
    "scripts/launch-d.sh", "scripts/launch-e.sh", "scripts/launch-f.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py",
    "scripts/odometry_monitor.py", "scripts/astar_navigation.py",
    "scripts/terrain_segmentation.py", "scripts/semantic_terrain_mapper.py",
    "scripts/semantic_projection.py", "scripts/semantic_layers.py",
    "scripts/terrain_geometry.py", "tests/test_semantic_projection.py",
    "tests/test_semantic_layers.py", "tests/test_terrain_geometry.py",
    "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz",
    "rviz/phase_d.rviz", "rviz/phase_e.rviz", "rviz/phase_f.rviz",
    "tools/validate_phase_a.py", "tools/validate_phase_b.py",
    "tools/validate_phase_c.py", "tools/validate_phase_d.py",
    "tools/validate_phase_e.py", "docs/phase-3-launch-c.md",
    "docs/phase-4-launch-d.md", "docs/phase-5-launch-e.md",
    "docs/phase-6-launch-f.md", "evidence/phase-e-launch-e/README.md",
    "evidence/phase-f-launch-f/README.md",
    "evidence/phase-f-launch-f/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
    "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-f", "scripts/launch-f.sh",
            "scripts/semantic_terrain_mapper.py", "tools/validate_phase_f.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/semantic_terrain_mapper.py", "scripts/semantic_projection.py",
            "scripts/semantic_layers.py", "tests/test_semantic_layers.py",
            "scripts/terrain_geometry.py", "tests/test_terrain_geometry.py",
            "tests/test_semantic_projection.py", "scripts/terrain_segmentation.py",
            "scripts/astar_navigation.py", "scripts/control_odometry.py",
            "scripts/odometry_monitor.py", "tools/validate_phase_a.py",
            "tools/validate_phase_b.py", "tools/validate_phase_c.py",
            "tools/validate_phase_d.py", "tools/validate_phase_e.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f",
            "scripts/launch-a.sh", "scripts/launch-b.sh",
            "scripts/launch-c.sh", "scripts/launch-d.sh",
            "scripts/launch-e.sh", "scripts/launch-f.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

for rel, expected in [("tools/validate_phase_a.py", "96/96"),
                      ("tools/validate_phase_b.py", "103/103"),
                      ("tools/validate_phase_c.py", "133/133"),
                      ("tools/validate_phase_d.py", "152/152"),
                      ("tools/validate_phase_e.py", "112/112")]:
    r = run([sys.executable, rel])
    check(f"approved baseline remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

launch = read("scripts/launch-f.sh")
wrapper = read("launch-f")
mapper = read("scripts/semantic_terrain_mapper.py")
rviz = read("rviz/phase_f.rviz")
docs = read("docs/phase-6-launch-f.md")
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))

# Independent launch and inherited safety.
check("root launch-f resolves its real path", "readlink -f" in wrapper and
      "SCRIPT_PATH" in wrapper)
check("root launch-f execs scripts/launch-f.sh", "scripts/launch-f.sh" in wrapper)
check("Phase F does not invoke an earlier launcher", "launch-e" not in noncomment and
      "launch-d" not in noncomment and "launch-c" not in noncomment and
      "launch-b" not in noncomment and "launch-a" not in noncomment)
check("Phase F has its own evidence directory", "phase-f-launch-f" in launch and
      "phase-e-launch-e" not in noncomment)
check("Phase F is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase F uses process groups", "setsid" in launch and shared_shutdown(launch))
check("Phase F has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase F cleans mapper process", "MAPPER_PID" in launch and
      'stop_group "$MAPPER_PID"' in launch)
check("Phase F reports its clean shutdown", "Launch F environment cleanly closed." in launch)
check("Phase F has bounded shutdown", shared_shutdown(launch))
check("Phase F preserves Phase E perception", 'PERCEPTION_PATH="$REPO_DIR/scripts/terrain_segmentation.py"' in launch)
check("Phase F preserves the Phase D planner", 'NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"' in launch)
check("Phase F preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch and
      "cartographer" not in (launch + mapper).lower() and
      "nav2_amcl" not in (launch + mapper).lower())

# Semantic mapper implementation.
check("mapper is a ROS node", "class SemanticTerrainMapper(Node)" in mapper)
check("mapper consumes the Phase E mask", "Image, self.mask_topic" in mapper and
      "_mask_callback" in mapper)
check("mapper consumes synchronized confidence", "Image, self.confidence_topic" in mapper and
      "confidence_stamp_ns" in mapper and "confidence_image=confidence" in mapper)
check("Phase E publishes confidence samples", "self.confidence_pub.publish" in read("scripts/terrain_segmentation.py") and
      "confidence_topic" in read("scripts/terrain_segmentation.py"))
check("mapper consumes depth", "Image, self.depth_topic" in mapper and
      "_depth_callback" in mapper)
check("mapper consumes RGB/depth CameraInfo", "CameraInfo, self.rgb_info_topic" in mapper and
      "CameraInfo, self.depth_info_topic" in mapper)
projection = read("scripts/semantic_projection.py")
check("mapper decodes 16/32/64-bit depth in metres", "decode_depth_buffer" in mapper and
      all(token in projection for token in ("32fc1", "64fc1", "16uc1")))
check("mapper uses calibrated focal lengths/principal points", "CameraIntrinsics.from_k" in mapper and
      "horizontal_fov" not in mapper)
check("mapper uses exact-time map TF and two-time RGB registration", "lookup_transform(target, source, stamp)" in mapper and
      "lookup_transform_full" in mapper and "Time()" not in mapper)
check("mapper enforces camera timestamp skew and input age", "max_sync_skew_ns" in mapper and
      "max_input_age_ns" in mapper and "WAITING_FOR_SYNC" in mapper)
check("projection z-buffers RGB occlusion", "np.minimum.at" in projection and
      "WAITING_FOR_TF" in projection and "project_synced_frame" in mapper)
layers = read("scripts/semantic_layers.py")
check("mapper bounds memory and recenters coverage", "self.grid.recenter" in mapper and
      "memory_bytes" in mapper and "np.zeros((height, width)" in layers)
check("semantic layers age, decay, clear, and confidence-gate cells",
      all(token in layers for token in ("half_life_ns", "ttl_ns", "min_confidence",
                                        "confidence_data", "age_data")))
check("mapper publishes semantic/confidence/age occupancy grids",
      "self.map_pub" in mapper and "self.confidence_map_pub" in mapper and
      "self.age_map_pub" in mapper and "OccupancyGrid" in mapper and
      all(topic in mapper for topic in ("semantic_map/confidence", "semantic_map/age")))
check("mapper derives aligned slope/roughness/crater/uncertainty layers",
      "derive_terrain_layers" in mapper and
      all(f'"{layer}"' in mapper for layer in
          ("slope", "roughness", "crater_risk", "uncertainty")))
check("mapper defines semantic values", "TERRAIN = 1" in mapper and
      "OBSTACLE = 2" in mapper and "25" in mapper and "100" in mapper)
check("mapper gives obstacles precedence", "value == OBSTACLE or index not in cells" in projection)
check("mapper publishes auditable status", "SEMANTIC_MAP_PASS" in mapper and
      "self.status_pub" in mapper)
check("mapper map/status are late-join safe", mapper.count("TRANSIENT_LOCAL") >= 2 and
      "map_qos" in mapper and "status_qos" in mapper)
check("mapper does not publish motion commands", "/cmd_vel" not in mapper)

# Phase F launcher and runtime evidence.
check("Phase F resolves mapper path", 'MAPPER_PATH="$REPO_DIR/scripts/semantic_terrain_mapper.py"' in launch)
check("Phase F starts mapper directly", 'python3 "$MAPPER_PATH" --ros-args' in launch)
check("Phase F validates semantic map type", 'type_ok "type semantic map nav_msgs/OccupancyGrid"' in launch)
check("Phase F validates semantic status type", 'type_ok "type semantic map status std_msgs/String"' in launch)
check("Phase F validates real semantic messages", 'topic_ok "topic /lunabot/terrain/semantic_map"' in launch and
      'topic_ok "topic /lunabot/terrain/semantic_map/status"' in launch)
check("Phase F validates semantic content", "SEMANTIC_MAP_PASS" in launch and
      "semantic map content: PASS" in launch)
check("Phase F validates confidence and age layer interfaces",
      all(topic in launch for topic in ("segmentation_confidence",
                                        "semantic_map/confidence", "semantic_map/age")))
check("Phase F records semantic evidence", "semantic_map_sample.txt" in launch and
      "semantic_map_status.txt" in launch and "semantic_mapping.log" in launch)
check("Phase F retains segmentation content gate", "SEGMENTATION_PASS" in launch and
      "terrain segmentation content: PASS" in launch)
check("Phase F retains A* goal gate", "wait_for_goal" in launch and
      "A* goal reached" in launch)
check("Phase F retains map motion evidence", "map_before_navigation.txt" in launch and
      "map_after_navigation.txt" in launch)

# RViz and documentation.
check("Phase F RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase F RViz keeps A* path", "Name: A* Path" in rviz and "Value: /plan" in rviz)
check("Phase F RViz keeps segmentation overlay", "Terrain Segmentation Overlay" in rviz and
      "Value: /lunabot/terrain/overlay" in rviz)
check("Phase F RViz shows semantic map", "Semantic Terrain Map" in rviz and
      "Value: /lunabot/terrain/semantic_map" in rviz)
check("Phase F RViz uses sensor-compatible QoS", rviz.count("Reliability Policy: Best Effort") >= 4)
for phrase, name in [
    ("Semantic terrain mapping", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f", "automated command"),
    ("SEMANTIC_MAP_PASS", "semantic acceptance"),
    ("/lunabot/terrain/semantic_map", "semantic map output"),
    ("CameraInfo", "calibrated camera input"),
    ("depth optical", "camera TF input"),
    ("semantic values", "legend"),
    ("clean relaunch", "relaunch"),
    ("Phase G", "Phase G gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "never claims" in read("tools/validate_phase_f.py") and
      "workstation" in read("tools/validate_phase_f.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE F STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-f-launch-f/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
