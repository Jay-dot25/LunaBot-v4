#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
set +u
source /opt/ros/humble/setup.bash
set -u
source "$ROOT/scripts/p502_runtime.sh"
if ! p502_prepare_runtime "$ROOT"; then
  echo "qualified P5-03 runtime unavailable" >&2
  exit 2
fi
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

RUN_ROOT="${P6_EVIDENCE_DIR:-$ROOT/evidence/runs/20261116-p6-sim-calibration}"
P6_BOWL_X="${P6_BOWL_X:-10.5}"
mkdir -p "$RUN_ROOT"
PLANE_WORLD="$RUN_ROOT/p6_known_plane/lunar_world.sdf"
"$P502_PYTHON" "$ROOT/tools/generate_p6_known_plane.py" \
  --output "$PLANE_WORLD" --height-m -2.788 --extent-m 40
ACTIVE_LAUNCH_PID=""
cleanup() {
  if [ -n "$ACTIVE_LAUNCH_PID" ] && kill -0 "$ACTIVE_LAUNCH_PID" 2>/dev/null; then
    # launch-f is started in its own session, so signal its complete process
    # group and bound cleanup even if a ROS child ignores the shell trap.
    kill -TERM -- "-$ACTIVE_LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 45); do
      kill -0 "$ACTIVE_LAUNCH_PID" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$ACTIVE_LAUNCH_PID" 2>/dev/null; then
      kill -KILL -- "-$ACTIVE_LAUNCH_PID" 2>/dev/null || true
      kill -KILL "$ACTIVE_LAUNCH_PID" 2>/dev/null || true
    fi
    wait "$ACTIVE_LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

BOWL_GROUND_Z="$($P502_PYTHON - "$P6_BOWL_X" <<'PY'
import sys
sys.path.insert(0, "tools")
import generate_lunar_terrain as terrain
_, _, z, *_ = terrain.generate_terrain(42)
print(f"{float(terrain.sample_height_grid(z, [float(sys.argv[1])], [10.0])[0, 0]):.6f}")
PY
)"
BOWL_SPAWN_Z="$(awk -v z="$BOWL_GROUND_Z" 'BEGIN {printf "%.6f", z + 0.48}')"
BOWL_CHASSIS_Z="$(awk -v z="$BOWL_GROUND_Z" 'BEGIN {printf "%.6f", z + 0.48}')"

run_capture() {
  local run_id="$1" kind="$2" x="$3" y="$4" yaw="$5" model_z="$6"
  local truth_x="$7" truth_y="$8" truth_z="$9"
  local truth_roll="${10:-0}" truth_pitch="${11:-0}"
  local world_path="${12:-}" surface="${13:-terrain}"
  local skip_projection="${14:-false}"
  local truth_yaw="$yaw"
  local run_dir="$RUN_ROOT/$run_id"
  mkdir -p "$run_dir"
  echo "=== P6 calibration capture: $run_id ==="
  setsid env \
    EVIDENCE_DIR="$run_dir/phase-f" HEADLESS=1 DEMO=0 EVIDENCE=0 AUTO_GOAL=false \
    SPAWN_X="$x" SPAWN_Y="$y" SPAWN_YAW="$yaw" SPAWN_Z="$model_z" \
    WORLD_PATH_OVERRIDE="$world_path" MAPPER_SAMPLE_STRIDE=2 \
    bash "$ROOT/scripts/launch-f.sh" \
    > "$run_dir/launcher.stdout.log" 2>&1 &
  ACTIVE_LAUNCH_PID=$!

  local ready=0
  for _ in $(seq 1 240); do
    if grep -q "semantic mapper running" "$run_dir/launcher.stdout.log" 2>/dev/null; then
      ready=1
      break
    fi
    if ! kill -0 "$ACTIVE_LAUNCH_PID" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if [ "$ready" -ne 1 ]; then
    echo "Phase F did not reach the semantic mapper; see $run_dir/launcher.stdout.log" >&2
    tail -80 "$run_dir/launcher.stdout.log" >&2 || true
    cleanup
    ACTIVE_LAUNCH_PID=""
    return 1
  fi
  sleep 8
  # Gazebo settles the rover after spawning, so use its actual world pose as
  # the reference-frame transform instead of the requested spawn pose.
  truth_pose="$( { timeout 4 ign topic -e -t /world/lunar_world/pose/info 2>/dev/null || true; } | awk '
    /name: "lunabot_v4"/ { found=1; next }
    found && /position \{/ { position=1; next }
    position && /x:/ { x=$2 }
    position && /y:/ { y=$2 }
    position && /z:/ { z=$2 }
    orientation && /x:/ { qx=$2 }
    orientation && /y:/ { qy=$2 }
    orientation && /z:/ { qz=$2 }
    orientation && /w:/ {
      qw=$2
      if (x != "" && y != "" && z != "") {
        yaw=atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        roll=atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))
        s=2*(qw*qy-qz*qx); if (s > 1) s=1; if (s < -1) s=-1
        pitch=atan2(s,sqrt(1-s*s))
        printf "%.9f %.9f %.9f %.9f %.9f %.9f\n",x,y,z,roll,pitch,yaw
      }
      exit
    }
    position && /\}/ { position=0 }
    found && /orientation \{/ { orientation=1; next }
  ' || true)"
  if [ -n "$truth_pose" ]; then
    read -r truth_x truth_y truth_z truth_roll truth_pitch truth_yaw <<< "$truth_pose"
  fi
  printf 'x=%s y=%s z=%s roll=%s pitch=%s yaw=%s\n' \
    "$truth_x" "$truth_y" "$truth_z" "$truth_roll" "$truth_pitch" "$truth_yaw" \
    > "$run_dir/gazebo_truth_pose.txt"
  set +e
  "$P502_PYTHON" "$ROOT/scripts/p6_calibration_recorder.py" \
    --output "$run_dir" --run-id "$run_id" --capture-kind "$kind" \
    --duration-s 45 --truth-world-x-offset-m "$truth_x" \
    --truth-world-y-offset-m "$truth_y" --truth-world-z-offset-m "$truth_z" \
    --truth-world-yaw-offset-rad "$truth_yaw" \
    --truth-world-roll-offset-rad "$truth_roll" \
    --truth-world-pitch-offset-rad "$truth_pitch" \
    --reference-surface "$surface" --plane-world-z-m -2.788 \
    $([ "$skip_projection" = true ] && echo --skip-projection-gates) \
    > "$run_dir/recorder.log" 2>&1
  local recorder_rc=$?
  set -e
  kill -TERM -- "-$ACTIVE_LAUNCH_PID" 2>/dev/null || true
  set +e
  for _ in $(seq 1 45); do
    kill -0 "$ACTIVE_LAUNCH_PID" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$ACTIVE_LAUNCH_PID" 2>/dev/null; then
    kill -KILL -- "-$ACTIVE_LAUNCH_PID" 2>/dev/null || true
    kill -KILL "$ACTIVE_LAUNCH_PID" 2>/dev/null || true
  fi
  wait "$ACTIVE_LAUNCH_PID"
  local launch_rc=$?
  set -e
  ACTIVE_LAUNCH_PID=""
  printf '{"recorder_exit_code":%d,"launcher_exit_code":%d}\n' \
    "$recorder_rc" "$launch_rc" > "$run_dir/process_status.json"
  cat "$run_dir/projection_summary.json" 2>/dev/null || tail -80 "$run_dir/recorder.log"
}

# Repeat the flat capture at the same calibrated pose after a clean process
# restart. Observe Crater A from its west rim so the target stays visible.
if [ "${P6_ONLY_BOWL:-0}" != "1" ]; then
  run_capture flat_origin flat 0.0 0.0 0.0 -2.308 0.0 0.0 -2.308 0 0 "$PLANE_WORLD" plane || true
  run_capture flat_forward flat 0.0 0.0 0.0 -2.308 0.0 0.0 -2.308 0 0 "$PLANE_WORLD" plane || true
  run_capture flat_repeat flat 0.0 0.0 0.0 -2.308 0.0 0.0 -2.308 0 0 "$PLANE_WORLD" plane || true
fi
run_capture bowl_center_geometry bowl "$P6_BOWL_X" 10.0 0.0 "$BOWL_SPAWN_Z" \
  "$P6_BOWL_X" 10.0 "$BOWL_CHASSIS_Z" 0 0 "" terrain true || true

"$P502_PYTHON" - "$RUN_ROOT" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
summaries = []
for path in sorted(root.glob("*/projection_summary.json")):
    summaries.append(json.loads(path.read_text(encoding="utf-8")))
overall = {
    "schema_version": 1,
    "gate_id": json.loads((root.parents[2] / "config/p6_calibration_gates.json").read_text(encoding="utf-8"))["gate_id"],
    "capture_runs": summaries,
    "result": "PASS" if len([s for s in summaries if s.get("reference_surface") == "plane"])
        >= 3 and all(s.get("result") == "PASS" for s in summaries) else "FAIL",
}
(root / "summary.json").write_text(json.dumps(overall, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"result": overall["result"], "run_count": len(summaries)}, indent=2))
sys.exit(0 if overall["result"] == "PASS" else 1)
PY
