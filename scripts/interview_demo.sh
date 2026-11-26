#!/usr/bin/env bash
set -Ee

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_DIR"

source /opt/ros/humble/setup.bash
if [ -f "$REPO_DIR/install/setup.bash" ]; then
  source "$REPO_DIR/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-46}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-interview-demo"
RUN_DIR="${INTERVIEW_DEMO_DIR:-$REPO_DIR/evidence/runs/$RUN_ID}"
if [ -e "$RUN_DIR" ]; then
  echo "ERROR: evidence directory already exists: $RUN_DIR" >&2
  exit 2
fi
mkdir -p "$RUN_DIR"

printf 'INTERVIEW_DEMO_START run=%s\n' "$RUN_ID" | tee "$RUN_DIR/console.log"
printf 'ROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\n' \
  "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" > "$RUN_DIR/environment.txt"

# Keep Foxglove connected to the same WSL ROS domain throughout and after the
# mission. Start only when no service already owns the configured port.
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9091}"
if ! ss -ltn "sport = :$ROSBRIDGE_PORT" | grep -q LISTEN; then
  setsid ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    "port:=$ROSBRIDGE_PORT" > "$RUN_DIR/rosbridge.log" 2>&1 &
  ROSBRIDGE_PID=$!
  printf '%s\n' "$ROSBRIDGE_PID" > "$RUN_DIR/rosbridge.pid"
  ready=false
  for _ in $(seq 1 20); do
    if ss -ltn "sport = :$ROSBRIDGE_PORT" | grep -q LISTEN; then
      ready=true
      break
    fi
    sleep 1
  done
  if [ "$ready" != true ]; then
    echo "ERROR: Rosbridge failed to listen on port $ROSBRIDGE_PORT" | tee -a "$RUN_DIR/console.log" >&2
    exit 3
  fi
  echo "FOXGLOVE_BRIDGE_STARTED port=$ROSBRIDGE_PORT pid=$ROSBRIDGE_PID" \
    | tee -a "$RUN_DIR/console.log"
else
  echo "FOXGLOVE_BRIDGE_REUSED port=$ROSBRIDGE_PORT" | tee -a "$RUN_DIR/console.log"
fi

START_EPOCH="$(date +%s)"
set +e
timeout --signal=INT --kill-after=10s 270s \
  env HEADLESS=1 DEMO=1 EVIDENCE=1 PRESENTATION_FAST=1 \
      WSL_SOFTWARE_PROFILE=1 AUTO_GOAL_DISTANCE=1.0 \
      DEMO_AUTO_GOAL_DISTANCE=1.0 FOLLOWER_MAX_LINEAR=0.20 \
      EVIDENCE_DIR="$RUN_DIR" \
      bash "$REPO_DIR/scripts/launch-l.sh" \
  2>&1 | tee -a "$RUN_DIR/console.log"
RUN_STATUS="${PIPESTATUS[0]}"
set -e
ELAPSED="$(( $(date +%s) - START_EPOCH ))"
printf 'INTERVIEW_DEMO_EXIT status=%s elapsed_sec=%s\n' \
  "$RUN_STATUS" "$ELAPSED" | tee -a "$RUN_DIR/console.log"

if [ "$RUN_STATUS" -ne 0 ] || [ "$ELAPSED" -ge 300 ] || \
   ! grep -q 'PHASE L RUN COMPLETE - overall result: PASS' "$RUN_DIR/console.log" 2>/dev/null || \
   ! grep -q 'validation PASS: MISSION_DEMO_PASS' "$RUN_DIR/last_run.log" 2>/dev/null; then
  echo "INTERVIEW_DEMO_FAIL evidence=$RUN_DIR" | tee -a "$RUN_DIR/console.log" >&2
  exit 1
fi

echo "INTERVIEW_DEMO_PASS elapsed_sec=$ELAPSED evidence=$RUN_DIR" \
  | tee -a "$RUN_DIR/console.log"
