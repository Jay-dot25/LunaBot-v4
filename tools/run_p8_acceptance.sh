#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_DIR"

RUN_ID="${P8_ACCEPTANCE_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)-p803}"
RUN_ROOT="${P8_ACCEPTANCE_DIR:-$REPO_DIR/evidence/runs/$RUN_ID/P8-03}"
if [ -e "$RUN_ROOT" ]; then
  echo "ERROR: evidence directory already exists: $RUN_ROOT" >&2
  exit 2
fi
mkdir -p "$RUN_ROOT"

python3 "$REPO_DIR/scripts/waypoint_registry.py" \
  --registry "$REPO_DIR/config/waypoints.json" \
  --map-id lunar_p803_roundtrip --frame-id map --require-bound

for trial in 01 02 03; do
  trial_dir="$RUN_ROOT/trial-$trial"
  mkdir -p "$trial_dir"
  echo "P8_ACCEPTANCE_START trial=$trial run_id=$RUN_ID-trial-$trial"
  P8_ROUNDTRIP=1 P8_MAP_QUALIFICATION=0 P8_RUN_ID="$RUN_ID-trial-$trial" \
    HEADLESS=1 DEMO=1 EVIDENCE=1 EVIDENCE_DIR="$trial_dir" \
    bash "$REPO_DIR/scripts/launch-l.sh"
  python3 "$REPO_DIR/tools/verify_p8_acceptance.py" \
    --trial "$trial_dir" --trial-number "$trial" --validate-only
done

python3 "$REPO_DIR/tools/verify_p8_acceptance.py" \
  --run-root "$RUN_ROOT" --registry "$REPO_DIR/config/waypoints.json"
echo "P8_ACCEPTANCE_PASS run_id=$RUN_ID evidence=$RUN_ROOT"
