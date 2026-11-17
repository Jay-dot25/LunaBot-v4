#!/usr/bin/env bash
# All callers track owned children launched with setsid.
stop_group() {
  local group="${1:-}"
  [ -n "$group" ] || return 0
  kill -TERM -- "-$group" 2>/dev/null || kill -TERM "$group" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$group" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$group" 2>/dev/null || true
  wait "$group" 2>/dev/null || true
}

# Status topics may alternate between PASS and a temporary waiting state.
# Poll volatile status publishers until the requested evidence token is observed.
wait_for_topic_token() {
  local topic="$1" token="$2" timeout_sec="$3"
  local deadline=$((SECONDS + timeout_sec)) status
  while (( SECONDS < deadline )); do
    status="$(timeout 3 ros2 topic echo "$topic" \
      --qos-reliability best_effort --qos-durability volatile \
      --once 2>/dev/null || true)"
    if printf '%s\n' "$status" | grep -qF "$token"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}
