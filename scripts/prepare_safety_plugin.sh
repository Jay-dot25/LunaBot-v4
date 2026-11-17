#!/usr/bin/env bash
# Sourced by legacy launchers after ROS setup, before starting Gazebo.
LUNABOT_NATIVE_BUILD="$REPO_DIR/.build/native_safety"
cmake -S "$REPO_DIR/src/lunabot_gazebo" -B "$LUNABOT_NATIVE_BUILD" \
  > "$EVIDENCE_DIR/native_build.log" 2>&1 &&
cmake --build "$LUNABOT_NATIVE_BUILD" --parallel 2 \
  >> "$EVIDENCE_DIR/native_build.log" 2>&1 || {
  echo "ERROR: actuator watchdog build failed; see $EVIDENCE_DIR/native_build.log"
  exit 1
}
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$LUNABOT_NATIVE_BUILD${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
