#!/usr/bin/env bash
# Shared setup for the active P5-03 three-class TorchScript runtime.

p502_prepare_runtime() {
  local repo_dir="$1"
  P502_THREECLASS_ARCHIVE="$repo_dir/evidence/runs/20261112-p503/P5-03/p502-threeclass-runtime.zip"
  P502_THREECLASS_PACKAGE="$repo_dir/.build/p502-threeclass-runtime"
  P502_PYTHON="${P502_PYTHON:-$repo_dir/.build/p502-training-venv/bin/python}"

  [ -f "$P502_THREECLASS_ARCHIVE" ] || return 1
  mkdir -p "$P502_THREECLASS_PACKAGE" || return 1
  unzip -oq "$P502_THREECLASS_ARCHIVE" -d "$P502_THREECLASS_PACKAGE" || return 1
  "$P502_PYTHON" -c 'import torch, rclpy, PIL, numpy' >/dev/null 2>&1 || return 1
  export P502_MODEL_PACKAGE="$P502_THREECLASS_PACKAGE"
}
