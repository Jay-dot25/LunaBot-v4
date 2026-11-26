#!/usr/bin/env bash
# Project-local Mesa workaround for RViz's indexed-map sampler link error.
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
resource_prefix="$(ros2 pkg prefix rviz_rendering)"
overlay="$REPO_DIR/.build/rviz_software_overlay"
mkdir -p "$overlay/share/ament_index/resource_index/packages"
if [ ! -f "$overlay/sampler-binding-v1" ]; then
  mkdir -p "$overlay/share/rviz_rendering"
  cp -r "$resource_prefix/share/rviz_rendering/ogre_media" "$overlay/share/rviz_rendering/"
  python3 - "$overlay" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1])
shader = root / 'share/rviz_rendering/ogre_media/materials/glsl120/indexed_8bit_image.frag'
text = shader.read_text()
text = text.replace('#version 120', '#version 120\n#extension GL_ARB_shading_language_420pack : require')
text = text.replace('uniform sampler2D eight_bit_image;', 'layout(binding = 0) uniform sampler2D eight_bit_image;')
text = text.replace('uniform sampler1D palette;', 'layout(binding = 1) uniform sampler1D palette;')
shader.write_text(text)
(root / 'sampler-binding-v1').touch()
PY
fi
touch "$overlay/share/ament_index/resource_index/packages/rviz_rendering"
export AMENT_PREFIX_PATH="$overlay:${AMENT_PREFIX_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
exec rviz2 -d "${LUNABOT_RVIZ_CONFIG:-$REPO_DIR/rviz/phase_d.rviz}" -f map --ros-args -p use_sim_time:=true "$@"
