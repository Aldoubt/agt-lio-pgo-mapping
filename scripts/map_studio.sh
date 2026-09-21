#!/usr/bin/env bash
# Launch AGT Map Studio from this repository's overlay.
# Usage: scripts/map_studio.sh [--package DIR | --pcd FILE | --session studio_session.yaml] [-- studio args]
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd -- "$ROOT/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
[[ -f "$ROS_SETUP" ]] || { echo "ROS setup not found: $ROS_SETUP (set ROS_SETUP)" >&2; exit 2; }
# Humble setup scripts may read unset variables; do not enable nounset here.
source "$ROS_SETUP"
for overlay in "$WORKSPACE/install/setup.bash" "$WORKSPACE/install_mapping_framework/setup.bash"; do
  if [[ -f "$overlay" ]]; then
    source "$overlay"
  fi
done
export AGT_MAP_ROOT="${AGT_MAP_ROOT:-$WORKSPACE/maps}"
if ! ros2 pkg executables agt_map_studio 2>/dev/null | grep -Fqx -- "agt_map_studio map_viewer"; then
  echo "agt_map_studio is not built in the sourced overlays; build it first." >&2
  exit 2
fi
exec ros2 run agt_map_studio map_viewer "$@"
