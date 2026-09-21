#!/usr/bin/env bash
# Convert a completed mapping artifact to PGM and open the lightweight 2D editor.
# Uses agt_mapping_framework packages only; no navigation-v3 workspace is required.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd -- "$ROOT/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
[[ -f "$ROS_SETUP" ]] || { echo "ROS setup not found: $ROS_SETUP (set ROS_SETUP)" >&2; exit 2; }
source "$ROS_SETUP"
for overlay in "$WORKSPACE/install/setup.bash" "$WORKSPACE/install_mapping_framework/setup.bash"; do
  if [[ -f "$overlay" ]]; then
    source "$overlay"
  fi
done
if ! ros2 pkg executables agt_mapping_bringup 2>/dev/null | grep -Fqx -- "agt_mapping_bringup mapping_review"; then
  echo "mapping_review is not built in the sourced overlays; rebuild agt_mapping_bringup first." >&2
  exit 2
fi
exec ros2 run agt_mapping_bringup mapping_review "$@"
