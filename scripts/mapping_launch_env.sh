#!/usr/bin/env bash
# Internal argv-only environment loader, invoked by the Python preflight CLI.
set -eo pipefail
if [[ $# -lt 3 ]]; then
  echo "Internal usage: mapping_launch_env.sh ROS_SETUP OVERLAY_SETUP COMMAND [ARG ...]" >&2
  exit 2
fi
ROS_SETUP=$1
OVERLAY_SETUP=$2
shift 2
# Humble setup scripts may read unset variables; do not enable nounset here.
source "$ROS_SETUP"
source "$OVERLAY_SETUP"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS environment has no ros2 executable: $ROS_SETUP" >&2
  exit 2
fi
executables="$(ros2 pkg executables agt_mapping_bringup)"
helpers=(mapping_wait_ready mapping_export_verified)
live_mode=0
for argument in "$@"; do
  if [[ "$argument" == mapping_live_mid360.launch.py ||
        "$argument" == mapping_live_yhs_mid360.launch.py ]]; then
    live_mode=1
  fi
done
if [[ $live_mode -eq 1 ]]; then
  helpers+=(mapping_live_supervisor)
fi
for helper in "${helpers[@]}"; do
  if ! grep -Fqx -- "agt_mapping_bringup $helper" <<< "$executables"; then
    echo "Selected overlay is stale: missing agt_mapping_bringup/$helper" >&2
    echo "Rebuild agt_mapping_bringup and agt_mapping_artifacts into $OVERLAY_SETUP, then retry." >&2
    exit 2
  fi
done
if [[ $live_mode -eq 1 ]] && ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  echo "Live mode requires livox_ros_driver2 in the sourced overlay (src/external/livox_ros_driver2)." >&2
  exit 2
fi
exec "$@"
