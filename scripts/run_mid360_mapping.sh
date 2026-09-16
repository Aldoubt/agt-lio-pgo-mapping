#!/usr/bin/env bash
# Start the validated MID360 -> FAST-LIO2 -> PGO mapping pipeline.
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 <MID360_ROSBAG_DIRECTORY> [OUTPUT_DIRECTORY]" >&2
  exit 2
fi

ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")/.." rev-parse --show-toplevel)"
WORKSPACE="$(cd "$ROOT/../.." && pwd)"
BAG_PATH="$(realpath "$1")"
OUTPUT_DIR="${2:-$WORKSPACE/experiments/artifacts/output/$(basename "$BAG_PATH")_$(date +%Y%m%d_%H%M%S)}"

if [[ ! -d "$BAG_PATH" ]]; then
  echo "Rosbag directory does not exist: $BAG_PATH" >&2
  exit 2
fi
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo "Workspace is not built. Run scripts/bootstrap.sh first." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
echo "Writing final artifact to: $OUTPUT_DIR/map_package"
echo "RViz opens with the live LIO cloud; wait for export completion before closing."
ros2 launch agt_mapping_bringup mapping_v0.launch.py \
  bag_path:="$BAG_PATH" output_dir:="$OUTPUT_DIR" start_rviz:=true auto_export:=true
