#!/usr/bin/env bash
# One-command RViz refinement editor and derived map exporter.
set -eo pipefail
source /opt/ros/humble/setup.bash

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 SOURCE_MAP_PACKAGE REFINED_MAP_PACKAGE" >&2
  exit 2
fi

SOURCE_PACKAGE=$(realpath "$1")
OUTPUT_PACKAGE=$(realpath -m "$2")
ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")/.." rev-parse --show-toplevel)"
WORKSPACE="$(cd "$ROOT/../.." && pwd)"

[[ -d "$SOURCE_PACKAGE" ]] || { echo "Source package does not exist: $SOURCE_PACKAGE" >&2; exit 2; }
[[ -f "$SOURCE_PACKAGE/map.pcd" ]] || { echo "Missing source map.pcd" >&2; exit 2; }
[[ -f "$WORKSPACE/install/setup.bash" ]] || {
  echo "Workspace is not built. Build agt_map_refinement_core and agt_map_editor first." >&2
  exit 2
}

source "$WORKSPACE/install/setup.bash"
ros2 launch agt_map_editor map_refinement_editor.launch.py \
  map_package:="$SOURCE_PACKAGE" \
  map_pcd:="$SOURCE_PACKAGE/map.pcd" \
  output_package:="$OUTPUT_PACKAGE" \
  refinement_file:="$OUTPUT_PACKAGE/refinement.yaml"