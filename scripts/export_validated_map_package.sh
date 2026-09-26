#!/usr/bin/env bash
# Mapping-owned release entry point. The native BBS implementation is invoked
# as an installed executable; no navigation package is imported by the mapper.
set -euo pipefail

if [[ $# -lt 4 ]]; then
  echo "usage: $0 SOURCE_MAP_PACKAGE SITE VERSION EXPERIMENT_ROOT [RELOCALIZATION_DIR]" >&2
  exit 2
fi

SOURCE_PACKAGE=$1
SITE=$2
VERSION=$3
EXPERIMENT_ROOT=$(realpath -m -- "$4")
case "$EXPERIMENT_ROOT/" in
  /home/yangxuan/ros2_ws/experiments/*) ;;
  *) echo "candidate output must be under /home/yangxuan/ros2_ws/experiments/" >&2; exit 2 ;;
esac
RELOCALIZATION_DIR=${5:-"$EXPERIMENT_ROOT/.staging-relocalization/$SITE/$VERSION"}

mkdir -p "$RELOCALIZATION_DIR"
ros2 run agt_global_relocalization_native build_relocalization_assets \
  --map "$SOURCE_PACKAGE/map.pcd" --output "$RELOCALIZATION_DIR"
ros2 run agt_global_relocalization_native build_relocalization_candidates \
  --map-dir "$SOURCE_PACKAGE" --output "$RELOCALIZATION_DIR"
ros2 run agt_mapping_artifacts export_map_package \
  --source-dir "$SOURCE_PACKAGE" --site "$SITE" --version "$VERSION" \
  --map-root "$EXPERIMENT_ROOT" --relocalization-assets-dir "$RELOCALIZATION_DIR"
