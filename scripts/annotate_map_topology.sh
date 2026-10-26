#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || ${1:-} == --help ]]; then
  cat <<'USAGE'
Usage: annotate_map_topology.sh MAP_PACKAGE ANNOTATION_DIR [GUI options]

Open any validated AGT map_package for manual row/headland/scene annotation.
Output: ANNOTATION_DIR/greenhouse_topology.yaml and geometry/label/hash sidecars.
Example:
  ./scripts/annotate_map_topology.sh /data/new/map_package /data/new/annotation
  ./scripts/annotate_map_topology.sh /data/new/map_package /data/new/annotation \
    --other-map-package /data/other_backend/map_package

Requires Python3, PyQt5, matplotlib, Open3D, NumPy, PyYAML and Shapely.
Draw physical IDs manually, save, validate, then freeze after review.
USAGE
  [[ ${1:-} == --help ]] && exit 0
  exit 2
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
map_package_path=$1
annotation_dir=$2
shift 2
export PYTHONPATH="$repo_dir/tools/agt_greenhouse_annotation${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m agt_greenhouse_annotation.gui \
  --map-package "$map_package_path" \
  --output "$annotation_dir/greenhouse_topology.yaml" "$@"
