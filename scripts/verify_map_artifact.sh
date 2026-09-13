#!/usr/bin/env bash
# Verify a PGO mapping artifact without requiring ROS nodes to be running.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <OUTPUT_DIRECTORY_OR_map_package>" >&2
  exit 2
fi

TARGET="$(realpath "$1")"
ROOT="$TARGET"
[[ -d "$ROOT/map_package" ]] && ROOT="$ROOT/map_package"

for file in map.pcd poses.txt poses_timed.txt calibration.yaml metadata.yaml manifest.yaml checksums.sha256; do
  [[ -s "$ROOT/$file" ]] || { echo "Missing or empty: $ROOT/$file" >&2; exit 1; }
done
grep -Eq '^backend: PGO$' "$ROOT/metadata.yaml"
grep -Eq '^[[:space:]]+optimized: true$' "$ROOT/metadata.yaml"
(cd "$ROOT" && sha256sum -c checksums.sha256 >/dev/null)

echo "Artifact verified: $ROOT"
