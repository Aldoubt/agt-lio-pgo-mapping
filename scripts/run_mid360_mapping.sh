#!/usr/bin/env bash
# Keep help/preflight usable before ROS is installed or sourced.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export AGT_MAPPING_REPOSITORY="$ROOT"
export PYTHONPATH="$ROOT/bringup/agt_mapping_bringup:$ROOT/artifacts/agt_mapping_artifacts${PYTHONPATH:+:$PYTHONPATH}"
exec "${PYTHON:-python3}" -m agt_mapping_bringup.cli "$@"
