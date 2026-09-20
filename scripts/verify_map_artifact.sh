#!/usr/bin/env bash
# Verify the complete artifact without ROS nodes. Requires Python 3 + PyYAML.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$ROOT/artifacts/agt_mapping_artifacts${PYTHONPATH:+:$PYTHONPATH}"
exec "${PYTHON:-python3}" -m agt_mapping_artifacts.validation "$@"
