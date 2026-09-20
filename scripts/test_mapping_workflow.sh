#!/usr/bin/env bash
# Pure-Python/operator contract tests. These do NOT claim ROS/bag integration.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$ROOT/bringup/agt_mapping_bringup:$ROOT/artifacts/agt_mapping_artifacts${PYTHONPATH:+:$PYTHONPATH}"
"${PYTHON:-python3}" -m unittest discover -s "$ROOT/bringup/agt_mapping_bringup/test" -p 'test_*.py' -v
"${PYTHON:-python3}" -m unittest discover -s "$ROOT/artifacts/agt_mapping_artifacts/test" -p 'test_validation.py' -v
