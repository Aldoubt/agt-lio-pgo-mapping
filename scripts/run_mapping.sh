#!/usr/bin/env bash
# Generic product entry; current verified sensor is MID360.
# New sensor/frontends must pass independent adapter acceptance.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export AGT_MAPPING_REPOSITORY="$ROOT"
export PYTHONPATH="$ROOT/bringup/agt_mapping_bringup:$ROOT/artifacts/agt_mapping_artifacts${PYTHONPATH:+:$PYTHONPATH}"
exec "${PYTHON:-python3}" -m agt_mapping_bringup.cli "$@"
