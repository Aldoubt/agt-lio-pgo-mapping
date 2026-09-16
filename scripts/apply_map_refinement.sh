#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 4 || $# -gt 6 ]]; then
  echo "Usage: $0 --map_package PATH --refinement PATH --output PATH [--resolution METERS]" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
if [[ -f "$(dirname "${BASH_SOURCE[0]}")/../../install/setup.bash" ]]; then
  source "$(dirname "${BASH_SOURCE[0]}")/../../install/setup.bash"
fi
ros2 run agt_map_refinement_core apply_map_refinement "$@"