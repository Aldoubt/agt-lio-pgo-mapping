#!/usr/bin/env bash
# Live single-MID360 mapping. Thin wrapper: run_mid360_mapping.sh --live [OUTPUT] [options]
# Records <OUTPUT>/raw_bag, maps online, finishes on:
#   ros2 service call /mapping/session/finish std_srvs/srv/Trigger "{}"   (same ROS_DOMAIN_ID)
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT/scripts/run_mid360_mapping.sh" --live "$@"
