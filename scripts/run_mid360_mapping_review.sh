#!/usr/bin/env bash
# Replay a MID360 bag, export the verified PCD, convert it to PGM, then open 2D review.
set -eo pipefail
if [[ $# -lt 2 ]]; then
  echo "Usage: $0 BAG OUTPUT [run_mid360_mapping options]" >&2
  exit 2
fi
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="$1"
OUTPUT="$2"
shift 2
dry_run=false
for argument in "$@"; do
  if [[ "$argument" == "--dry-run" ]]; then
    dry_run=true
  fi
done
"$ROOT/scripts/run_mid360_mapping.sh" "$BAG" "$OUTPUT" "$@"
if [[ "$dry_run" == true ]]; then
  exit 0
fi
exec "$ROOT/scripts/review_mapping_output.sh" "$OUTPUT"
