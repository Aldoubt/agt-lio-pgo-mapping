#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

from contract import validate_bundle


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate an AGT Research Asset Contract V1 project.")
    parser.add_argument("project", type=Path)
    parser.add_argument("--skip-source-hashes", action="store_true")
    args = parser.parse_args()
    print(json.dumps(validate_bundle(args.project, verify_source_files=not args.skip_source_hashes), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
