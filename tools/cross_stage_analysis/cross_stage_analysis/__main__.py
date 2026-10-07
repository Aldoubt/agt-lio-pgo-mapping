from __future__ import annotations

import argparse
import json
import sys

from .pipeline import run_stage


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Cross-stage greenhouse map audit and reviewed ROI workflow")
    parser.add_argument("--config", required=True, help="experiment YAML")
    parser.add_argument("--stage", choices=("all", "alignment", "roi", "occupancy"), default="all")
    args = parser.parse_args(argv)
    try:
        result = run_stage(args.config, args.stage)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "BLOCKED", "error": str(exc)}, indent=2, ensure_ascii=False), file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
