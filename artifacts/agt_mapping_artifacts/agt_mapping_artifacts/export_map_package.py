"""CLI for publishing an optimized PGO Map Package."""

from __future__ import annotations

import argparse
import os

from .map_package_exporter import MapPackageExporter


def main() -> int:
    parser = argparse.ArgumentParser(description='Export an optimized PGO artifact as a Map Package')
    parser.add_argument('--source-dir', required=True)
    parser.add_argument('--site', required=True)
    parser.add_argument('--version', required=True)
    parser.add_argument('--map-root', default=os.environ.get('AGT_MAP_ROOT', '~/ros2_ws/maps'))
    parser.add_argument('--relocalization-assets-dir')
    parser.add_argument('--calibration')
    args = parser.parse_args()
    destination = MapPackageExporter().export(
        args.source_dir, args.map_root, args.site, args.version,
        args.relocalization_assets_dir, args.calibration)
    print(destination)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
