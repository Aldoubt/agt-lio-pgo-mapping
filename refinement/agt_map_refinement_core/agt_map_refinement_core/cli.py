from __future__ import annotations

import argparse

from .pipeline import refine_map_package


def main(args=None):
    parser = argparse.ArgumentParser(description='Create a derived refined map package.')
    parser.add_argument('--map-package', required=True)
    parser.add_argument('--refinement', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='preview nav_map resolution; not the navigation-grade converter')
    parser.add_argument('--pcd-format', choices=('auto', 'ascii', 'binary'), default='auto',
                        help='refined map.pcd encoding; auto keeps the source encoding')
    parser.add_argument('--no-preview-nav-map', action='store_true',
                        help='skip the quick nav_map.pgm/yaml preview')
    options = parser.parse_args(args)
    result = refine_map_package(options.map_package, options.refinement, options.output,
                                options.resolution, pcd_format=options.pcd_format,
                                write_preview_nav_map=not options.no_preview_nav_map)
    print(result)