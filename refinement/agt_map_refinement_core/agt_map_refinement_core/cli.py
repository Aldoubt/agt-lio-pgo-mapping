from __future__ import annotations

import argparse

from .pipeline import refine_map_package


def main(args=None):
    parser = argparse.ArgumentParser(description='Create a derived refined map package.')
    parser.add_argument('--map-package', required=True)
    parser.add_argument('--refinement', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--resolution', type=float, default=0.05)
    options = parser.parse_args(args)
    result = refine_map_package(options.map_package, options.refinement, options.output, options.resolution)
    print(result)