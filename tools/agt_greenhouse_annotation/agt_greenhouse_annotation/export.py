"""Regenerate read-only export artifacts from a valid topology."""
import argparse
from pathlib import Path
from .topology import load_topology, load_map_package, save_topology
from .validator import validate_topology


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topology', required=True)
    parser.add_argument('--other-map-package')
    args = parser.parse_args(argv)
    topology = load_topology(args.topology)
    package = load_map_package(topology['source']['map_package'])
    result = validate_topology(topology, package)
    if not result['valid']:
        raise ValueError('; '.join(result['errors']))
    other = load_map_package(args.other_map_package) if args.other_map_package else None
    save_topology(topology, Path(args.topology), package, other)
    print(f'Exported labels and provenance for {args.topology}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
