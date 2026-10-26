"""Freeze an explicitly reviewed manual annotation for read-only benchmarks."""
import argparse
import copy
import json
from pathlib import Path

from .topology import load_map_package, load_topology, now_iso, save_topology
from .validator import validate_topology


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topology', required=True)
    parser.add_argument('--output', required=True, help='Frozen YAML path; use a dedicated annotation version directory')
    parser.add_argument('--other-map-package')
    parser.add_argument('--confirm-manual-review', action='store_true', required=True,
                        help='You have manually reviewed physical IDs, vertices and scenes')
    args = parser.parse_args(argv)
    topology = copy.deepcopy(load_topology(args.topology))
    package = load_map_package(topology['source']['map_package'])
    other = load_map_package(args.other_map_package) if args.other_map_package else None
    topology['annotation'].update(status='frozen', manual_review_confirmed=True, frozen_at=now_iso())
    result = validate_topology(topology, package, verify_map_files=True)
    if not result['valid']:
        print(json.dumps(result, indent=2))
        return 2
    save_topology(topology, args.output, package, other)
    result = validate_topology(topology, package, args.output,
                              Path(args.output).parent/'keyframe_topology_labels.csv')
    Path(args.output).parent.joinpath('annotation_validation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(frozen_annotation=args.output, validation=result), indent=2))
    return 0 if result['valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
