"""Generate a 2D map from a completed run and open the lightweight editor.

This module intentionally composes only packages shipped by
agt_mapping_framework.  It does not import or invoke navigation-v3 tools.
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys

import yaml


class ReviewError(RuntimeError):
    """A completed, verified mapping artifact is not available for review."""


def inspect_output(output):
    output = Path(output).expanduser().resolve()
    session_path = output / 'session.json'
    if not session_path.is_file():
        raise ReviewError(f'missing mapping session: {session_path}')
    try:
        session = json.loads(session_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ReviewError(f'cannot read mapping session {session_path}: {error}') from error
    if session.get('status') != 'completed':
        raise ReviewError(
            f"mapping is not complete (status={session.get('status', 'unknown')!r})"
        )
    if session.get('artifact_verified') is not True:
        raise ReviewError('mapping artifact was not verified; refusing to create a review map')

    artifact_value = session.get('artifact_path')
    artifact = Path(artifact_value).expanduser() if artifact_value else output / 'map_package'
    if not artifact.is_absolute():
        artifact = output / artifact
    artifact = artifact.resolve()
    for name in ('map.pcd', 'manifest.yaml'):
        if not (artifact / name).is_file():
            raise ReviewError(f'incomplete mapping artifact: {artifact / name} is missing')
    return output, artifact


def make_plan(output, artifact, config=None):
    review_root = output / 'map_review'
    base = review_root / 'base'
    confirmed = review_root / 'confirmed'
    converter = [
        'ros2', 'run', 'agt_pcd2grid_exporter', 'pcd2grid_exporter',
        '--package', str(artifact), '--output', str(base),
    ]
    if config:
        converter.extend(['--config', str(Path(config).expanduser().resolve())])
    studio = [
        'ros2', 'run', 'agt_map_studio', 'map_viewer',
        '--review-package', str(artifact),
        '--review-map', str(base / 'map.yaml'),
        '--review-output', str(confirmed),
    ]
    return {
        'mapping_output': str(output),
        'mapping_package': str(artifact),
        'base_map': str(base),
        'confirmed_map': str(confirmed),
        'converter_command': converter,
        'studio_command': studio,
    }


def _run(command, label):
    print(f'[mapping-review] {label}:', ' '.join(command), flush=True)
    try:
        result = subprocess.run(command, check=False)
    except OSError as error:
        raise ReviewError(f'cannot start {label}: {error}') from error
    if result.returncode != 0:
        raise ReviewError(f'{label} failed with exit code {result.returncode}')


def build_parser():
    parser = argparse.ArgumentParser(
        description='Convert a verified mapping PCD to PGM and open lightweight 2D review.'
    )
    parser.add_argument('output', help='Completed mapping output directory containing session.json')
    parser.add_argument('--config', help='Optional agt_pcd2grid_exporter projection YAML')
    parser.add_argument(
        '--no-studio', action='store_true',
        help='Generate/reuse the base PGM but do not open the editor',
    )
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Validate inputs and print the commands without running them',
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        output, artifact = inspect_output(args.output)
        if args.config and not Path(args.config).expanduser().is_file():
            raise ReviewError(f'projection config does not exist: {args.config}')
        plan = make_plan(output, artifact, args.config)
        if args.dry_run:
            print(json.dumps(plan, ensure_ascii=False, indent=2))
            return 0

        base = Path(plan['base_map'])
        base_files = (base / 'map.pgm', base / 'map.yaml')
        # Conversion is cheap relative to mapping and parameters evolve. Always
        # regenerate the unconfirmed base map so a stale PGM is never reviewed.
        _run(plan['converter_command'], 'static-map filtering and PCD to PGM conversion')
        if not all(path.is_file() for path in base_files):
            raise ReviewError(f'converter did not create map.pgm and map.yaml in {base}')
        if not args.no_studio:
            status_path = Path(plan['confirmed_map']) / 'review_status.yaml'
            previous_mtime = status_path.stat().st_mtime_ns if status_path.is_file() else None
            _run(plan['studio_command'], '2D map editor')
            if not status_path.is_file():
                raise ReviewError('editor closed without confirming the 2D map')
            if previous_mtime is not None and status_path.stat().st_mtime_ns <= previous_mtime:
                raise ReviewError('editor closed without updating the previous confirmation')
            try:
                confirmation = yaml.safe_load(status_path.read_text(encoding='utf-8')) or {}
            except (OSError, yaml.YAMLError) as error:
                raise ReviewError(f'cannot read confirmation status: {error}') from error
            if confirmation.get('status') != 'confirmed':
                raise ReviewError('2D map confirmation status is not confirmed')
            print(f'[mapping-review] confirmed map: {plan["confirmed_map"]}', flush=True)
        return 0
    except ReviewError as error:
        print(f'mapping review failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
