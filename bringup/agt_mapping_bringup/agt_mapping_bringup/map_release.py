"""Mapping-owned export workflow; shared map-manager CLI owns publication format.

No navigation nodes are started. Preparation never touches maps/. Publication
requires explicit human acceptance, and affects future auto navigation starts.
"""
import argparse
import json
import hashlib
import yaml
from pathlib import Path
import re
import subprocess
import sys

from agt_mapping_artifacts.validation import verify_artifact
from .review import ReviewError, default_projection_config


def _path(value):
    return Path(value).expanduser().resolve()


def _identifier(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', value):
        raise ValueError('map id/version must be a safe single path component')
    return value


def _robot_profile(value):
    profile = _identifier(value)
    if profile not in ('bunker_v1', 'yhs_v1'):
        raise ValueError(f'unsupported map-release robot profile: {profile}')
    return profile


def prepare_plan(args):
    source = verify_artifact(args.source)
    map_id, version = _identifier(args.map_id), _identifier(args.version)
    robot = _robot_profile(args.robot)
    if robot == 'yhs_v1' and not args.config:
        raise ValueError('YHS map preparation requires explicit --config with measured YHS footprint/traversability; refusing Bunker defaults')
    workspace = _path(args.workspace)
    experiments = workspace / 'experiments'
    # The currently installed shared publisher constrains candidates to this
    # host workspace. Reject alternate roots rather than fail after conversion.
    if experiments != _path('~/ros2_ws/experiments'):
        raise ValueError('shared map publisher currently requires ~/ros2_ws/experiments')
    work = experiments / 'mapping' / 'exports' / map_id / version
    candidate_root = experiments / 'mapping' / 'candidates'
    candidate = candidate_root / map_id / version
    if work.exists() or candidate.exists():
        raise ValueError('export/version already exists; use a new version')
    config = _path(args.config) if args.config else default_projection_config()
    if not config.is_file():
        raise ValueError(f'projection configuration missing: {config}')
    if robot == 'yhs_v1':
        # A second path to the Bunker YAML is not a YHS projection. Require an
        # explicit, human-reviewed profile marker in the YHS-only config.
        projection = yaml.safe_load(config.read_text(encoding='utf-8'))
        if (not isinstance(projection, dict) or projection.get('robot_profile') != 'yhs_v1' or
                projection.get('field_verified') is not True or not projection.get('verified_by')):
            raise ValueError('YHS projection requires robot_profile: yhs_v1, '
                             'field_verified: true, verified_by: <reviewer>; '
                             'these are an attestation, not automatic measurement')
    nav, assets = work / 'navigation', work / 'relocalization'
    commands = [
        ['ros2', 'run', 'agt_pcd2grid_exporter', 'pcd2grid_exporter',
         '--package', str(source), '--config', str(config), '--output', str(nav)],
        ['ros2', 'run', 'agt_global_relocalization_native', 'build_relocalization_assets',
         '--map', str(source / 'map.pcd'), '--output', str(assets)],
        ['ros2', 'run', 'agt_global_relocalization_native', 'build_relocalization_candidates',
         '--map-dir', str(source), '--output', str(assets)],
        ['ros2', 'run', 'agt_map_manager', 'build_map_candidate',
         '--map-id', map_id, '--map-version', version,
         '--source-pcd', str(source / 'map.pcd'), '--navigation-dir', str(nav),
         '--relocalization-assets-dir', str(assets), '--candidate-root', str(candidate_root)],
    ]
    return {'work': str(work), 'candidate': str(candidate),
            'robot': robot, 'source': str(source), 'config': str(config), 'commands': commands}


def publish_plan(args):
    robot = _robot_profile(args.robot)
    if not args.confirm_reviewed:
        raise ValueError('publication requires --confirm-reviewed after map and field acceptance')
    candidate, workspace = _path(args.candidate), _path(args.workspace)
    if not candidate.is_relative_to(workspace / 'experiments'):
        raise ValueError('candidate must be inside workspace experiments/')
    if not (candidate / 'candidate_state.yaml').is_file():
        raise ValueError('not a built candidate: missing candidate_state.yaml')
    if robot == 'yhs_v1':
        # Publishing a Bunker projection under a YHS compatibility label is
        # unsafe. Require the recorded YHS prepare plan, not just a CLI flag.
        work_plan = (workspace / 'experiments' / 'mapping' / 'exports' /
                     candidate.parent.name / candidate.name / 'release_plan.json')
        try:
            prepared = json.loads(work_plan.read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            raise ValueError(f'YHS candidate missing recorded YHS prepare plan: {work_plan}') from exc
        if prepared.get('robot') != 'yhs_v1' or prepared.get('candidate') != str(candidate):
            raise ValueError('YHS candidate provenance mismatch; refusing Bunker map relabel')
    command = ['ros2', 'run', 'agt_map_manager', 'promote_map_candidate',
               '--candidate', str(candidate), '--map-root', str(workspace / 'maps'),
               '--robot', robot]
    # Deliberately no --activate: legacy active pointer stays unchanged.
    return {'candidate': str(candidate), 'robot': robot, 'map_root': str(workspace / 'maps'),
            'commands': [command]}


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    subs = root.add_subparsers(dest='operation', required=True)
    prepare = subs.add_parser('prepare', help='export a new, unpublished candidate')
    prepare.add_argument('--source', required=True, help='verified optimized PGO map_package')
    prepare.add_argument('--map-id', required=True)
    prepare.add_argument('--version', required=True)
    prepare.add_argument('--config', help='projection configuration; REQUIRED for YHS (measured footprint/terrain)')
    publish = subs.add_parser('publish', help='publish a human-accepted candidate as latest_validated')
    publish.add_argument('--candidate', required=True)
    publish.add_argument('--confirm-reviewed', action='store_true',
                         help='attest completed visual, localization and field acceptance')
    for command in (prepare, publish):
        command.add_argument('--robot', choices=('bunker_v1', 'yhs_v1'), default='bunker_v1',
                             help='map compatibility; YHS never inherits Bunker projection defaults')
        command.add_argument('--workspace', default='~/ros2_ws')
        command.add_argument('--dry-run', action='store_true')
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        plan = prepare_plan(args) if args.operation == 'prepare' else publish_plan(args)
        print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
        if args.dry_run:
            return 0
        if args.operation == 'prepare':
            work = Path(plan['work'])
            work.mkdir(parents=True, exist_ok=False)
            (work / 'relocalization').mkdir()
            (work / 'release_plan.json').write_text(
                json.dumps(plan, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        for index, command in enumerate(plan['commands']):
            subprocess.run(command, check=True)
            if args.operation == 'prepare' and index == 0:
                # The shared builder copies this supported provenance sidecar.
                nav = Path(plan['work']) / 'navigation'
                provenance = {
                    'schema_version': 1,
                    'generator': 'agt_pcd2grid_exporter',
                    'source_package': plan['source'],
                    'source_checksums_sha256': hashlib.sha256(
                        (Path(plan['source']) / 'checksums.sha256').read_bytes()).hexdigest(),
                    'projection': yaml.safe_load((nav / 'projection.yaml').read_text()),
                    'export_statistics': yaml.safe_load((nav / 'metadata.yaml').read_text()),
                }
                (nav / 'converter_metadata.yaml').write_text(
                    yaml.safe_dump(provenance, sort_keys=False), encoding='utf-8')
        if args.operation == 'prepare':
            print('Candidate BUILT, NOT field-accepted or published: ' + plan['candidate'])
        else:
            print('Published as latest_validated for this map id; legacy active pointer unchanged.')
            print('auto uses the registry default_map_id/default_map; no running navigation was reloaded.')
        return 0
    except (ValueError, ReviewError, OSError, yaml.YAMLError, subprocess.CalledProcessError) as exc:
        print(f'map release failed: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
