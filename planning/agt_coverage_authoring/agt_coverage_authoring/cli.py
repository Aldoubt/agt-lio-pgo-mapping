"""Headless analysis, planning, review and existing Route asset export."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import signal
import tempfile

import numpy as np
from PIL import Image

from .assets import _rename_noreplace, export_draft, publish_ready
from .grid import analyze_traversability, write_planning_map
from .io import assert_current, file_digest, load_request, read_document
from .models import CoverageError, PathPoint, PlanResult


def _write_json(path, value):
    payload = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True,
                          allow_nan=False) + '\n').encode('utf-8')
    if len(payload) > 32 * 1024 * 1024:
        raise CoverageError('OUTPUT_TOO_LARGE', 'Split the task: a review document exceeds 32 MiB')
    Path(path).write_bytes(payload)


def _progress(event):
    print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)


def _load_plan(path):
    path = Path(path)
    if path.stat().st_size > 32 * 1024 * 1024:
        raise CoverageError('PLAN_TOO_LARGE', str(path))
    data = read_document(path, max_bytes=32 * 1024 * 1024, max_nodes=4_000_000)
    if len(data.get('points', [])) > 250_000 or type(data.get('complete')) is not bool:
        raise CoverageError('INVALID_PLAN', 'Invalid point count or completion flag')
    return PlanResult([PathPoint(**point) for point in data['points']], data['complete'],
                      data.get('report', {}), data.get('missing', []), data.get('provenance', {}))


def _publication(path):
    path = Path(path).resolve()
    config = read_document(path)
    for key in ('site_schema', 'profile_snapshot'):
        if config.get(key):
            config[key] = str((path.parent / config[key]).resolve())
    return config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('analyze', 'plan', 'validate', 'export'))
    parser.add_argument('--request', required=True, help='Reviewed job YAML or JSON')
    parser.add_argument('--output', required=True,
                        help='New result directory, or READY companion store; revisions are never overwritten')
    parser.add_argument('--plan', help='Existing plan.json for validate/export')
    parser.add_argument('--format', choices=('draft', 'ready'), default='draft')
    parser.add_argument('--publication', help='Explicit READY publication settings YAML')
    parser.add_argument('--confirm-review', action='store_true',
                        help='Confirm review of this exact plan when publishing READY')
    args = parser.parse_args(argv)
    canceled = [False]
    previous = {}
    stage = None
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: canceled.__setitem__(0, True))
        context = load_request(args.request)
        context.cancelled = lambda: canceled[0]
        destination = Path(args.output).expanduser().absolute()
        ready_export = args.operation == 'export' and args.format == 'ready'
        if destination.exists() and not (ready_export and destination.is_dir()):
            raise CoverageError('OUTPUT_EXISTS', str(destination))
        if destination.resolve() != destination:
            raise CoverageError('UNSAFE_OUTPUT_PATH', 'Output paths may not contain symlinks')
        if context.site_root:
            site = Path(context.site_root).resolve()
            if destination == site or site in destination.parents or destination in site.parents:
                raise CoverageError('OUTPUT_OVERLAPS_SITE', 'Output must be disjoint from the immutable Site')
        if args.operation in ('validate', 'export'):
            if not args.plan:
                raise CoverageError('PLAN_REQUIRED', '--plan is required')
            result = _load_plan(args.plan)
            if result.provenance.get('input_files') != context.input_files:
                raise CoverageError('STALE_PLAN', 'Plan does not match the current input snapshots')
            if args.operation == 'export':
                if args.format == 'draft':
                    refs = export_draft(context, result, destination)
                else:
                    if not args.publication:
                        raise CoverageError('PUBLICATION_REQUIRED', '--publication is required for READY')
                    publication_path = str(Path(args.publication).resolve())
                    if context.input_files.get(publication_path) != file_digest(publication_path):
                        raise CoverageError('UNREVIEWED_PUBLICATION', 'Bind publication YAML in the request before planning')
                    settings = _publication(args.publication)
                    # Review is an export-time acknowledgement, not a flag that
                    # can be pre-approved in a configuration before a plan exists.
                    settings['review_confirmed'] = args.confirm_review
                    refs = publish_ready(context, result, destination, **settings)
                _progress({'stage': 'EXPORTED', 'assets': refs})
                return 0
        destination.parent.mkdir(parents=True, exist_ok=True)
        stage = Path(tempfile.mkdtemp(prefix='.coverage-', dir=destination.parent))
        layers, analysis = analyze_traversability(context)
        np.savez_compressed(stage / 'clearance.npz', **layers,
                            resolution=context.grid.resolution, origin=context.grid.origin)
        # Image orientation matches the source raster; float distances remain in NPZ.
        scaled = np.clip(layers['distance_m'] / 2.0, 0.0, 1.0)
        heatmap = np.zeros((*scaled.shape, 4), dtype=np.uint8)
        heatmap[..., 0] = (255 * (1.0-scaled)).astype(np.uint8)
        heatmap[..., 1] = (255 * scaled).astype(np.uint8)
        heatmap[..., 2] = 40
        heatmap[..., 3] = np.where(layers['known'], 150, 0).astype(np.uint8)
        Image.fromarray(np.flipud(heatmap)).save(stage / 'clearance.png')
        _write_json(stage / 'analysis.json', analysis)
        if args.operation == 'analyze':
            payload = {'points': [], 'complete': False, 'missing': [],
                       'report': analysis, 'provenance': {'input_files': context.input_files}}
        elif args.operation == 'plan':
            from .planner import plan_coverage
            context.backend_options = dict(context.backend_options)
            if context.backend == 'ros':
                context.backend_options['planning_map'] = str(write_planning_map(context, stage / 'derived'))
            result = plan_coverage(context, progress=_progress)
            result.provenance['input_files'] = dict(context.input_files)
            payload = asdict(result)
            _write_json(stage / 'plan.json', payload)
        else:
            from .validation import validate_path
            from .assets import _assert_task_coverage
            validation = validate_path(result.points, context)
            payload = asdict(result)
            coverage_issue = None
            try:
                if not result.points:
                    raise CoverageError('INCOMPLETE_COVERAGE', 'Empty path')
                _assert_task_coverage(context, result)
            except CoverageError as exc:
                coverage_issue = {'code': exc.code, 'detail': str(exc)}
            payload['complete'] = bool(result.complete and validation['valid'] and coverage_issue is None)
            if coverage_issue:
                payload['missing'].append(coverage_issue)
            payload['report'] = dict(payload['report'], validation=validation,
                                     status='COMPLETE' if payload['complete'] else 'PARTIAL',
                                     eligible_for_execution=False)
            blockers = []
            if not payload['complete']:
                blockers.append('INCOMPLETE_TASK')
            if any(point.direction == 'R' for point in result.points):
                blockers.append('REVERSE_UNSUPPORTED')
            if len({point.v_ref for point in result.points}) > 1:
                blockers.append('VARIABLE_SPEED_UNSUPPORTED')
            if any(abs(offset) > 1e-12 for offset in context.robot.rear_axle_to_base):
                blockers.append('REFERENCE_POINT_UNSUPPORTED')
            if validation['dynamic_status'] != 'VALID':
                blockers.append('DYNAMIC_CONSTRAINTS_UNVERIFIED')
            payload['report']['runtime_compatibility'] = {'compatible': not blockers, 'blocked_by': blockers}
            _write_json(stage / 'plan.json', payload)
        _write_json(stage / 'report.json', payload)
        _write_json(stage / 'inputs.json', context.input_files)
        context.check_cancelled()
        assert_current(context.input_files)
        _rename_noreplace(stage, destination)
        stage = None
        _progress({'stage': 'FINISHED', 'output': str(destination),
                   'complete': payload['complete'], 'eligible_for_execution': False})
        return 0
    except (CoverageError, OSError, ValueError, KeyError, TypeError) as exc:
        _progress({'stage': 'FAILED', 'code': getattr(exc, 'code', 'INVALID_INPUT'),
                   'detail': str(exc), 'eligible_for_execution': False})
        return 2
    finally:
        if stage is not None:
            shutil.rmtree(stage, ignore_errors=True)
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    raise SystemExit(main())
