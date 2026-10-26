"""Immutable, integrity-checked offline relocalization evidence bundles."""
from __future__ import annotations

import hashlib
import json
import math
import numbers
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
from typing import Any

import yaml

from .frontend_package import require_output_outside, sha256


EVIDENCE_TYPE = 'agt.relocalization_evidence/v1'
EVIDENCE_STATUS = {'KNOWN', 'UNKNOWN', 'NO_DATA', 'CENSORED'}
EMPIRICAL_RESULTS = {
    'CORRECT', 'FALSE_ACCEPT', 'REJECTED', 'TIMEOUT', 'NOT_RUN', 'REFERENCE_UNKNOWN',
}


def _canonical_digest(value: Any) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(raw).hexdigest()


def _digest(value: Any, field: str, *, nullable: bool = False) -> None:
    if nullable and value is None:
        return
    if not isinstance(value, str) or re.fullmatch(r'[0-9a-f]{64}', value) is None:
        raise ValueError(f'{field} must be a lowercase SHA-256 digest')


def _required(mapping: dict[str, Any], keys: tuple[str, ...], label: str) -> None:
    missing = [key for key in keys if key not in mapping]
    if missing:
        raise ValueError(f'{label} missing required fields: {missing}')


def _safe_bundle_path(value: Any, label: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f'{label} must be a relative POSIX bundle path')
    path = PurePosixPath(value)
    if (path.is_absolute() or '..' in path.parts or not path.parts or path.as_posix() != value or
            '\\' in value or any(character.isspace() for character in value)):
        raise ValueError(f'{label} must be a safe relative POSIX bundle path')
    return value


def _bundle_file_path(root: Path, value: Any, label: str) -> Path:
    rel_text = _safe_bundle_path(value, label)
    path = root
    for part in PurePosixPath(rel_text).parts:
        path = path / part
        if path.is_symlink():
            raise ValueError(f'{label} must not traverse a symbolic link: {rel_text}')
    return path


def _analysis_artifact_paths(analysis_artifacts: dict[str, Any]) -> dict[str, str]:
    allowed = {'query_cloud', 'raw_trace', 'summary', 'assets_manifest', 'job', 'display_only'}
    unknown = set(analysis_artifacts) - allowed
    if unknown:
        raise ValueError(f'unsupported analysis artifact fields: {sorted(unknown)}')
    paths: dict[str, str] = {}
    for key in ('query_cloud', 'raw_trace', 'summary', 'assets_manifest', 'job'):
        if key in analysis_artifacts:
            paths[key] = _safe_bundle_path(analysis_artifacts[key], f'analysis_artifacts.{key}')
    display_only = analysis_artifacts.get('display_only', {})
    if not isinstance(display_only, dict):
        raise ValueError('analysis_artifacts.display_only must be a mapping')
    for name, value in display_only.items():
        if not isinstance(name, str) or not name:
            raise ValueError('analysis_artifacts.display_only keys must be nonempty strings')
        paths[f'display_only.{name}'] = _safe_bundle_path(
            value, f'analysis_artifacts.display_only.{name}')
    return paths


def _validate_finite_tree(value: Any, path: str = '$') -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, numbers.Real):
        if not math.isfinite(float(value)):
            raise ValueError(f'{path} contains a nonfinite number')
        return
    if isinstance(value, dict):
        for key, child in value.items():
            _validate_finite_tree(child, f'{path}.{key}')
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _validate_finite_tree(child, f'{path}[{index}]')


def validate_evidence_record(record: dict[str, Any]) -> None:
    if not isinstance(record, dict) or record.get('schema_version') != 1 or record.get('asset_type') != EVIDENCE_TYPE:
        raise ValueError('unsupported relocalization evidence schema')
    _required(record, ('evidence_id', 'source', 'block_set', 'query', 'algorithm', 'reference',
                       'evidence', 'analysis_artifacts'), 'record')
    _validate_finite_tree(record)
    if not isinstance(record.get('evidence_id'), str) or re.fullmatch(r'[A-Za-z0-9_-]{1,100}', record['evidence_id']) is None:
        raise ValueError('invalid evidence_id')
    source = record.get('source')
    block_set = record.get('block_set')
    query = record.get('query')
    algorithm = record.get('algorithm')
    reference = record.get('reference')
    if not all(isinstance(v, dict) for v in (source, block_set, query, algorithm, reference)):
        raise ValueError('source, block_set, query, algorithm and reference mappings are required')
    analysis_artifacts = record.get('analysis_artifacts')
    if not isinstance(analysis_artifacts, dict):
        raise ValueError('analysis_artifacts must be a mapping')
    analysis_paths = _analysis_artifact_paths(analysis_artifacts)
    if 'query_cloud' not in analysis_paths:
        raise ValueError('analysis_artifacts.query_cloud is required')
    _required(source, ('identity', 'manifest_sha256', 'checksums_sha256', 'pose_revision'), 'source')
    _required(block_set, ('block_set_id', 'revision', 'parent_source_digest', 'manifest_sha256',
                          'index_sha256', 'checksums_sha256', 'builder_config_digest',
                          'block_keyframe_count', 'query_excluded_block_ids',
                          'query_excluded_keyframes'), 'block_set')
    _required(query, ('query_id', 'keyframe', 'timestamp', 'window_keyframes',
                      'query_accumulation_frames', 'window_sha256', 'query_cloud_sha256',
                      'row_id', 'along_row_s_m'), 'query')
    _required(algorithm, ('name', 'candidate_top_k', 'config_digest', 'binary_sha256'), 'algorithm')
    _required(reference, ('type', 'same_session', 'independent_ground_truth', 'absolute_ground_truth'), 'reference')
    if not isinstance(source.get('identity'), str) or not source['identity']:
        raise ValueError('source.identity must be a nonempty string')
    if not isinstance(block_set.get('block_set_id'), str) or not block_set['block_set_id']:
        raise ValueError('block_set.block_set_id must be a nonempty string')
    if (isinstance(block_set.get('revision'), bool) or not isinstance(block_set.get('revision'), int) or
            block_set['revision'] < 1):
        raise ValueError('block_set.revision must be a positive integer')
    if (not isinstance(block_set.get('query_excluded_block_ids'), list) or
            any(not isinstance(value, str) or not value for value in block_set['query_excluded_block_ids']) or
            not isinstance(block_set.get('query_excluded_keyframes'), list) or
            any(not isinstance(value, int) or isinstance(value, bool) or value < 0
                for value in block_set['query_excluded_keyframes'])):
        raise ValueError('block_set query exclusion lists have invalid types')
    if not isinstance(query.get('query_id'), str) or not query['query_id']:
        raise ValueError('query.query_id must be a nonempty string')
    if isinstance(query.get('timestamp'), bool) or not isinstance(query.get('timestamp'), numbers.Real):
        raise ValueError('query.timestamp must be a finite number')
    if not isinstance(query.get('row_id'), str):
        raise ValueError('query.row_id must be a string')
    if not isinstance(algorithm.get('name'), str) or not algorithm['name']:
        raise ValueError('algorithm.name must be a nonempty string')
    if query.get('along_row_s_m') is not None and (
            isinstance(query['along_row_s_m'], bool) or
            not isinstance(query['along_row_s_m'], numbers.Real) or query['along_row_s_m'] < 0):
        raise ValueError('query.along_row_s_m must be null or a nonnegative finite number')
    for key in ('type',):
        if not isinstance(reference.get(key), str) or not reference[key]:
            raise ValueError(f'reference.{key} must be a nonempty string')
    for key in ('same_session', 'independent_ground_truth'):
        if not isinstance(reference.get(key), bool):
            raise ValueError(f'reference.{key} must be a boolean')
    for key in ('manifest_sha256', 'checksums_sha256', 'pose_revision'):
        _digest(source.get(key), f'source.{key}')
    for key in ('manifest_sha256', 'index_sha256', 'checksums_sha256', 'builder_config_digest'):
        _digest(block_set.get(key), f'block_set.{key}')
    for key in ('window_sha256', 'query_cloud_sha256'):
        _digest(query.get(key), f'query.{key}')
    _digest(algorithm.get('config_digest'), 'algorithm.config_digest')
    if source.get('manifest_sha256') != block_set.get('parent_source_digest'):
        raise ValueError('block set is bound to a different source manifest')
    _digest(block_set.get('row_annotation_sha256'), 'block_set.row_annotation_sha256', nullable=True)
    _digest(query.get('row_annotation_sha256'), 'query.row_annotation_sha256', nullable=True)
    if (block_set.get('row_annotation_sha256') is not None and
            query.get('row_annotation_sha256') != block_set.get('row_annotation_sha256')):
        raise ValueError('query and candidate block labels use different row annotation revisions')
    frames = query.get('window_keyframes')
    if (not isinstance(query.get('keyframe'), int) or isinstance(query.get('keyframe'), bool) or
            not isinstance(frames, list) or not frames or
            any(not isinstance(i, int) or isinstance(i, bool) or i < 0 for i in frames) or
            query['keyframe'] not in frames):
        raise ValueError('invalid query frame/window identity')
    if query.get('query_accumulation_frames') not in (1, 3, 5) or len(frames) != query['query_accumulation_frames']:
        raise ValueError('query window does not match query_accumulation_frames')
    expected_window = list(range(query['keyframe'] - len(frames) // 2,
                                 query['keyframe'] + len(frames) // 2 + 1))
    if frames != expected_window:
        raise ValueError('query window keyframes must be the contiguous symmetric window around query.keyframe')
    if block_set.get('block_keyframe_count') is None or not isinstance(block_set.get('block_keyframe_count'), int):
        raise ValueError('block_keyframe_count must be recorded independently')
    if algorithm.get('candidate_top_k') is None or not isinstance(algorithm.get('candidate_top_k'), int):
        raise ValueError('candidate_top_k must be recorded independently')
    if (isinstance(block_set['block_keyframe_count'], bool) or block_set['block_keyframe_count'] <= 0 or
            isinstance(algorithm['candidate_top_k'], bool) or algorithm['candidate_top_k'] <= 0):
        raise ValueError('block_keyframe_count and candidate_top_k must be positive integers')
    binaries = algorithm.get('binary_sha256')
    if not isinstance(binaries, dict) or not binaries:
        raise ValueError('algorithm binary hashes are required')
    for name, digest in binaries.items():
        _digest(digest, f'algorithm.binary_sha256.{name}')
    config = algorithm.get('config')
    if not isinstance(config, dict) or _canonical_digest(config) != algorithm.get('config_digest'):
        raise ValueError('algorithm config digest does not match canonical config')
    if (config.get('candidate_top_k') != algorithm['candidate_top_k'] or
            config.get('query_accumulation_frames') != query['query_accumulation_frames'] or
            config.get('block_keyframe_count') != block_set['block_keyframe_count']):
        raise ValueError('algorithm config reuses or disagrees with an independent query/block parameter')
    if reference.get('absolute_ground_truth') is not False:
        raise ValueError('this MVP evidence contract requires explicit absolute_ground_truth: false')
    categories = record.get('evidence', {})
    for category in ('geometry_observability', 'candidate_ambiguity', 'local_convergence_basin'):
        item = categories.get(category) if isinstance(categories, dict) else None
        if not isinstance(item, dict) or item.get('status') not in EVIDENCE_STATUS:
            raise ValueError(f'{category} needs an explicit KNOWN/UNKNOWN/NO_DATA/CENSORED status')
        if 'metrics' in item and not isinstance(item['metrics'], dict):
            raise ValueError(f'{category}.metrics must be a mapping')
        if item.get('status') == 'KNOWN' and not item.get('metrics'):
            raise ValueError(f'{category} with KNOWN status needs at least one measured metric')
    empirical = categories.get('empirical_global_result') if isinstance(categories, dict) else None
    if not isinstance(empirical, dict) or empirical.get('classification') not in EMPIRICAL_RESULTS:
        raise ValueError('empirical_global_result has unsupported classification')
    classification = empirical['classification']
    if classification in {'CORRECT', 'FALSE_ACCEPT'}:
        if empirical.get('converged') is not True:
            raise ValueError(f'{classification} requires a converged candidate result')
        error = empirical.get('reference_error')
        if not isinstance(error, dict) or not any(
                isinstance(error.get(key), numbers.Real) and math.isfinite(float(error[key]))
                for key in ('xy_m', 'translation_3d_m')):
            raise ValueError(f'{classification} requires a finite reference-relative translation error')
        if reference.get('independent_ground_truth') is not True and empirical.get('reference_relative_only') is not True:
            raise ValueError(f'{classification} requires explicit reference-relative semantics without independent ground truth')
    if classification in {'REJECTED', 'TIMEOUT', 'NOT_RUN'} and empirical.get('converged') is True:
        raise ValueError(f'{classification} cannot be marked converged')


def publish_evidence_bundle(output_dir: str | Path, record: dict[str, Any],
                            files: dict[str, str | Path], *, source_package: str | Path,
                            block_dir: str | Path, topology_path: str | Path | None = None) -> dict[str, Any]:
    """Publish a new evidence revision atomically; never overwrite existing evidence."""
    validate_evidence_record(record)
    source_root = Path(source_package).expanduser().resolve(strict=True)
    block_root = Path(block_dir).expanduser().resolve(strict=True)
    raw_output = Path(output_dir).expanduser().resolve(strict=False)
    if raw_output.exists():
        raise FileExistsError(f'refusing to overwrite evidence revision: {raw_output}')
    output = require_output_outside(output_dir, [source_root, block_root], label='evidence output')
    from .frontend_package import verify_frontend_map_package
    source_check = verify_frontend_map_package(source_root)
    if source_check.get('status') != 'PASS':
        raise ValueError(f'cannot publish evidence against an invalid source package: {source_check}')
    expected_source = record['source']
    for filename, field in (('manifest.yaml', 'manifest_sha256'),
                            ('checksums.sha256', 'checksums_sha256'),
                            ('poses_timed.txt', 'pose_revision')):
        if sha256(source_root / filename) != expected_source[field]:
            raise ValueError(f'evidence source {field} does not match the protected source package')
    from .keyframe_blocks import verify_keyframe_blocks
    block_check = verify_keyframe_blocks(block_root, source_package=source_root)
    expected_block = record['block_set']
    for field in ('block_set_id', 'manifest_sha256', 'index_sha256', 'checksums_sha256'):
        if block_check.get(field) != expected_block.get(field):
            raise ValueError(f'evidence block set {field} does not match the protected block asset')
    annotation_digest = record['query'].get('row_annotation_sha256')
    if annotation_digest is not None:
        if topology_path is None or sha256(Path(topology_path).expanduser().resolve(strict=True)) != annotation_digest:
            raise ValueError('evidence row annotation does not match a supplied topology revision')
    output.parent.mkdir(parents=True, exist_ok=True)
    temp = Path(tempfile.mkdtemp(prefix=f'.{output.name}.tmp-', dir=output.parent))
    committed = False
    try:
        for rel_text, source_text in files.items():
            rel = PurePosixPath(rel_text)
            if rel.is_absolute() or '..' in rel.parts or not rel.parts or rel.as_posix() != rel_text:
                raise ValueError(f'unsafe evidence file path: {rel_text}')
            raw_source = Path(source_text).expanduser()
            if raw_source.is_symlink():
                raise ValueError(f'evidence file must not be a symbolic link: {raw_source}')
            source = raw_source.resolve(strict=True)
            if not source.is_file():
                raise ValueError(f'evidence file must be a regular file: {source}')
            destination = temp.joinpath(*rel.parts)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
        _write_manifest(temp, record)
        verify_evidence_bundle(temp, source_package=source_root, block_dir=block_root,
                               topology_path=topology_path)
        os.rename(temp, output)
        committed = True
        return verify_evidence_bundle(output, source_package=source_root, block_dir=block_root,
                                      topology_path=topology_path)
    finally:
        if not committed and temp.exists():
            shutil.rmtree(temp)


def _write_manifest(root: Path, record: dict[str, Any]) -> None:
    # The manifest does not contain its own digest. The external checksum index
    # commits to its exact bytes and covers every other published file.
    (root / 'manifest.yaml').write_text(yaml.safe_dump(record, sort_keys=False, allow_unicode=True),
                                       encoding='utf-8')
    lines = []
    for path in sorted(root.rglob('*')):
        if path.is_file() and path.name != 'checksums.sha256':
            lines.append(f'{sha256(path)}  {path.relative_to(root).as_posix()}')
    (root / 'checksums.sha256').write_text('\n'.join(lines) + '\n', encoding='ascii')


def verify_evidence_bundle(evidence_dir: str | Path, *, source_package: str | Path | None = None,
                           block_dir: str | Path | None = None,
                           topology_path: str | Path | None = None) -> dict[str, Any]:
    requested_root = Path(evidence_dir).expanduser()
    if requested_root.is_symlink():
        raise ValueError(f'unsafe evidence directory: {requested_root}')
    root = requested_root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f'unsafe evidence directory: {root}')
    checksum_path = root / 'checksums.sha256'
    if not checksum_path.is_file() or checksum_path.is_symlink():
        raise ValueError('evidence checksum index missing')
    entries: dict[str, str] = {}
    for line in checksum_path.read_text(encoding='ascii').splitlines():
        found = re.fullmatch(r'([0-9a-f]{64})  (.+)', line)
        if not found:
            raise ValueError('malformed evidence checksum index')
        digest, rel_text = found.groups()
        rel = PurePosixPath(rel_text)
        if rel.is_absolute() or '..' in rel.parts or rel.as_posix() != rel_text or rel_text in entries:
            raise ValueError(f'unsafe/duplicate evidence path: {rel_text}')
        path = _bundle_file_path(root, rel_text, 'evidence checksum path')
        if not path.is_file() or sha256(path) != digest:
            raise ValueError(f'evidence checksum mismatch: {rel_text}')
        entries[rel_text] = digest
    actual = {path.relative_to(root).as_posix() for path in root.rglob('*')
              if path.is_file() and path != checksum_path}
    if actual != set(entries):
        raise ValueError(f'evidence checksum coverage mismatch: unlisted={sorted(actual-set(entries))}')
    manifest_path = root / 'manifest.yaml'
    if 'manifest.yaml' not in entries:
        raise ValueError('evidence manifest is not covered by checksum index')
    record = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    validate_evidence_record(record)
    analysis_paths = _analysis_artifact_paths(record['analysis_artifacts'])
    for label, rel_text in analysis_paths.items():
        path = _bundle_file_path(root, rel_text, f'analysis artifact {label}')
        if rel_text not in entries or not path.is_file():
            raise ValueError(f'declared analysis artifact {label} is not included in the evidence checksum index')
    query_cloud_rel = analysis_paths['query_cloud']
    query_cloud = _bundle_file_path(root, query_cloud_rel, 'analysis_artifacts.query_cloud')
    if sha256(query_cloud) != record['query']['query_cloud_sha256']:
        raise ValueError('query cloud digest does not match its declared analysis artifact')
    stale_reasons = []
    unchecked_reasons = []
    if source_package is not None:
        from .frontend_package import verify_frontend_map_package
        source = Path(source_package).expanduser().resolve(strict=True)
        try:
            verified = verify_frontend_map_package(source)
            if verified.get('status') != 'PASS':
                stale_reasons.append('current source package fails authoritative validation')
        except (OSError, ValueError) as exc:
            stale_reasons.append(f'current source package is invalid: {exc}')
        for rel, key, description in (
                ('manifest.yaml', 'manifest_sha256', 'source manifest revision changed'),
                ('poses_timed.txt', 'pose_revision', 'source pose revision changed'),
                ('checksums.sha256', 'checksums_sha256', 'source checksum index changed')):
            try:
                if sha256(source / rel) != record['source'][key]:
                    stale_reasons.append(description)
            except OSError:
                if description not in stale_reasons:
                    stale_reasons.append(description)
    else:
        unchecked_reasons.append('current source package was not supplied')
    if block_dir is not None:
        from .keyframe_blocks import verify_keyframe_blocks
        try:
            current_block = verify_keyframe_blocks(block_dir, source_package=source_package)
            expected = record['block_set']
            for field, key in (('block_set_id', 'block_set_id'), ('manifest_sha256', 'manifest_sha256'),
                               ('index_sha256', 'index_sha256'), ('checksums_sha256', 'checksums_sha256')):
                if current_block.get(field) != expected.get(key):
                    stale_reasons.append(f'block set {field} changed')
        except (OSError, ValueError) as exc:
            stale_reasons.append(f'block set invalid: {exc}')
    else:
        unchecked_reasons.append('current block set was not supplied')
    expected_topology_digest = record.get('query', {}).get('row_annotation_sha256')
    if expected_topology_digest is not None:
        if topology_path is None:
            unchecked_reasons.append('current row annotation was not supplied')
        else:
            try:
                if sha256(Path(topology_path).expanduser().resolve(strict=True)) != expected_topology_digest:
                    stale_reasons.append('row annotation revision changed')
            except OSError:
                stale_reasons.append('row annotation revision is unavailable')
    if stale_reasons:
        state = 'STALE'
    elif unchecked_reasons:
        state = 'UNCHECKED'
    else:
        state = 'CURRENT'
    return {'status': 'PASS', 'evidence_id': record['evidence_id'],
            'manifest_sha256': sha256(manifest_path),
            'checksums_sha256': sha256(checksum_path),
            'file_count': len(entries), 'revision_state': state,
            'freshness_verified': state == 'CURRENT',
            'stale_reasons': stale_reasons, 'unchecked_reasons': unchecked_reasons}
