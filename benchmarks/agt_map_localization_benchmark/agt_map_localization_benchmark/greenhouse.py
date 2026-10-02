"""Greenhouse-specific offline relocalization benchmark.

This module deliberately contains no registration implementation. It reuses the
installed production native executables to answer two questions from a verified
PGO map package:

1. For row-middle / row-end / headland / row-entry queries, what initial
   x/y/yaw perturbations remain inside the local GICP convergence basin?
2. Does the GLOBAL pipeline's 3D-BBS coarse pose place the same query inside
   that basin, and does the subsequent GICP converge to the PGO reference?

All generated maps, queries and assets live in a new experiment directory.
The source map package is checksum-verified and never modified.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
import traceback
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .backends import (GLOBAL_SETTINGS, LOCAL_SETTINGS, NativePrograms,
                       global_command, invoke, local_command)
from .metrics import classify_failure, measure_pose, pose_record
from .pcd import read_pcd, sha256_file, verify_checksum_index, write_pcd, xyz

SCENE_TYPES = ('row_middle', 'row_end', 'headland', 'row_entry')
BASIN_PROFILES = {
    'smoke': {
        'dx_m': (-1.0, 0.0, 1.0),
        'dy_m': (-1.0, 0.0, 1.0),
        'dyaw_deg': (-20.0, 0.0, 20.0),
        'frames': (1,),
    },
    'standard': {
        'dx_m': tuple(np.arange(-2.0, 2.0001, 0.5).tolist()),
        'dy_m': tuple(np.arange(-1.5, 1.5001, 0.5).tolist()),
        'dyaw_deg': tuple(np.arange(-40.0, 40.0001, 10.0).tolist()),
        'frames': (1, 3, 5),
    },
}


@dataclass(frozen=True)
class Pose:
    index: int
    patch: str
    stamp: float
    t: np.ndarray
    quat_xyzw: np.ndarray

    @property
    def rotation(self) -> np.ndarray:
        return Rotation.from_quat(self.quat_xyzw).as_matrix()


@dataclass(frozen=True)
class Scene:
    scene_id: str
    scene_type: str
    keyframe: int
    row_id: str | None
    note: str | None


@dataclass(frozen=True)
class RowRange:
    row_id: str
    start: int
    end: int

    def contains(self, index: int) -> bool:
        return self.start <= index <= self.end


@dataclass(frozen=True)
class SceneConfig:
    scenes: tuple[Scene, ...]
    rows: tuple[RowRange, ...]

    def row_for_index(self, index: int) -> str | None:
        found = [r.row_id for r in self.rows if r.contains(index)]
        if len(found) > 1:
            raise ValueError(f'keyframe {index} belongs to overlapping row ranges: {found}')
        return found[0] if found else None


def _safe_id(value: str, field: str) -> str:
    if (not value or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in value)):
        raise ValueError(f'{field} must contain only ASCII letters, digits, _ or -: {value!r}')
    return value


def load_scene_config(path: Path, pose_count: int | None = None) -> SceneConfig:
    """Load human-labelled evaluation locations; no automatic scene inference."""
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(raw, dict) or raw.get('schema_version') != 1:
        raise ValueError('scene YAML requires schema_version: 1')
    scene_rows = raw.get('scenes')
    if not isinstance(scene_rows, list) or not scene_rows:
        raise ValueError('scene YAML requires a non-empty scenes list')
    scenes = []
    seen_ids = set()
    for item in scene_rows:
        if not isinstance(item, dict):
            raise ValueError('each scene must be a mapping')
        scene_id = _safe_id(str(item.get('id', '')), 'scene id')
        scene_type = str(item.get('type', ''))
        if scene_type not in SCENE_TYPES:
            raise ValueError(f'{scene_id}: type must be one of {SCENE_TYPES}')
        keyframe = int(item['keyframe'])
        if pose_count is not None and not (2 <= keyframe < pose_count - 2):
            raise ValueError(f'{scene_id}: keyframe {keyframe} needs a full five-frame window')
        if scene_id in seen_ids:
            raise ValueError(f'duplicate scene id: {scene_id}')
        seen_ids.add(scene_id)
        row_id = item.get('row_id')
        scenes.append(Scene(scene_id, scene_type, keyframe,
                            None if row_id is None else str(row_id),
                            None if item.get('note') is None else str(item['note'])))
    ranges = []
    for item in raw.get('rows', []) or []:
        if not isinstance(item, dict):
            raise ValueError('each row range must be a mapping')
        start, end = int(item['start_keyframe']), int(item['end_keyframe'])
        if start < 0 or end < start or (pose_count is not None and end >= pose_count):
            raise ValueError(f'invalid row range: {item}')
        ranges.append(RowRange(str(item['row_id']), start, end))
    config = SceneConfig(tuple(scenes), tuple(ranges))
    if pose_count is not None:
        for i in range(pose_count):
            config.row_for_index(i)
    for scene in config.scenes:
        inferred = config.row_for_index(scene.keyframe)
        if scene.row_id is not None and inferred is not None and str(scene.row_id) != str(inferred):
            raise ValueError(f'{scene.scene_id}: row_id={scene.row_id} conflicts with row range {inferred}')
    return config


def parse_frames(value: str) -> tuple[int, ...]:
    frames = tuple(int(v.strip()) for v in value.split(',') if v.strip())
    if not frames or any(v not in (1, 3, 5) for v in frames) or len(set(frames)) != len(frames):
        raise ValueError('--frames must be a unique comma-separated subset of 1,3,5')
    return frames


def grid_values(minimum: float, maximum: float, step: float) -> tuple[float, ...]:
    if not all(math.isfinite(v) for v in (minimum, maximum, step)) or step <= 0 or maximum < minimum:
        raise ValueError('invalid perturbation grid bounds')
    count = int(math.floor((maximum - minimum) / step + 1e-9)) + 1
    values = tuple(float(minimum + i * step) for i in range(count))
    if values[-1] < maximum - 1e-8:
        values = values + (float(maximum),)
    if len(values) > 101:
        raise ValueError('a single perturbation axis may not exceed 101 samples')
    return values


def perturb_pose(reference: Pose, dx: float, dy: float, dyaw_deg: float) -> tuple[np.ndarray, np.ndarray]:
    t = reference.t + np.array([dx, dy, 0.0], dtype='f8')
    yaw = Rotation.from_euler('z', dyaw_deg, degrees=True)
    quat = (yaw * Rotation.from_quat(reference.quat_xyzw)).as_quat()
    return t, quat


class MapPackage:
    """Read-only, checksum-verified PGO map package without confidence sidecars."""

    def __init__(self, root: Path):
        self.root = Path(root).resolve(strict=True)
        self.checksums = verify_checksum_index(
            self.root, ('map.pcd', 'poses_timed.txt', 'manifest.yaml'))
        patches = self.root / 'patches'
        if not patches.is_dir() or patches.is_symlink():
            raise ValueError(f'missing/unsafe patches directory: {patches}')
        self.poses = self._read_poses()
        self.patch_to_index = {p.patch: p.index for p in self.poses}
        for pose in self.poses:
            path = patches / pose.patch
            if not path.is_file() or path.is_symlink():
                raise ValueError(f'missing/unsafe query patch: {path}')

    def _read_poses(self) -> tuple[Pose, ...]:
        poses = []
        names = set()
        for i, line in enumerate((self.root / 'poses_timed.txt').read_text(encoding='ascii').splitlines()):
            tokens = line.split()
            if len(tokens) != 9:
                raise ValueError(f'bad poses_timed record {i}')
            patch = tokens[0]
            if Path(patch).name != patch or not patch.endswith('.pcd') or patch in names:
                raise ValueError(f'unsafe/repeated patch name: {patch}')
            names.add(patch)
            values = np.asarray([float(v) for v in tokens[1:]], dtype='f8')
            if not np.isfinite(values).all():
                raise ValueError(f'nonfinite pose record: {i}')
            qw, qx, qy, qz = values[4:8]
            quat = np.array([qx, qy, qz, qw], dtype='f8')
            norm = float(np.linalg.norm(quat))
            if norm < 1e-8:
                raise ValueError(f'zero quaternion: {i}')
            poses.append(Pose(i, patch, float(values[0]), values[1:4], quat / norm))
        if len(poses) < 7:
            raise ValueError('map package needs at least seven keyframes')
        return tuple(poses)

    def patch_body(self, index: int) -> np.ndarray:
        data = read_pcd(self.root / 'patches' / self.poses[index].patch, ('x', 'y', 'z'))
        points = xyz(data).astype('f8')
        return points[np.isfinite(points).all(axis=1)]

    def patch_map(self, index: int) -> np.ndarray:
        pose = self.poses[index]
        return self.patch_body(index) @ pose.rotation.T + pose.t

    def query_body(self, center: int, frames: int) -> np.ndarray:
        if frames not in (1, 3, 5) or center - frames // 2 < 0 or center + frames // 2 >= len(self.poses):
            raise ValueError('invalid query window')
        ref = self.poses[center]
        chunks = []
        for index in range(center - frames // 2, center + frames // 2 + 1):
            pose = self.poses[index]
            points = self.patch_body(index)
            aligned = (points @ pose.rotation.T + pose.t - ref.t) @ ref.rotation
            chunks.append(aligned.astype('<f4'))
        query = np.concatenate(chunks)
        if not len(query):
            raise ValueError(f'empty query at keyframe {center}')
        return query

    def local_map_indices(self, center: int, excluded: set[int], radius_xy: float,
                          max_patches: int) -> tuple[int, ...]:
        ref = self.poses[center]
        choices = []
        for pose in self.poses:
            if pose.index in excluded:
                continue
            distance = float(np.linalg.norm(pose.t[:2] - ref.t[:2]))
            if distance <= radius_xy:
                choices.append((distance, pose.index))
        choices.sort()
        indices = tuple(i for _, i in choices[:max_patches])
        if len(indices) < 5:
            raise ValueError(f'not enough local target patches around keyframe {center}')
        return indices

    def write_map(self, indices: Iterable[int], destination: Path) -> dict:
        ids = tuple(sorted(set(int(i) for i in indices)))
        if not ids:
            raise ValueError('cannot write an empty evaluation map')
        chunks = [self.patch_map(i).astype('<f4') for i in ids]
        points = np.concatenate(chunks)
        write_pcd(destination, points)
        return {'path': str(destination), 'sha256': sha256_file(destination),
                'points': int(len(points)), 'patch_indices': list(ids)}


def heldout_indices(config: SceneConfig, frames: Iterable[int]) -> set[int]:
    result = set()
    for scene in config.scenes:
        for count in frames:
            result.update(range(scene.keyframe - count // 2, scene.keyframe + count // 2 + 1))
    return result


def _native_pose_error(reference: Pose, native: dict) -> dict:
    ref = pose_record(reference.t, reference.quat_xyzw)
    return measure_pose(ref, native)


def _coarse_pose_error(reference: Pose, coarse: dict | None) -> dict:
    if coarse is None:
        return {'translation_3d_error_m': None, 'xy_error_m': None, 'z_error_m': None,
                'yaw_error_deg': None, 'so3_error_deg': None, 'error_xyz_map_m': None,
                'strict_success': False, 'nominal_success': False, 'loose_success': False}
    return measure_pose(pose_record(reference.t, reference.quat_xyzw),
                        {'pose': coarse, 'backend_success': True})


def _flatten_basin_row(scene: Scene, frames: int, dx: float, dy: float, yaw: float,
                       native: dict, measured: dict, failure: str | None) -> dict:
    return {
        'scene_id': scene.scene_id, 'scene_type': scene.scene_type, 'row_id': scene.row_id,
        'keyframe': scene.keyframe, 'frames': frames,
        'dx_m': dx, 'dy_m': dy, 'dyaw_deg': yaw,
        'backend_success': bool(native.get('backend_success')), 'failure_code': failure,
        'strict_success': bool(measured['strict_success']),
        'nominal_success': bool(measured['nominal_success']),
        'loose_success': bool(measured['loose_success']),
        'final_translation_3d_error_m': measured['translation_3d_error_m'],
        'final_xy_error_m': measured['xy_error_m'], 'final_z_error_m': measured['z_error_m'],
        'final_yaw_error_deg': measured['yaw_error_deg'], 'final_so3_error_deg': measured['so3_error_deg'],
        'fitness': native.get('fitness_native'), 'overlap': native.get('overlap_native'),
        'wall_ms': native.get('wall_ms_external'),
        'hessian_eigenvalues': native.get('native_hessian_eigenvalues_mixed_units'),
    }


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    fields = list(rows[0].keys())
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            cooked = dict(row)
            for key, value in list(cooked.items()):
                if isinstance(value, (dict, list, tuple)):
                    cooked[key] = json.dumps(value, allow_nan=False, sort_keys=True)
            writer.writerow(cooked)


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n', encoding='utf-8')


def run_basin(dataset: MapPackage, config: SceneConfig, native: NativePrograms,
              run: Path, frames: tuple[int, ...], dx_values: tuple[float, ...],
              dy_values: tuple[float, ...], yaw_values: tuple[float, ...],
              local_radius_m: float, local_max_patches: int) -> tuple[list[dict], dict[str, dict]]:
    stage = run / 'basin'
    stage.mkdir()
    query_root = stage / 'queries'
    target_root = stage / 'targets'
    rows = []
    target_meta: dict[str, dict] = {}
    exclude = heldout_indices(config, frames)
    for scene in config.scenes:
        ref = dataset.poses[scene.keyframe]
        local_indices = dataset.local_map_indices(scene.keyframe, exclude, local_radius_m,
                                                  local_max_patches)
        target = target_root / f'{scene.scene_id}.pcd'
        target_meta[scene.scene_id] = dataset.write_map(local_indices, target)
        for frame_count in frames:
            query = query_root / f'{scene.scene_id}_f{frame_count}.pcd'
            write_pcd(query, dataset.query_body(scene.keyframe, frame_count))
            for dx in dx_values:
                for dy in dy_values:
                    for yaw in yaw_values:
                        t, quat = perturb_pose(ref, dx, dy, yaw)
                        result = invoke(local_command(native.paths['map_gicp_tracker'], target, query, t, quat),
                                        timeout=35)
                        measured = _native_pose_error(ref, result)
                        record_for_failure = {'algorithm': 'LOCAL', 'reference_pose': pose_record(ref.t, ref.quat_xyzw),
                                              'native': result, **measured}
                        failure = classify_failure(record_for_failure)
                        rows.append(_flatten_basin_row(scene, frame_count, dx, dy, yaw,
                                                       result, measured, failure))
            print(f'BASIN {scene.scene_id} f={frame_count}: {len(rows)} accumulated trials', flush=True)
    _write_csv(stage / 'basin.csv', rows)
    _write_json(stage / 'targets.json', target_meta)
    return rows, target_meta


def _copy_descriptor_source(dataset: MapPackage, indices: tuple[int, ...], destination: Path,
                            min_patch_points: int) -> int:
    patches = destination / 'patches'
    patches.mkdir(parents=True)
    pose_rows = []
    for index in indices:
        pose = dataset.poses[index]
        body = dataset.patch_body(index)
        if len(body) < min_patch_points:
            continue
        write_pcd(patches / pose.patch, body.astype('<f4'))
        qx, qy, qz, qw = pose.quat_xyzw
        pose_rows.append(f'{pose.patch} {pose.t[0]:.17g} {pose.t[1]:.17g} {pose.t[2]:.17g} '
                         f'{qw:.17g} {qx:.17g} {qy:.17g} {qz:.17g}\n')
    if not pose_rows:
        raise ValueError('no descriptor patch satisfies minimum point count')
    (destination / 'poses.txt').write_text(''.join(pose_rows), encoding='ascii')
    return len(pose_rows)


def build_global_assets(dataset: MapPackage, config: SceneConfig, native: NativePrograms,
                        run: Path, frames: tuple[int, ...]) -> tuple[Path, dict]:
    stage = run / 'global'
    stage.mkdir(exist_ok=True)
    exclude = heldout_indices(config, frames)
    map_indices = tuple(i for i in range(len(dataset.poses)) if i not in exclude)
    target = stage / 'global_target_map.pcd'
    target_meta = dataset.write_map(map_indices, target)
    source = stage / 'descriptor_source'
    descriptor_count = _copy_descriptor_source(
        dataset, map_indices, source, GLOBAL_SETTINGS['descriptor_min_patch_points'])
    assets = stage / 'assets'
    coarse_cmd = [str(native.paths['build_relocalization_assets']), '--map', str(target),
                  '--output', str(assets), '--map-leaf', str(GLOBAL_SETTINGS['assets_map_leaf_m']),
                  '--bbs-min-level-res', str(GLOBAL_SETTINGS['bbs_min_level_res_m']),
                  '--bbs-max-level', str(GLOBAL_SETTINGS['bbs_max_level'])]
    desc_cmd = [str(native.paths['build_relocalization_candidates']), '--map-dir', str(source),
                '--output', str(assets), '--min-patch-points',
                str(GLOBAL_SETTINGS['descriptor_min_patch_points'])]
    coarse = subprocess.run(coarse_cmd, text=True, capture_output=True, timeout=180, check=False)
    desc = subprocess.run(desc_cmd, text=True, capture_output=True, timeout=180, check=False)
    if coarse.returncode != 0 or desc.returncode != 0:
        raise RuntimeError('global asset build failed: '
                           f'coarse={coarse.returncode} desc={desc.returncode}; '
                           f'{coarse.stderr[-300:]} {desc.stderr[-300:]}')
    meta = {'target': target_meta, 'descriptor_patch_count': descriptor_count,
            'heldout_query_indices': sorted(exclude),
            'same_session_pgo_reference': True,
            'asset_files_sha256': {str(p.relative_to(assets)): sha256_file(p)
                                   for p in sorted(assets.rglob('*')) if p.is_file()}}
    _write_json(stage / 'assets_manifest.json', meta)
    return target, meta


def _candidate_row(dataset: MapPackage, config: SceneConfig, candidate_patch: object) -> tuple[int | None, str | None]:
    if candidate_patch is None:
        return None, None
    name = Path(str(candidate_patch)).name
    index = dataset.patch_to_index.get(name)
    return index, config.row_for_index(index) if index is not None else None


def run_global(dataset: MapPackage, config: SceneConfig, native: NativePrograms,
               run: Path, frames: tuple[int, ...], local_targets: dict[str, dict] | None) -> list[dict]:
    if not native.global_ready:
        raise RuntimeError('GLOBAL native executables are not installed')
    global_target, _ = build_global_assets(dataset, config, native, run, frames)
    assets = run / 'global' / 'assets'
    queries = run / 'global' / 'queries'
    queries.mkdir()
    rows = []
    for scene in config.scenes:
        ref = dataset.poses[scene.keyframe]
        for frame_count in frames:
            query = queries / f'{scene.scene_id}_f{frame_count}.pcd'
            write_pcd(query, dataset.query_body(scene.keyframe, frame_count))
            global_result = invoke(global_command(native.paths['candidate_bbs_gicp_localizer'],
                                                  global_target, query, assets), timeout=30)
            final_measured = _native_pose_error(ref, global_result)
            global_failure = classify_failure({'algorithm': 'GLOBAL',
                                               'reference_pose': pose_record(ref.t, ref.quat_xyzw),
                                               'native': global_result, **final_measured})
            coarse = global_result.get('coarse_pose')
            coarse_measured = _coarse_pose_error(ref, coarse)
            seed_result = None
            seed_measured = None
            seed_failure = None
            if coarse is not None and local_targets and scene.scene_id in local_targets:
                target = Path(local_targets[scene.scene_id]['path'])
                t = np.array([coarse[k] for k in ('x', 'y', 'z')], dtype='f8')
                quat = np.array([coarse[k] for k in ('qx', 'qy', 'qz', 'qw')], dtype='f8')
                seed_result = invoke(local_command(native.paths['map_gicp_tracker'], target, query, t, quat),
                                     timeout=35)
                seed_measured = _native_pose_error(ref, seed_result)
                seed_failure = classify_failure({'algorithm': 'LOCAL',
                                                 'reference_pose': pose_record(ref.t, ref.quat_xyzw),
                                                 'native': seed_result, **seed_measured})
            candidate_index, candidate_row = _candidate_row(
                dataset, config, global_result.get('candidate_patch_native'))
            wrong_row = (scene.row_id is not None and candidate_row is not None
                         and str(scene.row_id) != str(candidate_row))
            rows.append({
                'scene_id': scene.scene_id, 'scene_type': scene.scene_type, 'row_id': scene.row_id,
                'keyframe': scene.keyframe, 'frames': frame_count,
                'global_backend_success': bool(global_result.get('backend_success')),
                'global_failure_code': global_failure,
                'final_strict_success': bool(final_measured['strict_success']),
                'final_nominal_success': bool(final_measured['nominal_success']),
                'final_loose_success': bool(final_measured['loose_success']),
                'final_xy_error_m': final_measured['xy_error_m'],
                'final_translation_3d_error_m': final_measured['translation_3d_error_m'],
                'final_yaw_error_deg': final_measured['yaw_error_deg'],
                'coarse_available': coarse is not None,
                'coarse_xy_error_m': coarse_measured['xy_error_m'],
                'coarse_translation_3d_error_m': coarse_measured['translation_3d_error_m'],
                'coarse_yaw_error_deg': coarse_measured['yaw_error_deg'],
                'bbs_score': global_result.get('bbs_score_native'),
                'bbs_elapsed_ms': global_result.get('bbs_elapsed_ms_native'),
                'descriptor_ring_distance': global_result.get('descriptor_ring_distance_native'),
                'descriptor_similarity': global_result.get('descriptor_similarity_native'),
                'candidate_patch': global_result.get('candidate_patch_native'),
                'candidate_keyframe': candidate_index, 'candidate_row_id': candidate_row,
                'wrong_row_candidate': wrong_row if candidate_row is not None else None,
                'coarse_seed_gicp_ran': seed_result is not None,
                'coarse_seed_gicp_backend_success': None if seed_result is None else bool(seed_result.get('backend_success')),
                'coarse_seed_gicp_failure_code': seed_failure,
                'coarse_seed_gicp_nominal_success': None if seed_measured is None else bool(seed_measured['nominal_success']),
                'coarse_seed_gicp_xy_error_m': None if seed_measured is None else seed_measured['xy_error_m'],
                'coarse_seed_gicp_yaw_error_deg': None if seed_measured is None else seed_measured['yaw_error_deg'],
                'global_wall_ms': global_result.get('wall_ms_external'),
            })
            print(f'GLOBAL {scene.scene_id} f={frame_count}: final={final_measured["nominal_success"]} '
                  f'coarse_seed={None if seed_measured is None else seed_measured["nominal_success"]}', flush=True)
    _write_csv(run / 'global' / 'global.csv', rows)
    return rows


def _fraction(rows: list[dict], field: str) -> dict:
    values = [r.get(field) for r in rows if r.get(field) is not None]
    hits = sum(bool(v) for v in values)
    return {'hits': hits, 'total': len(values), 'fraction': hits / len(values) if values else None}


def summarize(basin_rows: list[dict], global_rows: list[dict]) -> dict:
    types = sorted(set([r['scene_type'] for r in basin_rows] + [r['scene_type'] for r in global_rows]))
    summary = {'by_scene_type': {}, 'overall': {}}
    for scene_type in types:
        b = [r for r in basin_rows if r['scene_type'] == scene_type]
        g = [r for r in global_rows if r['scene_type'] == scene_type]
        summary['by_scene_type'][scene_type] = {
            'basin_nominal': _fraction(b, 'nominal_success'),
            'basin_strict': _fraction(b, 'strict_success'),
            'global_final_nominal': _fraction(g, 'final_nominal_success'),
            'bbs_coarse_seed_gicp_nominal': _fraction(g, 'coarse_seed_gicp_nominal_success'),
            'wrong_row_candidate': _fraction(g, 'wrong_row_candidate'),
        }
    summary['overall'] = {
        'basin_nominal': _fraction(basin_rows, 'nominal_success'),
        'global_final_nominal': _fraction(global_rows, 'final_nominal_success'),
        'bbs_coarse_seed_gicp_nominal': _fraction(global_rows, 'coarse_seed_gicp_nominal_success'),
        'wrong_row_candidate': _fraction(global_rows, 'wrong_row_candidate'),
    }
    return summary


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='Offline greenhouse GICP-basin and BBS->GICP benchmark; never publishes TF or controls a robot.')
    p.add_argument('--map-package', type=Path, required=True)
    p.add_argument('--scenes', type=Path, required=True, help='human-labelled greenhouse scene YAML')
    p.add_argument('--run-id', required=True)
    p.add_argument('--output-root', type=Path,
                   default=Path.home() / 'ros2_ws/experiments/greenhouse_relocalization_benchmark')
    p.add_argument('--ros-install', type=Path, default=Path.home() / 'ros2_ws/install')
    p.add_argument('--profile', choices=tuple(BASIN_PROFILES), default='smoke')
    p.add_argument('--mode', choices=('basin', 'global', 'all'), default='all')
    p.add_argument('--frames', help='override profile frames, e.g. 1,3,5')
    p.add_argument('--local-radius-m', type=float, default=25.0)
    p.add_argument('--local-max-patches', type=int, default=128)
    for axis, default in (('dx', None), ('dy', None), ('yaw', None)):
        p.add_argument(f'--{axis}-min', type=float, default=default)
        p.add_argument(f'--{axis}-max', type=float, default=default)
        p.add_argument(f'--{axis}-step', type=float, default=default)
    return p


def _grid_from_args(args: argparse.Namespace) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], tuple[int, ...]]:
    profile = BASIN_PROFILES[args.profile]
    values = []
    for axis, key in (('dx', 'dx_m'), ('dy', 'dy_m'), ('yaw', 'dyaw_deg')):
        triple = (getattr(args, f'{axis}_min'), getattr(args, f'{axis}_max'), getattr(args, f'{axis}_step'))
        if any(v is not None for v in triple):
            if not all(v is not None for v in triple):
                raise ValueError(f'--{axis}-min/--{axis}-max/--{axis}-step must be supplied together')
            values.append(grid_values(*triple))
        else:
            values.append(tuple(float(v) for v in profile[key]))
    frames = parse_frames(args.frames) if args.frames else tuple(profile['frames'])
    return values[0], values[1], values[2], frames


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    run: Path | None = None
    manifest: dict | None = None
    try:
        run_id = _safe_id(args.run_id, 'run-id')
        dx_values, dy_values, yaw_values, frames = _grid_from_args(args)
        if args.local_radius_m <= 0 or args.local_max_patches < 5:
            raise ValueError('local map bounds must be positive and include at least five patches')
        dataset = MapPackage(args.map_package)
        config = load_scene_config(args.scenes, len(dataset.poses))
        native = NativePrograms.discover(args.ros_install)
        if not native.local_ready:
            raise ValueError('installed map_gicp_tracker is required')
        if args.mode in ('global', 'all') and not native.global_ready:
            raise ValueError('GLOBAL mode requires candidate_bbs_gicp_localizer and both asset builders')
        root = args.output_root.expanduser().resolve()
        source = dataset.root
        if root == source or source in root.parents:
            raise ValueError('output-root may not be inside the immutable source map package')
        root.mkdir(parents=True, exist_ok=True)
        run = root / run_id
        if run.exists():
            raise ValueError(f'run already exists; refusing overwrite: {run}')
        run.mkdir()
        manifest = {
            'schema_version': 1, 'status': 'IN_PROGRESS', 'offline_only': True,
            'created_utc': datetime.now(timezone.utc).isoformat(), 'run_id': run_id,
            'mode': args.mode, 'profile': args.profile, 'map_package': str(dataset.root),
            'map_checksums_sha256': sha256_file(dataset.root / 'checksums.sha256'),
            'scene_yaml': str(args.scenes.resolve()), 'scene_yaml_sha256': sha256_file(args.scenes),
            'native_executable_sha256': native.sha256,
            'reference': 'same-session optimized_PGO_pose; NOT absolute ground truth',
            'data_leakage_control': 'all declared query windows excluded from generated target/descriptor maps',
            'remaining_bias': 'same-session PGO poses are used for reference and query accumulation',
            'parameters': {
                'frames': list(frames), 'dx_m': list(dx_values), 'dy_m': list(dy_values),
                'dyaw_deg': list(yaw_values), 'local_radius_m': args.local_radius_m,
                'local_max_patches': args.local_max_patches,
                'local_native': LOCAL_SETTINGS, 'global_native': GLOBAL_SETTINGS,
            },
        }
        _write_json(run / 'manifest.json', manifest)
        basin_rows: list[dict] = []
        global_rows: list[dict] = []
        targets: dict[str, dict] | None = None
        if args.mode in ('basin', 'all'):
            basin_rows, targets = run_basin(dataset, config, native, run, frames,
                                             dx_values, dy_values, yaw_values,
                                             args.local_radius_m, args.local_max_patches)
        elif args.mode == 'global':
            target_stage = run / 'coarse_seed_targets'
            target_stage.mkdir()
            targets = {}
            exclude = heldout_indices(config, frames)
            for scene in config.scenes:
                ids = dataset.local_map_indices(scene.keyframe, exclude, args.local_radius_m,
                                                args.local_max_patches)
                targets[scene.scene_id] = dataset.write_map(ids, target_stage / f'{scene.scene_id}.pcd')
        if args.mode in ('global', 'all'):
            global_rows = run_global(dataset, config, native, run, frames, targets)
        summary = summarize(basin_rows, global_rows)
        _write_json(run / 'summary.json', summary)
        manifest['status'] = 'COMPLETED'
        manifest['counts'] = {'basin_trials': len(basin_rows), 'global_trials': len(global_rows)}
        manifest['summary'] = summary
        _write_json(run / 'manifest.json', manifest)
        print(f'GREENHOUSE BENCHMARK COMPLETED: {run}', flush=True)
        return 0
    except BaseException as exc:
        if run is not None and run.is_dir() and manifest is not None:
            manifest['status'] = 'INCOMPLETE'
            manifest['failure'] = f'{type(exc).__name__}: {exc}'
            try:
                _write_json(run / 'manifest.json', manifest)
            except Exception:
                pass
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
