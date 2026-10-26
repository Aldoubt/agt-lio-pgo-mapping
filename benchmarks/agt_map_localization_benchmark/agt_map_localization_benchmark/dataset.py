"""Verified PGO/evidence inputs and strictly disjoint offline map/query splits."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .pcd import read_pcd, sha256_file, verify_checksum_index, xyz

KEY_DTYPE = np.dtype([('x', '<i8'), ('y', '<i8'), ('z', '<i8')])
MAX_MAP_PATCHES_PER_CENTER = 96  # resource bound declared before registration outcomes


@dataclass(frozen=True)
class Pose:
    index: int
    patch: str
    stamp: float
    t: np.ndarray  # map-frame body origin
    quat_xyzw: np.ndarray

    @property
    def rotation(self) -> np.ndarray:
        return Rotation.from_quat(self.quat_xyzw).as_matrix()


@dataclass(frozen=True)
class Split:
    tier: str
    center_indices: tuple[int, ...]
    map_indices: tuple[int, ...]
    heldout_indices: tuple[int, ...]
    all_query_indices: tuple[int, ...]
    max_map_radius_xy_m: float


def key_array(components: np.ndarray) -> np.ndarray:
    values = np.asarray(components)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError('voxel keys must be Nx3')
    keys = np.empty(len(values), dtype=KEY_DTYPE)
    for i, name in enumerate(('x', 'y', 'z')):
        keys[name] = values[:, i].astype('<i8')
    return keys


def keys_from_pcd(data: np.ndarray) -> np.ndarray:
    cols = np.column_stack((data['voxel_x'], data['voxel_y'], data['voxel_z']))
    if not np.isfinite(cols).all() or not np.array_equal(cols, cols.astype('<i8').astype(cols.dtype)):
        raise ValueError('non-integral/nonfinite sidecar voxel key')
    return key_array(cols)


def point_keys(coords: np.ndarray, voxel_size: float) -> np.ndarray:
    """V1's float32 floor division convention (not round-to-nearest)."""
    points = np.asarray(coords, dtype='<f4')
    return key_array(np.floor(points / np.float32(voxel_size)).astype('<i8'))


def lookup_sorted_keys(source_keys: np.ndarray, query_keys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    positions = np.searchsorted(source_keys, query_keys)
    found = positions < len(source_keys)
    found[found] = source_keys[positions[found]] == query_keys[found]
    return positions.clip(max=len(source_keys) - 1), found


def stable_flags(voxels: np.ndarray, stable_threshold: float) -> np.ndarray:
    modes = voxels['override_mode']
    return ((voxels['final_confidence'] >= np.float32(stable_threshold))
            & (modes != 2) & (modes != 3))


class PgoEvidence:
    """Loads evidence without mutating parent, V1, geometry, or reviewed files."""

    def __init__(self, pgo: Path, confidence: Path, geometry: Path,
                 reviewed: Path | None = None):
        self.pgo = Path(pgo).resolve(strict=True)
        self.confidence = Path(confidence).resolve(strict=True)
        self.geometry = Path(geometry).resolve(strict=True)
        self.reviewed = Path(reviewed).resolve(strict=True) if reviewed else None
        self.validate_sources()
        self.v1_meta = yaml.safe_load((self.confidence / 'confidence_metadata.yaml').read_text())
        self.geom_meta = yaml.safe_load((self.geometry / 'geometry_metadata.yaml').read_text())
        self.rev_meta = (yaml.safe_load((self.reviewed / 'confidence_metadata.yaml').read_text())
                         if self.reviewed else None)
        self._check_metadata()
        self.v1 = read_pcd(self.confidence / 'confidence_voxels.pcd',
                           ('x', 'y', 'z', 'voxel_x', 'voxel_y', 'voxel_z',
                            'final_confidence', 'override_mode', 'geometry_score'))
        self.geom = read_pcd(self.geometry / 'geometry_voxels.pcd',
                             ('x', 'y', 'z', 'voxel_x', 'voxel_y', 'voxel_z',
                              'translation_valid', 'translation_q',
                              'translation_weak_x', 'translation_weak_y', 'translation_weak_z',
                              'normal_valid', 'normal_x', 'normal_y', 'normal_z',
                              'rotation_valid', 'rotation_q',
                              'rotation_weak_x', 'rotation_weak_y', 'rotation_weak_z'))
        self.rev = (read_pcd(self.reviewed / 'confidence_voxels.pcd',
                             ('x', 'y', 'z', 'voxel_x', 'voxel_y', 'voxel_z',
                              'auto_confidence', 'final_confidence', 'override_mode'))
                    if self.reviewed else None)
        self.keys = keys_from_pcd(self.v1)
        if (not np.array_equal(np.argsort(self.keys, kind='stable'), np.arange(len(self.keys)))
                or np.any(self.keys[1:] == self.keys[:-1])):
            raise ValueError('V1 voxel keys not sorted or not unique')
        if not np.array_equal(self.keys, keys_from_pcd(self.geom)):
            raise ValueError('geometry/V1 key mismatch')
        for field in ('x', 'y', 'z'):
            if not np.array_equal(self.v1[field], self.geom[field]):
                raise ValueError(f'geometry/V1 {field} centroid mismatch')
        if not np.all(self.v1['geometry_score'] == np.float32(1)):
            raise ValueError('V1 geometry_score changed from 1')
        if self.rev is not None:
            if not np.array_equal(self.keys, keys_from_pcd(self.rev)):
                raise ValueError('reviewed/V1 key mismatch')
            for name in ('x', 'y', 'z', 'auto_confidence'):
                if not np.array_equal(self.rev[name], self.v1[name]):
                    raise ValueError(f'reviewed evidence recomputed: {name}')
        self.voxel_size = float(self.v1_meta['parameters']['voxel_size'])
        self.v1_stable = stable_flags(self.v1, float(self.v1_meta['parameters']['stable_threshold']))
        self.reviewed_stable = (stable_flags(self.rev, float(self.rev_meta['parameters']['stable_threshold']))
                                if self.rev is not None else None)
        if int(self.v1_stable.sum()) != int(self.v1_meta['counts']['stable_voxels']):
            raise ValueError('V1 stable predicate/count mismatch')
        if self.rev is not None and int(self.reviewed_stable.sum()) != int(self.rev_meta['counts']['stable_voxels']):
            raise ValueError('reviewed stable predicate/count mismatch')
        self.poses = self._read_poses()
        self.map_data = read_pcd(self.pgo / 'map.pcd', ('x', 'y', 'z', 'intensity'))
        self.patch_ranges = self._patch_offsets()

    def _check_metadata(self) -> None:
        parent_manifest = sha256_file(self.pgo / 'manifest.yaml')
        parent_checksums = sha256_file(self.pgo / 'checksums.sha256')
        v1_checksums = sha256_file(self.confidence / 'checksums.sha256')
        for data, kind in ((self.v1_meta, 'spatial_confidence_v1'),
                           (self.geom_meta, 'spatial_geometry_evidence_v1'),
                           (self.rev_meta, 'spatial_confidence_v1')):
            if data is None:
                continue
            if data.get('schema_version') != 1 or data.get('artifact_type') != kind or data.get('frame_id') != 'map':
                raise ValueError(f'invalid sidecar contract: {kind}')
            source = data.get('source', {})
            if source.get('parent_manifest_sha256') != parent_manifest or source.get('parent_checksums_sha256') != parent_checksums:
                raise ValueError(f'{kind} does not belong to this PGO parent')
        if self.v1_meta['geometry']['mode'] != 'deferred' or self.geom_meta['source']['confidence_checksums_sha256'] != v1_checksums:
            raise ValueError('geometry evidence does not belong to frozen deferred V1')
        if self.rev_meta is not None:
            if (self.rev_meta.get('review', {}).get('source_checksums_sha256') != v1_checksums
                    or self.rev_meta['review'].get('automatic_evidence') != 'copied_from_verified_source_without_recomputation'):
                raise ValueError('reviewed stable source is not frozen V1')

    def validate_sources(self) -> dict[str, str]:
        required = {
            'pgo': (self.pgo, ('map.pcd', 'poses_timed.txt', 'manifest.yaml')),
            'v1': (self.confidence, ('confidence_voxels.pcd', 'stable_map.pcd', 'confidence_metadata.yaml', 'manual_overrides.yaml')),
            'geometry': (self.geometry, ('geometry_voxels.pcd', 'geometry_metadata.yaml')),
        }
        if self.reviewed:
            required['reviewed'] = (self.reviewed, ('confidence_voxels.pcd', 'stable_map.pcd', 'confidence_metadata.yaml', 'manual_overrides.yaml'))
        result = {}
        for name, (folder, files) in required.items():
            verify_checksum_index(folder, files)
            result[name + '_checksums_index'] = sha256_file(folder / 'checksums.sha256')
        result['pgo_map'] = sha256_file(self.pgo / 'map.pcd')
        result['pgo_poses_timed'] = sha256_file(self.pgo / 'poses_timed.txt')
        result['v1_confidence_pcd'] = sha256_file(self.confidence / 'confidence_voxels.pcd')
        result['geometry_pcd'] = sha256_file(self.geometry / 'geometry_voxels.pcd')
        if self.reviewed:
            result['reviewed_confidence_pcd'] = sha256_file(self.reviewed / 'confidence_voxels.pcd')
        return result

    def _read_poses(self) -> tuple[Pose, ...]:
        poses = []
        names = set()
        for i, line in enumerate((self.pgo / 'poses_timed.txt').read_text().splitlines()):
            tokens = line.split()
            if len(tokens) != 9:
                raise ValueError(f'bad poses_timed record {i}')
            name = tokens[0]
            if name in names or Path(name).name != name or not name.endswith('.pcd'):
                raise ValueError(f'unsafe/repeated patch {name}')
            names.add(name)
            values = np.array([float(v) for v in tokens[1:]], dtype='f8')
            if not np.isfinite(values).all():
                raise ValueError(f'nonfinite pose {i}')
            qw, qx, qy, qz = values[4:8]
            quat = np.array([qx, qy, qz, qw], dtype='f8')
            if np.linalg.norm(quat) < 1e-8:
                raise ValueError(f'zero quaternion {i}')
            poses.append(Pose(i, name, float(values[0]), values[1:4], quat / np.linalg.norm(quat)))
        if len(poses) < 6:
            raise ValueError('PGO has fewer than six pose records')
        return tuple(poses)

    def _patch_offsets(self) -> tuple[tuple[int, int], ...]:
        """Prove ALL parent map points follow the patch order before splitting.

        A three-point sample can miss a permuted query block and accidentally
        leak the query into a Tier1 map.  This full linear check uses only
        immutable input bytes; it never rewrites the parent map or patches.
        """
        offsets = []
        cursor = 0
        for pose in self.poses:
            patch = read_pcd(self.pgo / 'patches' / pose.patch, ('x', 'y', 'z'))
            end = cursor + len(patch)
            if end > len(self.map_data):
                raise ValueError('patch totals exceed optimized PGO map')
            if len(patch):
                reference = xyz(patch).astype('f8') @ pose.rotation.T + pose.t
                in_map = xyz(self.map_data[cursor:end]).astype('f8')
                if not np.allclose(reference, in_map, rtol=0, atol=2e-4, equal_nan=False):
                    raise ValueError(f'PGO map order does not follow ALL patch points: {pose.patch}')
            offsets.append((cursor, end))
            cursor = end
        if cursor != len(self.map_data):
            raise ValueError('PGO map count does not match all patches')
        return tuple(offsets)

    def raw_map_subset(self, map_indices: tuple[int, ...]) -> tuple[np.ndarray, np.ndarray]:
        coords = []
        intensity = []
        for i in map_indices:
            start, end = self.patch_ranges[i]
            chunk = self.map_data[start:end]
            coord = xyz(chunk)
            mask = np.isfinite(coord).all(axis=1)
            coords.append(coord[mask])
            intensity.append(np.asarray(chunk['intensity'][mask], dtype='<f4'))
        return np.concatenate(coords), np.concatenate(intensity)

    def query_body(self, center: int, frames: int) -> np.ndarray:
        """T_body_ref_body_i = inverse(T_map_body_ref) * T_map_body_i."""
        if frames not in (1, 3, 5):
            raise ValueError('only 1, 3, or 5 frames are defined')
        ref = self.poses[center]
        aligned = []
        for index in range(center - frames // 2, center + frames // 2 + 1):
            pose = self.poses[index]
            patch = read_pcd(self.pgo / 'patches' / pose.patch, ('x', 'y', 'z'))
            points = xyz(patch).astype('f8')
            points = (points @ pose.rotation.T + pose.t - ref.t) @ ref.rotation
            aligned.append(points.astype('<f4'))
        return np.concatenate(aligned)


def fixed_split(poses: tuple[Pose, ...], centers: tuple[int, ...], tier: str,
                radius: float = 30.0) -> Split:
    """Predeclared spatial ROI/stride; never depends on registration outcome or Qt."""
    if tier not in ('tier0', 'tier1') or not centers or len(set(centers)) != len(centers):
        raise ValueError('invalid tier or centers')
    n = len(poses)
    if any(c < 3 or c >= n - 3 for c in centers):
        raise ValueError('center needs a full five-frame window')
    heldout = tuple(sorted({c + d for c in centers for d in (-2, -1, 0, 1, 2)}))
    # Bounded, fixed-size spatial ROI per predetermined reference pose. This
    # experiment uses PGO reference poses to pick its *evaluation map subset*;
    # the corresponding oracle ROI bias is disclosed, not a runtime selector.
    selected_set: set[int] = set()
    for c in centers:
        distances = [(float(np.linalg.norm(pose.t[:2] - poses[c].t[:2])), i)
                     for i, pose in enumerate(poses) if i % 3 != 2 and i not in heldout]
        nearby = sorted((dist, i) for dist, i in distances if dist <= radius)
        selected_set.update(i for dist, i in nearby[:MAX_MAP_PATCHES_PER_CENTER])
    selected = sorted(selected_set)
    if tier == 'tier0':
        selected = sorted(set(selected).union(heldout))
    if len(selected) < 10:
        raise ValueError('not enough map subset poses in predefined ROI')
    if tier == 'tier1' and set(heldout).intersection(selected):
        raise AssertionError('query patch entered tier1 map')
    return Split(tier, centers, tuple(selected), heldout,
                 tuple(i for i in range(n) if i not in selected), radius)
