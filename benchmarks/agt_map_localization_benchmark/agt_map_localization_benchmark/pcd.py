"""Small, strict PCD/immutability I/O adapter. No point-cloud registration code.

The PGO and evidence files in this experiment are uncompressed binary PCDs.
The reader respects declared POINTS (PCL may append zero page padding).
"""
from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Iterable

import numpy as np


_DTYPES = {
    ('F', 4): '<f4', ('F', 8): '<f8',
    ('I', 1): '<i1', ('I', 2): '<i2', ('I', 4): '<i4', ('I', 8): '<i8',
    ('U', 1): '<u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8',
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def verify_checksum_index(folder: Path, required: Iterable[str] = ()) -> dict[str, str]:
    """Verify *each* indexed source byte without writing to the source tree."""
    folder = Path(folder).resolve(strict=True)
    index = folder / 'checksums.sha256'
    if not index.is_file() or index.is_symlink():
        raise ValueError(f'missing/unsafe checksum index: {index}')
    entries: dict[str, str] = {}
    for line in index.read_text(encoding='ascii').splitlines():
        found = re.fullmatch(r'([a-fA-F0-9]{64})  (.+)', line)
        if not found:
            raise ValueError(f'bad checksum entry: {line[:100]!r}')
        digest, name = found.groups()
        rel = Path(name)
        if rel.is_absolute() or not rel.parts or '..' in rel.parts or name in entries:
            raise ValueError(f'unsafe/duplicate checksum entry: {name}')
        file = folder / rel
        if not file.is_file() or file.is_symlink() or not file.resolve().is_relative_to(folder):
            raise ValueError(f'unsafe/missing source file: {file}')
        if sha256_file(file) != digest.lower():
            raise ValueError(f'source checksum mismatch: {file}')
        entries[name] = digest.lower()
    if not entries or not set(required).issubset(entries):
        raise ValueError(f'checksum index missing required files: {set(required) - set(entries)}')
    return entries


def read_pcd(path: Path, fields: Iterable[str] = ()) -> np.memmap:
    """Read declared records only; fail closed on malformed/unexpected formats."""
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'missing/unsafe PCD: {path}')
    header: dict[str, list[str]] = {}
    with path.open('rb') as stream:
        for _ in range(64):
            raw = stream.readline(4096)
            if not raw or len(raw) >= 4096:
                raise ValueError(f'PCD header missing or too large: {path}')
            line = raw.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            tokens = line.split()
            if tokens[0] in header:
                raise ValueError(f'duplicate PCD field: {tokens[0]}')
            header[tokens[0]] = tokens[1:]
            if tokens[0] == 'DATA':
                offset = stream.tell()
                break
        else:
            raise ValueError(f'PCD DATA header missing: {path}')
    if header['DATA'] != ['binary']:
        raise ValueError(f'only PCD binary is accepted: {path}')
    names = header['FIELDS']
    sizes = [int(v) for v in header['SIZE']]
    types = header['TYPE']
    counts = [int(v) for v in header.get('COUNT', ['1'] * len(names))]
    if len(set(names)) != len(names) or len(names) != len(sizes) or len(names) != len(types) or len(names) != len(counts):
        raise ValueError(f'inconsistent PCD field header: {path}')
    if any(n != 1 for n in counts):
        raise ValueError(f'unsupported multi-count PCD field: {path}')
    try:
        dtype = np.dtype([(n, _DTYPES[(t, size)]) for n, t, size in zip(names, types, sizes)])
    except (KeyError, TypeError) as error:
        raise ValueError(f'unsupported PCD field format: {path}') from error
    if not set(fields).issubset(names):
        raise ValueError(f'PCD lacks {set(fields) - set(names)}: {path}')
    points = int(header['POINTS'][0])
    if points < 1 or int(header['WIDTH'][0]) * int(header['HEIGHT'][0]) != points:
        raise ValueError(f'invalid PCD point count: {path}')
    end = offset + points * dtype.itemsize
    size = path.stat().st_size
    if size < end or size - end > 4096:
        raise ValueError(f'inconsistent PCD payload length: {path}')
    if size != end:
        with path.open('rb') as stream:
            stream.seek(end)
            if any(stream.read()):
                raise ValueError(f'nonzero trailer after declared PCD points: {path}')
    return np.memmap(path, dtype=dtype, mode='r', offset=offset, shape=(points,))


def xyz(points: np.ndarray) -> np.ndarray:
    return np.column_stack((points['x'], points['y'], points['z'])).astype('<f4', copy=False)


def write_pcd(path: Path, coords: np.ndarray, intensity: np.ndarray | None = None) -> None:
    """Write an offline candidate/query into a *new* file, never a source PCD."""
    path = Path(path)
    coord = np.asarray(coords, dtype='<f4')
    if coord.ndim != 2 or coord.shape[1] != 3 or not len(coord) or not np.isfinite(coord).all():
        raise ValueError(f'PCD requires nonempty finite Nx3 coordinates: {path}')
    if intensity is None:
        refl = np.zeros(len(coord), dtype='<f4')
    else:
        refl = np.asarray(intensity, dtype='<f4')
        if refl.shape != (len(coord),) or not np.isfinite(refl).all():
            raise ValueError('invalid intensity input')
    payload = np.empty((len(coord), 4), dtype='<f4')
    payload[:, :3] = coord
    payload[:, 3] = refl
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
        'FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {len(coord)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(coord)}\nDATA binary\n'
    )
    with path.open('xb') as stream:
        stream.write(header.encode('ascii'))
        payload.tofile(stream)
