from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import struct


@dataclass
class AsciiPcd:
    header: list[str]
    fields: list[str]
    rows: list[list[str]]

    @property
    def x_index(self) -> int:
        return self.fields.index('x')

    @property
    def y_index(self) -> int:
        return self.fields.index('y')

    @property
    def z_index(self) -> int:
        return self.fields.index('z')


def read_pcd(path: Path) -> AsciiPcd:
    raw = path.read_bytes()
    data_marker = b'DATA '
    data_offset = raw.upper().find(b'\n' + data_marker)
    if data_offset >= 0:
        data_offset += 1
    if data_offset < 0:
        raise ValueError(f'PCD has no DATA declaration: {path}')
    data_end = raw.find(b'\n', data_offset)
    header_text = raw[:data_end + 1].decode('ascii')
    lines = header_text.splitlines()
    data_index = len(lines) - 1
    if data_index is None:
        raise ValueError(f'PCD has no DATA declaration: {path}')
    fields_line = next((line for line in lines[:data_index] if line.upper().startswith('FIELDS ')), None)
    if fields_line is None:
        raise ValueError(f'PCD has no FIELDS declaration: {path}')
    fields = fields_line.split()[1:]
    if not {'x', 'y', 'z'}.issubset(fields):
        raise ValueError('PCD must contain x, y and z fields')
    data_format = lines[data_index].split(maxsplit=1)[1].lower()
    if data_format == 'ascii':
        rows = [line.split() for line in raw[data_end + 1:].decode('utf-8').splitlines() if line.strip()]
    elif data_format == 'binary':
        sizes = [int(value) for value in next(line for line in lines if line.upper().startswith('SIZE ')).split()[1:]]
        types = next(line for line in lines if line.upper().startswith('TYPE ')).split()[1:]
        counts = [int(value) for value in next(line for line in lines if line.upper().startswith('COUNT ')).split()[1:]]
        if any(count != 1 or size != 4 or kind != 'F' for size, kind, count in zip(sizes, types, counts)):
            raise ValueError('binary PCD requires one float32 value per field')
        stride = sum(sizes)
        if len(raw[data_end + 1:]) % stride:
            raise ValueError('binary PCD payload is not aligned to its field stride')
        rows = [[str(value) for value in struct.unpack('<' + 'f' * len(fields), raw[offset:offset + stride])]
                for offset in range(data_end + 1, len(raw), stride)]
    else:
        raise ValueError(f'unsupported PCD data encoding: {data_format}')
    if any(len(row) != len(fields) for row in rows):
        raise ValueError('PCD point row does not match FIELDS count')
    return AsciiPcd(lines[:data_index + 1], fields, rows)


def read_ascii_pcd(path: Path) -> AsciiPcd:
    """Backward-compatible name for the Phase 2 PCD reader."""
    return read_pcd(path)


def write_ascii_pcd(path: Path, pcd: AsciiPcd) -> None:
    header = []
    for line in pcd.header:
        key = line.split(maxsplit=1)[0].upper() if line.split() else ''
        if key in {'WIDTH', 'POINTS'}:
            header.append(f'{key} {len(pcd.rows)}')
        elif key == 'DATA':
            header.append('DATA ascii')
        else:
            header.append(line)
    path.write_text('\n'.join(header + [' '.join(row) for row in pcd.rows]) + '\n', encoding='utf-8')