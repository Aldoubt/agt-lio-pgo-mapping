from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import struct


@dataclass
class AsciiPcd:
    """In-memory PCD with string rows. ``source_format`` remembers the input encoding."""
    header: list[str]
    fields: list[str]
    rows: list[list[str]]
    source_format: str = field(default='ascii')

    @property
    def x_index(self) -> int:
        return self.fields.index('x')

    @property
    def y_index(self) -> int:
        return self.fields.index('y')

    @property
    def z_index(self) -> int:
        return self.fields.index('z')

    def header_value(self, key: str) -> list[str]:
        prefix = key.upper() + ' '
        for line in self.header:
            if line.upper().startswith(prefix):
                return line.split()[1:]
        return []


def _binary_layout(pcd: AsciiPcd) -> tuple[list[int], list[str], list[int]]:
    sizes = [int(value) for value in pcd.header_value('SIZE')]
    types = pcd.header_value('TYPE')
    counts = [int(value) for value in pcd.header_value('COUNT')] or [1] * len(pcd.fields)
    if not sizes or not types or len(sizes) != len(pcd.fields) or len(types) != len(pcd.fields):
        raise ValueError('PCD header SIZE/TYPE do not match FIELDS')
    if any(count != 1 or size != 4 or kind != 'F' for size, kind, count in zip(sizes, types, counts)):
        raise ValueError('binary PCD requires one float32 value per field')
    return sizes, types, counts


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
        pcd = AsciiPcd(lines[:data_index + 1], fields, [], 'binary')
        sizes, _, _ = _binary_layout(pcd)
        stride = sum(sizes)
        if len(raw[data_end + 1:]) % stride:
            raise ValueError('binary PCD payload is not aligned to its field stride')
        unpack = struct.Struct('<' + 'f' * len(fields)).unpack_from
        rows = [[repr(value) for value in unpack(raw, offset)]
                for offset in range(data_end + 1, len(raw), stride)]
    else:
        raise ValueError(f'unsupported PCD data encoding: {data_format}')
    if any(len(row) != len(fields) for row in rows):
        raise ValueError('PCD point row does not match FIELDS count')
    return AsciiPcd(lines[:data_index + 1], fields, rows, data_format)


def read_ascii_pcd(path: Path) -> AsciiPcd:
    """Backward-compatible name for the Phase 2 PCD reader."""
    return read_pcd(path)


def _rewritten_header(pcd: AsciiPcd, data_format: str) -> list[str]:
    header = []
    for line in pcd.header:
        key = line.split(maxsplit=1)[0].upper() if line.split() else ''
        if key == 'WIDTH':
            header.append(f'WIDTH {len(pcd.rows)}')
        elif key == 'HEIGHT':
            header.append('HEIGHT 1')
        elif key == 'POINTS':
            header.append(f'POINTS {len(pcd.rows)}')
        elif key == 'DATA':
            header.append(f'DATA {data_format}')
        else:
            header.append(line)
    return header


def write_ascii_pcd(path: Path, pcd: AsciiPcd) -> None:
    header = _rewritten_header(pcd, 'ascii')
    path.write_text('\n'.join(header + [' '.join(row) for row in pcd.rows]) + '\n', encoding='utf-8')


def write_binary_pcd(path: Path, pcd: AsciiPcd) -> None:
    """Write float32 binary PCD; avoids the size blow-up and precision loss of ASCII."""
    _binary_layout(pcd)
    header = '\n'.join(_rewritten_header(pcd, 'binary')) + '\n'
    pack = struct.Struct('<' + 'f' * len(pcd.fields)).pack
    with path.open('wb') as stream:
        stream.write(header.encode('ascii'))
        for row in pcd.rows:
            stream.write(pack(*(float(value) for value in row)))


def write_pcd(path: Path, pcd: AsciiPcd, data_format: str = 'auto') -> str:
    """Write ``pcd`` as ``ascii``/``binary``; ``auto`` keeps the source encoding."""
    chosen = pcd.source_format if data_format == 'auto' else data_format
    if chosen == 'binary':
        write_binary_pcd(path, pcd)
    elif chosen == 'ascii':
        write_ascii_pcd(path, pcd)
    else:
        raise ValueError(f'unsupported PCD output format: {data_format}')
    return chosen