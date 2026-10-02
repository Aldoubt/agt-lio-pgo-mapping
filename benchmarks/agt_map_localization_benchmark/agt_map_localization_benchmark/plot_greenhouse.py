"""Plot sampled greenhouse GICP basin slices from a completed run."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load_rows(path: Path) -> list[dict]:
    with Path(path).open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'empty basin CSV: {path}')
    return rows


def choose_yaws(values: list[float], maximum: int = 5) -> list[float]:
    unique = sorted(set(values))
    if len(unique) <= maximum:
        return unique
    desired = np.linspace(0, len(unique) - 1, maximum)
    return [unique[int(round(i))] for i in desired]


def plot_title(scene_id: str, frames: int, yaw: float, metric: str) -> str:
    display_id = scene_id.replace('trajectory_row_group_', 'rowgroup_')
    return f'{display_id}\nf={frames} | yaw={yaw:+g} deg | {metric}'


def plot(run: Path, metric: str = 'nominal_success') -> int:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit('matplotlib is required: sudo apt install python3-matplotlib') from exc
    run = Path(run).resolve(strict=True)
    rows = load_rows(run / 'basin' / 'basin.csv')
    if metric not in rows[0]:
        raise ValueError(f'unknown basin metric: {metric}')
    output = run / 'basin' / 'plots'
    output.mkdir(exist_ok=True)
    groups = sorted(set((r['scene_id'], int(r['frames'])) for r in rows))
    count = 0
    for scene_id, frames in groups:
        subset = [r for r in rows if r['scene_id'] == scene_id and int(r['frames']) == frames]
        yaws = choose_yaws([float(r['dyaw_deg']) for r in subset])
        for yaw in yaws:
            plane = [r for r in subset if abs(float(r['dyaw_deg']) - yaw) < 1e-9]
            xs = sorted(set(float(r['dx_m']) for r in plane))
            ys = sorted(set(float(r['dy_m']) for r in plane))
            x_lookup = {v: i for i, v in enumerate(xs)}
            y_lookup = {v: i for i, v in enumerate(ys)}
            grid = np.full((len(ys), len(xs)), np.nan, dtype='f8')
            for row in plane:
                raw = str(row[metric]).strip().lower()
                value = 1.0 if raw in ('true', '1', '1.0') else 0.0
                grid[y_lookup[float(row['dy_m'])], x_lookup[float(row['dx_m'])]] = value
            fig, ax = plt.subplots(figsize=(6.0, 5.0))
            image = ax.imshow(grid, origin='lower', vmin=0.0, vmax=1.0, aspect='auto',
                              extent=(min(xs), max(xs), min(ys), max(ys)))
            ax.set_xlabel('initial dx [m]')
            ax.set_ylabel('initial dy [m]')
            ax.set_title(plot_title(scene_id, frames, yaw, metric))
            fig.colorbar(image, ax=ax, label='success (1=yes, 0=no)')
            fig.tight_layout()
            name = f'{scene_id}_f{frames}_yaw_{yaw:+g}_{metric}.png'.replace('+', 'p').replace('-', 'm')
            fig.savefig(output / name, dpi=150)
            plt.close(fig)
            count += 1
    print(f'wrote {count} basin plots to {output}')
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description='Plot greenhouse GICP basin yaw slices from basin.csv')
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--metric', choices=('strict_success', 'nominal_success', 'loose_success'),
                   default='nominal_success')
    args = p.parse_args(argv)
    return plot(args.run, args.metric)


if __name__ == '__main__':
    raise SystemExit(main())
