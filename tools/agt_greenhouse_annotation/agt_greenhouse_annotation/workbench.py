"""Unified offline greenhouse structure workbench entry point and smoke runner."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import resource
import sys
import time

import numpy as np
import yaml

from .structure_artifacts import (
    GreenhouseStructureConfig,
    _weighted_quantile,
    analyze_greenhouse_structure,
    save_proposal_revision,
)
from .topology import load_map_package, load_topology, new_topology


def _peak_rss_mib() -> float:
    value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return float(value / (1024.0 if sys.platform != 'darwin' else 1024.0 * 1024.0))


def _default_structure_config() -> Path | None:
    filename = Path('config') / 'greenhouse_structure_current.yaml'
    for parent in Path(__file__).resolve().parents:
        candidate = parent / filename
        if candidate.is_file():
            return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory('agt_greenhouse_annotation')) / filename
        if candidate.is_file():
            return candidate
    except (ImportError, LookupError):
        pass
    return None


def _map_display_bounds(nav, margin_m: float = 2.5) -> tuple[float, float, float, float]:
    rows, cols = np.nonzero(np.asarray(nav.point_count) > 0)
    if not len(rows):
        return nav.bounds_m()
    weights = np.asarray(nav.point_count, dtype=np.float64)[rows, cols]
    x = nav.origin_x_m + (cols + 0.5) * nav.resolution_m
    y = nav.origin_y_m + (rows + 0.5) * nav.resolution_m
    return (
        _weighted_quantile(x, weights, 0.005) - margin_m,
        _weighted_quantile(x, weights, 0.995) + margin_m,
        _weighted_quantile(y, weights, 0.005) - margin_m,
        _weighted_quantile(y, weights, 0.995) + margin_m,
    )


def _reference_topology_summary(path: str | Path | None, package) -> dict | None:
    if path is None:
        return None
    topology_path = Path(path).expanduser().resolve()
    topology = load_topology(topology_path)
    source = topology.get('source', {})
    referenced_map = Path(source.get('map_package', '')).expanduser().resolve()
    manifest_hash = source.get('map_package_manifest_sha256')
    if referenced_map != package.path:
        raise ValueError(f'reference topology map package does not match: {referenced_map}')
    if manifest_hash != package.manifest_sha256:
        raise ValueError('reference topology map manifest SHA-256 does not match the smoke input')
    return {
        'path': str(topology_path),
        'file_sha256': hashlib.sha256(topology_path.read_bytes()).hexdigest(),
        'map_package': str(referenced_map),
        'map_package_manifest_sha256': manifest_hash,
        'annotation_status': topology.get('annotation', {}).get('status', 'UNKNOWN'),
        'manual_review_confirmed': bool(topology.get('annotation', {}).get('manual_review_confirmed', False)),
        'physical_row_count': len(topology.get('rows', [])),
        'headland_count': len(topology.get('headlands', [])),
        'scene_count': len(topology.get('scenes', [])),
    }


def _write_figures(analysis, output: Path) -> list[str]:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from scipy.ndimage import gaussian_filter1d
    from .navigation_structure import _hybrid_row_evidence

    output.mkdir(parents=True, exist_ok=True)
    nav = analysis.navigation
    extent = (nav.origin_x_m, nav.origin_x_m + nav.width * nav.resolution_m,
              nav.origin_y_m, nav.origin_y_m + nav.height * nav.resolution_m)
    x_min, x_max, y_min, y_max = _map_display_bounds(nav)
    saved = []
    fig, axes = plt.subplots(2, 2, figsize=(15, 11), constrained_layout=True)
    axes[0, 0].imshow(nav.occupancy, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=255)
    axes[0, 0].set_title('Ground-relative map evidence')
    axes[0, 1].imshow(analysis.terrain.ground_confidence, origin='lower', extent=extent, vmin=0, vmax=1, cmap='viridis')
    axes[0, 1].set_title('Ground confidence')
    axes[1, 0].imshow(analysis.terrain.ridge_evidence, origin='lower', extent=extent, vmin=0, vmax=1, cmap='magma')
    axes[1, 0].set_title('Terrain ridge evidence')
    axes[1, 1].imshow(analysis.terrain.depression_evidence, origin='lower', extent=extent, vmin=0, vmax=1, cmap='Blues')
    axes[1, 1].set_title('Terrain depression evidence')
    for ax in axes.ravel():
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel('map x [m]')
        ax.set_ylabel('map y [m]')
    path = output / 'current_greenhouse_structure_overview.png'
    fig.savefig(path, dpi=160)
    plt.close(fig)
    saved.append(str(path))

    fig, axes = plt.subplots(1, 2, figsize=(16, 7), constrained_layout=True)
    if analysis.global_result is not None:
        axes[0].imshow(analysis.global_result.row_regularized_obstacle, origin='lower', extent=extent, cmap='Greens')
        for row in analysis.global_rows:
            points = np.asarray(row.centerline_xy)
            axes[0].plot(points[:, 0], points[:, 1], color='#187b35', linewidth=1.4)
    axes[0].set_title('GLOBAL_PROFILE rows')
    if analysis.local_result is not None:
        axes[1].imshow(analysis.local_result.row_structural_band, origin='lower', extent=extent, cmap='Purples')
        for row in analysis.local_rows:
            points = np.asarray(row.centerline_xy)
            axes[1].plot(points[:, 0], points[:, 1], color='#602d91', linewidth=1.4)
    axes[1].set_title('LOCAL_TRACKS rows')
    for ax in axes:
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel('map x [m]')
        ax.set_ylabel('map y [m]')
    path = output / 'current_greenhouse_global_vs_local_rows.png'
    fig.savefig(path, dpi=160)
    plt.close(fig)
    saved.append(str(path))

    fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    ax.imshow(nav.occupancy, origin='lower', extent=extent, cmap='gray', alpha=.6, vmin=0, vmax=255)
    if analysis.geometric_aisle_result is not None:
        geometric = analysis.geometric_aisle_result.mask
        ax.imshow(np.ma.masked_where(~geometric, geometric), origin='lower', extent=extent,
                  cmap='Blues', alpha=.55, vmin=0, vmax=1)
    if analysis.corridor_result is not None:
        safe = analysis.corridor_result.aisle_centerline
        ax.contour(safe, levels=[.5], origin='lower', extent=extent, colors='#e44723', linewidths=1.2)
    if analysis.local_result is not None:
        ax.contour(analysis.local_result.aisle_centerline, levels=[.5], origin='lower', extent=extent,
                   colors='#25a781', linewidths=1.0, linestyles='dashed')
    ax.set_title('Geometric (blue), safe global (red), local-track (green dashed) aisle centerlines')
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    path = output / 'current_greenhouse_aisle_centerlines.png'
    fig.savefig(path, dpi=160)
    plt.close(fig)
    saved.append(str(path))

    # Paper-facing profile uses the actual hybrid evidence raster and proposal
    # locations from this analysis revision.
    profile_fig, profile_ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    if analysis.global_result is not None:
        direction = np.asarray(analysis.global_result.row_model.direction_xy, dtype=float)
        perpendicular = np.array([-direction[1], direction[0]])
        active_rows, active_cols = np.nonzero(np.asarray(nav.point_count) > 0)
        x = nav.origin_x_m + (active_cols + .5) * nav.resolution_m
        y = nav.origin_y_m + (active_rows + .5) * nav.resolution_m
        v = x * perpendicular[0] + y * perpendicular[1]
        hybrid = _hybrid_row_evidence(nav, analysis.terrain.ridge_evidence, analysis.config.global_profile)
        weight = hybrid[active_rows, active_cols]
        valid = weight > 0.05
        if np.any(valid):
            v_min = float(np.min(v[valid]))
            v_max = float(np.max(v[valid]))
            edges = np.arange(v_min, v_max + nav.resolution_m * 1.01, nav.resolution_m)
            profile, edges = np.histogram(v[valid], bins=edges, weights=weight[valid])
            centers = .5 * (edges[:-1] + edges[1:])
            profile = gaussian_filter1d(profile.astype(float), sigma=max(.5, .2 / nav.resolution_m))
            profile_ax.plot(centers, profile, color='#315b89', lw=1.4, label='Smoothed hybrid P(v)')
            for row in analysis.global_rows:
                profile_ax.axvline(row.lateral_v_m, color='#d88718', alpha=.55, lw=.8)
            profile_ax.set_xlabel('Cross-row coordinate v [m]')
            profile_ax.set_ylabel('Weighted hybrid evidence')
            profile_ax.legend(loc='upper right')
    profile_ax.set_title('Global row profile and AUTO candidate centers')
    path = output / 'fig_global_row_profile.png'
    profile_fig.savefig(path, dpi=170)
    plt.close(profile_fig)
    saved.append(str(path))

    # A dedicated local view exposes the longitudinal fragments used to build
    # tracks, without implying that each fragment is a confirmed physical row.
    local_fig, local_ax = plt.subplots(figsize=(11, 8), constrained_layout=True)
    local_ax.imshow(nav.occupancy, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=255, alpha=.55)
    if analysis.local_result is not None and analysis.local_rows:
        for row in analysis.local_rows:
            points = np.asarray(row.centerline_xy)
            local_ax.plot(points[:, 0], points[:, 1], color='#623a9b', alpha=.72, linewidth=1.1)
        for track in analysis.local_result.tracks:
            points = np.asarray([(item.u_center_m, item.v_center_m) for item in track.observations])
            if len(points):
                direction = np.asarray(analysis.local_rows[0].direction_xy, dtype=float)
                perpendicular = np.array([-direction[1], direction[0]])
                xy = points[:, :1] * direction[None, :] + points[:, 1:] * perpendicular[None, :]
                local_ax.scatter(xy[:, 0], xy[:, 1], s=7, color='#ed8b31', alpha=.60)
    local_ax.set_xlim(x_min, x_max)
    local_ax.set_ylim(y_min, y_max)
    local_ax.set_aspect('equal', adjustable='box')
    local_ax.set_xlabel('map x [m]')
    local_ax.set_ylabel('map y [m]')
    local_ax.set_title(f'LOCAL_TRACKS observations and track segments ({len(analysis.local_rows)} AUTO tracks)')
    path = output / 'fig_local_row_tracks.png'
    local_fig.savefig(path, dpi=170)
    plt.close(local_fig)
    saved.append(str(path))

    # The combined candidate comparison is also exported under the stable
    # paper figure name used by the experiment protocol.
    combined_fig, combined_axes = plt.subplots(1, 2, figsize=(16, 7), constrained_layout=True)
    if analysis.global_result is not None:
        combined_axes[0].imshow(analysis.global_result.row_regularized_obstacle, origin='lower', extent=extent, cmap='Greens')
        for row in analysis.global_rows:
            points = np.asarray(row.centerline_xy)
            combined_axes[0].plot(points[:, 0], points[:, 1], color='#187b35', linewidth=1.2)
    if analysis.local_result is not None:
        combined_axes[1].imshow(analysis.local_result.row_structural_band, origin='lower', extent=extent, cmap='Purples')
        for row in analysis.local_rows:
            points = np.asarray(row.centerline_xy)
            combined_axes[1].plot(points[:, 0], points[:, 1], color='#602d91', linewidth=1.0)
    for ax, title in zip(combined_axes, ('GLOBAL_PROFILE', 'LOCAL_TRACKS')):
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal', adjustable='box')
        ax.set_title(title)
        ax.set_xlabel('map x [m]')
        ax.set_ylabel('map y [m]')
    path = output / 'fig_global_vs_local_row_proposals.png'
    combined_fig.savefig(path, dpi=170)
    plt.close(combined_fig)
    saved.append(str(path))

    centerline_fig, centerline_axes = plt.subplots(1, 2, figsize=(15, 7), constrained_layout=True)
    centerline_axes[0].imshow(nav.occupancy, origin='lower', extent=extent, cmap='gray', alpha=.65, vmin=0, vmax=255)
    if analysis.geometric_aisle_result is not None:
        centerline_axes[0].contour(analysis.geometric_aisle_result.mask, levels=[.5], origin='lower',
                                   extent=extent, colors='#2688dd', linewidths=1.2)
    if analysis.global_result is not None:
        for row in analysis.global_rows:
            points = np.asarray(row.centerline_xy)
            centerline_axes[0].plot(points[:, 0], points[:, 1], color='#e59b23', alpha=.55, lw=.7)
    centerline_axes[0].set_title('Geometric structure centerline')
    centerline_axes[1].imshow(nav.occupancy, origin='lower', extent=extent, cmap='gray', alpha=.65, vmin=0, vmax=255)
    if analysis.corridor_result is not None:
        centerline_axes[1].contour(analysis.corridor_result.aisle_centerline, levels=[.5], origin='lower',
                                   extent=extent, colors='#eb5236', linewidths=1.2)
    centerline_axes[1].set_title('Safe centerline from local clearance evidence')
    for ax in centerline_axes:
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('map x [m]')
        ax.set_ylabel('map y [m]')
    path = output / 'fig_geometric_vs_safe_centerline.png'
    centerline_fig.savefig(path, dpi=170)
    plt.close(centerline_fig)
    saved.append(str(path))

    overview_fig, overview_ax = plt.subplots(figsize=(12, 9), constrained_layout=True)
    overview_ax.imshow(nav.occupancy, origin='lower', extent=extent, cmap='gray', alpha=.60, vmin=0, vmax=255)
    for row in analysis.global_rows:
        points = np.asarray(row.centerline_xy)
        overview_ax.plot(points[:, 0], points[:, 1], color='#df9a1d', lw=1.0, alpha=.75)
    for row in analysis.local_rows:
        points = np.asarray(row.centerline_xy)
        overview_ax.plot(points[:, 0], points[:, 1], color='#69439c', lw=.8, alpha=.55)
    overview_ax.set_xlim(x_min, x_max)
    overview_ax.set_ylim(y_min, y_max)
    overview_ax.set_aspect('equal', adjustable='box')
    overview_ax.set_xlabel('map x [m]')
    overview_ax.set_ylabel('map y [m]')
    overview_ax.set_title('Current greenhouse workbench overview: global and local AUTO proposals')
    overview_ax.text(.02, .02,
                     f'global={len(analysis.global_rows)} | local={len(analysis.local_rows)} | '
                     f'aisles={len(analysis.aisle_proposals)} | hash={analysis.analysis_hash[:12]}',
                     transform=overview_ax.transAxes, color='black', fontsize=10,
                     bbox={'facecolor': 'white', 'alpha': .82, 'edgecolor': 'none'})
    path = output / 'fig_structure_workbench_overview.png'
    overview_fig.savefig(path, dpi=170)
    plt.close(overview_fig)
    saved.append(str(path))

    pipeline_fig, pipeline_ax = plt.subplots(figsize=(14, 3.6), constrained_layout=True)
    pipeline_ax.axis('off')
    boxes = [
        ('Validated map', f'{analysis.diagnostics.get("input_point_count", 0):,} PCD points'),
        ('Terrain evidence', f'{np.count_nonzero(analysis.terrain.ground_valid):,} ground cells'),
        ('Global rows', f'{len(analysis.global_rows)} AUTO proposals'),
        ('Local tracks', f'{len(analysis.local_rows)} AUTO tracks'),
        ('Aisle proposals', f'{len(analysis.aisle_proposals)} AUTO aisles'),
        ('Physical topology', '0 rows before review'),
    ]
    for index, (title, detail) in enumerate(boxes):
        x = .09 + index * .164
        pipeline_ax.text(x, .54, f'{title}\n{detail}', ha='center', va='center', fontsize=9,
                         bbox={'boxstyle': 'round,pad=.7', 'facecolor': '#eaf0f5', 'edgecolor': '#36516a'},
                         transform=pipeline_ax.transAxes)
        if index < len(boxes) - 1:
            pipeline_ax.annotate('', xy=(x + .094, .54), xytext=(x + .070, .54),
                                 xycoords=pipeline_ax.transAxes, textcoords=pipeline_ax.transAxes,
                                 arrowprops={'arrowstyle': '->', 'color': '#36516a', 'lw': 1.3})
    pipeline_ax.set_title(f'Proposal pipeline | Point-LIO | map {analysis.input_map_hash[:12]} | {analysis.analysis_hash[:12]}')
    path = output / 'fig_structure_pipeline.png'
    pipeline_fig.savefig(path, dpi=170)
    plt.close(pipeline_fig)
    saved.append(str(path))
    return saved


def _write_tables(analysis, output: Path) -> dict[str, str]:
    output.mkdir(parents=True, exist_ok=True)
    row_path = output / 'greenhouse_row_proposals.csv'
    with row_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=('source_mode', 'auto_id', 'v_m', 'width_m', 'support',
                                                       'support_fraction', 'centerline_xy', 'diagnostics'))
        writer.writeheader()
        for row in (*analysis.global_rows, *analysis.local_rows):
            writer.writerow({'source_mode': row.source_mode, 'auto_id': row.auto_id,
                             'v_m': row.lateral_v_m, 'width_m': row.half_width_m * 2,
                             'support': row.profile_support,
                             'support_fraction': row.longitudinal_support_fraction,
                             'centerline_xy': json.dumps(row.centerline_xy),
                             'diagnostics': json.dumps(row.diagnostics, sort_keys=True)})
    aisle_path = output / 'greenhouse_aisle_proposals.csv'
    with aisle_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=('auto_id', 'left_auto_row', 'right_auto_row', 'kind',
                                                       'width_m', 'minimum_width_m', 'centerline_xy', 'diagnostics'))
        writer.writeheader()
        for aisle in analysis.aisle_proposals:
            writer.writerow({'auto_id': aisle.auto_id, 'left_auto_row': aisle.left_row_auto_id,
                             'right_auto_row': aisle.right_row_auto_id, 'kind': aisle.aisle_kind,
                             'width_m': aisle.geometric_width_m, 'minimum_width_m': aisle.minimum_width_m,
                             'centerline_xy': json.dumps(aisle.centerline_xy),
                             'diagnostics': json.dumps(aisle.diagnostics, sort_keys=True)})

    paper_rows = output / 'table_row_proposal_summary.csv'
    with paper_rows.open('w', newline='', encoding='utf-8') as stream:
        fields = ('scene_map', 'method', 'auto_row_id', 'length_m', 'mean_support',
                  'confidence', 'center_v_m', 'physical_row_id')
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in (*analysis.global_rows, *analysis.local_rows):
            points = np.asarray(row.centerline_xy, dtype=float)
            length = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1))) if len(points) > 1 else 0.0
            support = float(row.diagnostics.get('mean_support', row.profile_support))
            writer.writerow({
                'scene_map': 'current_greenhouse_point_lio', 'method': row.source_mode,
                'auto_row_id': row.auto_id, 'length_m': length, 'mean_support': support,
                'confidence': min(float(row.profile_support), float(row.longitudinal_support_fraction)),
                'center_v_m': row.lateral_v_m,
                'physical_row_id': row.physical_row_id or 'UNKNOWN',
            })

    paper_aisles = output / 'table_aisle_summary.csv'
    with paper_aisles.open('w', newline='', encoding='utf-8') as stream:
        fields = ('aisle_proposal_id', 'left_auto_row', 'right_auto_row', 'length_m',
                  'median_width_m', 'minimum_width_m', 'geometric_status', 'safe_status',
                  'physical_aisle_id')
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for aisle in analysis.aisle_proposals:
            geometric_points = np.asarray(aisle.centerline_xy, dtype=float).reshape(-1, 2)
            safe_points = np.asarray(aisle.safe_centerline_xy, dtype=float).reshape(-1, 2)
            length = float(np.sum(np.linalg.norm(np.diff(geometric_points, axis=0), axis=1))) if len(geometric_points) > 1 else 0.0
            safe_length = float(np.sum(np.linalg.norm(np.diff(safe_points, axis=0), axis=1))) if len(safe_points) > 1 else 0.0
            writer.writerow({
                'aisle_proposal_id': aisle.auto_id, 'left_auto_row': aisle.left_row_auto_id,
                'right_auto_row': aisle.right_row_auto_id, 'length_m': length,
                # Current aisle proposal width is a geometry envelope scalar;
                # along-row width variation is retained as a known limitation.
                'median_width_m': aisle.geometric_width_m,
                'minimum_width_m': aisle.minimum_width_m,
                'geometric_status': aisle.diagnostics.get('status', 'CANDIDATE'),
                'safe_status': 'SAFE_CENTERLINE_PRESENT' if safe_length > 0 else 'NO_SAFE_CENTERLINE',
                'physical_aisle_id': aisle.physical_aisle_id or 'UNKNOWN',
            })

    paper_comparison = output / 'table_global_vs_local_rows.csv'
    global_rows, local_rows = list(analysis.global_rows), list(analysis.local_rows)
    if global_rows and local_rows:
        from scipy.optimize import linear_sum_assignment
        cost = np.abs(np.asarray([row.lateral_v_m for row in global_rows])[:, None]
                      - np.asarray([row.lateral_v_m for row in local_rows])[None, :])
        global_indices, local_indices = linear_sum_assignment(cost)
        tolerance = max(analysis.navigation.resolution_m * 2,
                        global_rows[0].half_width_m + local_rows[0].half_width_m)
        matched_diffs = [float(cost[i, j]) for i, j in zip(global_indices, local_indices)
                         if cost[i, j] <= tolerance]
    else:
        matched_diffs = []
    with paper_comparison.open('w', newline='', encoding='utf-8') as stream:
        fields = ('scene_map', 'global_row_count', 'local_track_count', 'matched_count',
                  'unmatched_global_count', 'unmatched_local_count', 'mean_center_difference_m',
                  'match_tolerance_m')
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow({
            'scene_map': 'current_greenhouse_point_lio',
            'global_row_count': len(global_rows), 'local_track_count': len(local_rows),
            'matched_count': len(matched_diffs),
            'unmatched_global_count': len(global_rows) - len(matched_diffs),
            'unmatched_local_count': len(local_rows) - len(matched_diffs),
            'mean_center_difference_m': float(np.mean(matched_diffs)) if matched_diffs else '',
            'match_tolerance_m': tolerance if global_rows and local_rows else '',
        })
    return {
        'rows': str(row_path), 'aisles': str(aisle_path),
        'paper_row_summary': str(paper_rows), 'paper_aisle_summary': str(paper_aisles),
        'paper_global_vs_local': str(paper_comparison),
    }


def run_real_smoke(map_package_path: str | Path, site_workspace: str | Path,
                   config_path: str | Path | None = None, mode: str = 'COMPARE',
                   reference_topology_path: str | Path | None = None) -> dict:
    from .point_cloud import read_pcd_cloud

    start = time.perf_counter()
    package = load_map_package(map_package_path)
    reference_topology = _reference_topology_summary(reference_topology_path, package)
    if config_path:
        mapping = yaml.safe_load(Path(config_path).read_text(encoding='utf-8')) or {}
        config = GreenhouseStructureConfig.from_mapping(mapping)
    else:
        config = GreenhouseStructureConfig()
    cloud = read_pcd_cloud(package.path / 'map.pcd')
    analysis = analyze_greenhouse_structure(cloud, package.map_package_hash, mode=mode, config=config)
    source_provenance = new_topology(package)['source']
    workspace = Path(site_workspace).expanduser().resolve()
    artifact = save_proposal_revision(analysis, workspace, source_provenance)
    figure_root = workspace / 'paper' / 'figures'
    table_root = workspace / 'paper' / 'tables'
    figures = _write_figures(analysis, figure_root)
    tables = _write_tables(analysis, table_root)
    elapsed = time.perf_counter() - start
    report = {
        'status': 'SMOKE_ONLY', 'mode': analysis.mode, 'map_package': str(package.path),
        'map_manifest_sha256': package.manifest_sha256,
        'point_count': int(len(cloud.points)),
        'backend_id': package.backend_id,
        'backend_commit': source_provenance.get('backend_commit', 'UNKNOWN'),
        'mapping_repository_commit': source_provenance.get('mapping_repository_commit', 'UNKNOWN'),
        'rosbag_metadata_sha256': source_provenance.get('rosbag_metadata_sha256'),
        'physical_topology_rows_before_review': (
            reference_topology['physical_row_count'] if reference_topology is not None
            else len(new_topology(package)['rows'])
        ),
        'reference_topology': reference_topology,
        'global_row_candidates': [row.auto_id for row in analysis.global_rows],
        'local_track_candidates': [row.auto_id for row in analysis.local_rows],
        'aisle_candidates': [aisle.auto_id for aisle in analysis.aisle_proposals],
        'geometric_aisle_count': (analysis.geometric_aisle_result.geometric_aisle_count
                                  if analysis.geometric_aisle_result else 0),
        'safe_centerline_cells': int(np.count_nonzero(analysis.corridor_result.aisle_centerline))
                                  if analysis.corridor_result else 0,
        'local_centerline_cells': int(np.count_nonzero(analysis.local_result.aisle_centerline))
                                  if analysis.local_result else 0,
        'diagnostics': dict(analysis.diagnostics), 'analysis_hash': analysis.analysis_hash,
        'stage_timings_seconds': dict(analysis.diagnostics.get('stage_timings_seconds', {})),
        'proposal_artifact': str(artifact), 'figures': figures, 'tables': tables,
        'elapsed_seconds': elapsed, 'peak_rss_mib': _peak_rss_mib(),
        'physical_row_metrics': 'UNKNOWN/N/A pending manual physical ID and direction confirmation',
        'topk_behavior': 'reads frozen schema-v1 topology; detector is not invoked by benchmark',
    }
    report_path = workspace / 'structure_smoke_report.json'
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-package', required=True)
    parser.add_argument('--site-workspace', required=True)
    parser.add_argument('--config', default=str(_default_structure_config() or ''))
    parser.add_argument('--mode', choices=('GLOBAL_PROFILE', 'LOCAL_TRACKS', 'COMPARE'), default='COMPARE')
    parser.add_argument('--reference-topology', help='existing schema-v1 topology to verify as the map-bound review source')
    parser.add_argument('--analyze-only', action='store_true', help='run batch evidence smoke without opening Qt')
    parser.add_argument('--output', help='topology draft YAML path for GUI mode')
    args = parser.parse_args(argv)
    if args.analyze_only:
        report = run_real_smoke(args.map_package, args.site_workspace, args.config, args.mode,
                                args.reference_topology)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    from .gui import main as gui_main
    workspace = Path(args.site_workspace).expanduser().resolve()
    output = Path(args.output).expanduser().resolve() if args.output else workspace / 'annotation' / 'greenhouse_topology.yaml'
    return gui_main(['--map-package', args.map_package, '--output', str(output),
                     '--site-workspace', str(workspace), '--structure-config', args.config])


if __name__ == '__main__':
    raise SystemExit(main())
