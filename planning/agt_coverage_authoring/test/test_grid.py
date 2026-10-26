"""Distance lower bounds, map transforms, conservative masks and cache freshness."""
from dataclasses import replace
import math

import numpy as np
import pytest
from shapely.geometry import Point, box

from agt_coverage_authoring import grid as module
from agt_coverage_authoring.grid import (
    analyze_traversability, clearance_field, forbidden_geometry, map_polygon,
    planning_cells, validate_grid, world_to_grid, write_planning_map,
)
from agt_coverage_authoring.io import load_grid
from agt_coverage_authoring.models import CoverageError, GridMap, PlanningContext, Pose, RobotProfile, WorkRow


def context(grid=None):
    grid = grid or GridMap(np.zeros((20, 20), dtype=np.int8), .1, (0, 0, 0))
    robot = RobotProfile('fixture', 'a' * 64, .6, 1.5,
                         ((-.4, -.3), (.4, -.3), (.4, .3), (-.4, .3)))
    return PlanningContext(grid, robot, map_polygon(grid), [], map_polygon(grid), [], Pose(1, 1, 0), Pose(1, 1, 0))


def test_distance_field_is_conservative_and_unknown_stays_unknown():
    data = np.zeros((11, 11), dtype=np.int8)
    data[5, 5] = 100
    data[2, 2] = -1
    grid = GridMap(data, .1, (0, 0, 0))
    layers = clearance_field(grid)
    assert layers['distance_m'][5, 5] == layers['distance_m'][2, 2] == 0
    assert not layers['known'][2, 2]
    assert not layers['free'][2, 2]
    assert 0 < layers['distance_m'][5, 6] <= .05
    assert set(np.unique(layers['planning_cells'])) == {-1, 0, 100}


def test_boundary_and_partial_keepout_cell_are_conservative():
    grid = GridMap(np.zeros((10, 10), dtype=np.int8), .1, (0, 0, 0))
    cells = planning_cells(grid, [box(.501, .501, .502, .502)], box(.05, .05, .95, .95))
    assert cells[5, 5] == 100
    assert np.all(cells[0] == 100) and np.all(cells[:, -1] == 100)
    assert cells[2, 2] == 0
    assert not grid.data.any()  # base raster remains untouched


def test_world_grid_roundtrip_handles_origin_yaw():
    grid = GridMap(np.zeros((10, 20), dtype=np.int8), .1, (3, 4, math.pi / 2))
    assert world_to_grid(2.95, 4.05, grid) == (0, 0)
    assert map_polygon(grid).covers(Point(2.5, 5))
    assert not map_polygon(grid).covers(Point(3.1, 4))


def test_mutable_grid_changes_never_reuse_old_forbidden_geometry():
    ctx = context()
    assert forbidden_geometry(ctx).is_empty
    ctx.grid.data[5, 5] = 100
    assert forbidden_geometry(ctx).covers(Point(.55, .55))


def test_immutable_grid_cache_invalidates_on_keepout_or_raster_replacement():
    ctx = context()
    ctx.grid.data.flags.writeable = False
    first = forbidden_geometry(ctx)
    assert forbidden_geometry(ctx) is first
    ctx.keepouts.append(box(.5, .5, .6, .6))
    assert forbidden_geometry(ctx).covers(Point(.55, .55))
    ctx.keepouts.clear()
    replacement = ctx.grid.data.copy()
    replacement[10, 10] = -1
    replacement.flags.writeable = False
    ctx.grid.data = replacement
    assert forbidden_geometry(ctx).covers(Point(1.05, 1.05))


@pytest.mark.parametrize('data', [np.array([[5]]), np.array([[float('nan')]]), np.zeros(0), np.zeros((0, 2))])
def test_invalid_trinary_grid_rejected(data):
    with pytest.raises(CoverageError):
        validate_grid(GridMap(data, .1, (0, 0, 0)))


def test_fragmented_grid_union_has_explicit_resource_limit(monkeypatch):
    ctx = context()
    ctx.grid.data[::2, ::2] = 100
    monkeypatch.setattr(module, 'MAX_OBSTACLE_RUNS', 5)
    with pytest.raises(CoverageError) as error:
        forbidden_geometry(ctx)
    assert error.value.code == 'GRID_COMPLEXITY_LIMIT'


def test_row_analysis_sample_budget_and_cancellation():
    ctx = context()
    ctx.rows = [WorkRow('row', ((.5, .5), (1.5, .5)))]
    _, report = analyze_traversability(ctx)
    assert report['rows'][0]['traversability'] == 'REQUIRES_FOOTPRINT_VALIDATION'
    assert not report['eligible_for_execution']
    ctx.sample_step = 1e-12
    with pytest.raises(CoverageError) as error:
        analyze_traversability(ctx)
    assert error.value.code == 'SAMPLE_LIMIT_EXCEEDED'
    ctx.cancelled = lambda: True
    with pytest.raises(CoverageError, match='canceled'):
        analyze_traversability(ctx)


def test_derived_map_roundtrips_without_overwriting_sources(tmp_path):
    ctx = context()
    ctx.grid.data[0, 0] = -1
    ctx.keepouts = [box(.5, .5, .6, .6)]
    original = ctx.grid.data.copy()
    path = write_planning_map(ctx, tmp_path / 'derived')
    derived = load_grid(path)
    assert derived.origin == ctx.grid.origin and derived.resolution == ctx.grid.resolution
    assert derived.data[0, 0] == -1
    assert derived.data[5, 5] == 100
    assert np.array_equal(ctx.grid.data, original)
    with pytest.raises(CoverageError) as error:
        write_planning_map(ctx, path.parent)
    assert error.value.code == 'OUTPUT_EXISTS'
