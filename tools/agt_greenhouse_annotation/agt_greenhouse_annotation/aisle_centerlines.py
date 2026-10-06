"""Geometric aisle centerlines independent from current traversability evidence.

The existing corridor refinement intentionally derives a *safe* centerline from
Ground/obstacle-filtered cells.  For review and semantic authoring we also need
a stable geometric centerline that answers a different question: where is the
agricultural aisle according to the paired row geometry?  This module derives
that line only from the validated aisle geometric envelope, so a local Ground
hole or occupied patch does not fragment the structural representation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class GeometricAislePair:
    pair_index: int
    pair_kind: str
    geometric_cell_count: int
    centerline_cell_count: int


@dataclass(frozen=True)
class GeometricAisleCenterlineResult:
    mask: np.ndarray
    pairs: tuple[GeometricAislePair, ...]
    expected_interior_aisles: int

    @property
    def geometric_aisle_count(self) -> int:
        return sum(pair.geometric_cell_count > 0 for pair in self.pairs)

    @property
    def interior_geometric_aisle_count(self) -> int:
        return sum(
            pair.pair_kind == "ROW_ROW" and pair.geometric_cell_count > 0
            for pair in self.pairs
        )

    @property
    def geometric_centerline_count(self) -> int:
        return sum(pair.centerline_cell_count > 0 for pair in self.pairs)


def _grid_xy(navigation: Any) -> tuple[np.ndarray, np.ndarray]:
    rows, cols = np.indices(navigation.occupancy.shape, dtype=np.float64)
    xx = float(navigation.origin_x_m) + (
        cols + 0.5
    ) * float(navigation.resolution_m)
    yy = float(navigation.origin_y_m) + (
        rows + 0.5
    ) * float(navigation.resolution_m)
    return xx, yy


def derive_geometric_aisle_centerlines(
    navigation: Any,
    structure: Any,
    corridor: Any,
) -> GeometricAisleCenterlineResult:
    """Trace one continuous structural center band for every valid aisle geometry.

    This is deliberately *not* a navigation-safe path.  It ignores Ground FREE,
    local sensor obstacles, slope and step evidence after the aisle geometry has
    been accepted.  Those constraints continue to define the existing safe
    ``aisle_centerline`` and the formal Navigation Map.
    """

    shape = np.asarray(navigation.occupancy).shape
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    if geometric.shape != shape:
        raise ValueError("geometric aisle centerline grid shape mismatch")

    direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("geometric aisle centerline row direction must be finite and non-zero")
    direction /= norm
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    xx, yy = _grid_xy(navigation)
    vv = xx * perpendicular[0] + yy * perpendicular[1]

    output = np.zeros(shape, dtype=bool)
    pair_results: list[GeometricAislePair] = []
    half_width = max(
        float(getattr(corridor.config, "aisle_centerline_half_width_m", 0.06)),
        0.75 * float(navigation.resolution_m),
    )

    for diagnostic in corridor.aisle_pair_diagnostics:
        if int(getattr(diagnostic, "geometric_cell_count", 0)) <= 0:
            pair_results.append(
                GeometricAislePair(
                    pair_index=int(diagnostic.pair_index),
                    pair_kind=str(diagnostic.pair_kind),
                    geometric_cell_count=0,
                    centerline_cell_count=0,
                )
            )
            continue

        low = min(
            float(diagnostic.left_row_center_v_m),
            float(diagnostic.right_row_center_v_m),
        )
        high = max(
            float(diagnostic.left_row_center_v_m),
            float(diagnostic.right_row_center_v_m),
        )
        owned = geometric & (vv >= low - 1.0e-9) & (vv <= high + 1.0e-9)
        geometric_count = int(np.count_nonzero(owned))
        if geometric_count == 0:
            pair_results.append(
                GeometricAislePair(
                    pair_index=int(diagnostic.pair_index),
                    pair_kind=str(diagnostic.pair_kind),
                    geometric_cell_count=0,
                    centerline_cell_count=0,
                )
            )
            continue

        corridor_low = float(np.min(vv[owned]))
        corridor_high = float(np.max(vv[owned]))
        midpoint = 0.5 * (corridor_low + corridor_high)
        line = owned & (np.abs(vv - midpoint) <= half_width)
        output |= line
        pair_results.append(
            GeometricAislePair(
                pair_index=int(diagnostic.pair_index),
                pair_kind=str(diagnostic.pair_kind),
                geometric_cell_count=geometric_count,
                centerline_cell_count=int(np.count_nonzero(line)),
            )
        )

    expected_interior = max(0, len(corridor.accepted_row_centers_v_m) - 1)
    return GeometricAisleCenterlineResult(
        mask=output,
        pairs=tuple(pair_results),
        expected_interior_aisles=expected_interior,
    )
