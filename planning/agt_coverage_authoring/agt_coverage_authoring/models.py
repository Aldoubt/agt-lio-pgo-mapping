"""Internal authoring types, not a replacement for Site/Route/Profile contracts."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np


class CoverageError(ValueError):
    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class PathPoint:
    """Rear axle position and body yaw; gear is explicit, never inferred from yaw."""
    x: float
    y: float
    yaw: float
    direction: str = "F"
    v_ref: float = 0.2
    segment_id: str = "connection_0"
    kind: str = "CONNECTION"
    semantic_ref: str = ""
    event_ref: str = ""
    curvature: float = 0.0
    clearance: float = 0.0
    time_from_start: float | None = None


@dataclass
class GridMap:
    """ROS trinary cells: -1 unknown, 0 free, 100 occupied; row 0 is bottom."""
    data: np.ndarray
    resolution: float
    origin: tuple[float, float, float]
    yaml_path: str = ""
    image_path: str = ""


@dataclass(frozen=True)
class RobotProfile:
    platform_id: str
    digest: str
    wheelbase: float
    min_turning_radius: float
    footprint: tuple[tuple[float, float], ...]
    rear_axle_to_base: tuple[float, float] = (0.0, 0.0)
    geometry_verified: bool = False
    reference_verified: bool = False
    limits_verified: bool = False
    max_forward_speed: float = 0.2
    max_reverse_speed: float = 0.1
    max_steering_angle: float | None = None
    max_steering_rate: float | None = None
    max_acceleration: float | None = None
    max_deceleration: float | None = None
    path_reference: str = "rear_axle_center"


@dataclass(frozen=True)
class WorkRow:
    id: str
    centerline: tuple[tuple[float, float], ...]
    corridor: Any = None
    allowed_directions: tuple[str, ...] = ("along", "against")


@dataclass
class PlanningContext:
    grid: GridMap
    robot: RobotProfile
    boundary: Any
    rows: list[WorkRow]
    work_area: Any
    keepouts: list[Any]
    start: Pose
    end: Pose
    mode: str = "rows"
    backend: str = "ros"
    operation_width: float | None = None
    allow_reverse: bool = True
    heading: float | None = None
    clearance_margin: float = 0.0
    sample_step: float = 0.05
    connection_timeout: float = 15.0
    planning_timeout: float = 120.0
    input_files: dict[str, str] = field(default_factory=dict)
    map_binding: dict[str, Any] = field(default_factory=dict)
    request_path: str = ""
    profile_path: str = ""
    site_root: str = ""
    structure_path: str = ""
    backend_options: dict[str, Any] = field(default_factory=dict)
    cancelled: Callable[[], bool] = field(default=lambda: False, repr=False)
    headlands: list[Any] = field(default_factory=list)

    def check_cancelled(self) -> None:
        if self.cancelled():
            raise CoverageError("CANCELED", "Authoring job canceled")


@dataclass
class PlanResult:
    points: list[PathPoint]
    complete: bool
    report: dict[str, Any]
    missing: list[dict[str, Any]] = field(default_factory=list)
    provenance: dict[str, Any] = field(default_factory=dict)
