"""Dependency-light rotatable 3D review view for AGT Map Workbench.

The 2D canvas remains the authoring authority.  This module is review-only: it
renders a deterministic PCD display sample plus offline navigation evidence in
one shared XYZ frame.  It intentionally avoids OpenGL/VTK/Open3D dependencies
so the Workbench stays deployable in the current ROS 2 Humble environment.

The selected 6/15/30 万点 setting is the stationary review density. During
mouse orbit/pan the renderer intentionally switches to a lightweight preview:
the cloud is capped and dense evidence layers are deterministically decimated.
The full selected review density is restored as soon as the drag ends.
"""

from __future__ import annotations

import math

import numpy as np

from .spatial_regions import points_in_polygon
from PyQt5.QtCore import QPointF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QSlider,
    QWidget,
)


_LAYER_COLORS = {
    "selected_reference": QColor(255, 255, 255, 255),
    "ground": QColor(80, 150, 235, 150),
    "row_centerline": QColor(255, 220, 40, 235),
    "row_structural_band": QColor(235, 80, 185, 120),
    "refined_aisle": QColor(40, 205, 225, 95),
    "geometric_centerline": QColor(65, 155, 255, 245),
    "safe_centerline": QColor(255, 110, 60, 245),
    "aisle_centerline": QColor(80, 245, 140, 245),
    "vehicle_corridor": QColor(255, 165, 40, 115),
}

_INTERACTIVE_CLOUD_LIMIT = 20_000
_INTERACTIVE_DENSE_LAYER_LIMIT = 4_000
_DENSE_LAYERS = {
    "ground",
    "row_structural_band",
    "refined_aisle",
    "vehicle_corridor",
}


def deterministic_display_sample(points: np.ndarray, maximum: int) -> np.ndarray:
    """Return an evenly spaced point sample for rendering only."""
    values = np.asarray(points)
    limit = int(maximum)
    if limit <= 0:
        raise ValueError("display sample limit must be > 0")
    if values.shape[0] <= limit:
        return values.copy()
    return values[np.linspace(0, values.shape[0] - 1, limit, dtype=np.int64)].copy()


class Review3DCanvas(QWidget):
    """Orthographic 3D point reviewer with mouse orbit/pan/zoom."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(480, 360)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self._points = np.empty((0, 3), dtype=np.float64)
        self._intensity = np.empty(0, dtype=np.float64)
        self._has_intensity = False
        self._height_range = None
        self._review_region = None
        self._layers: dict[str, np.ndarray] = {}
        self._layer_visibility = {name: True for name in _LAYER_COLORS}
        self._layer_opacity = 1.0
        self._layer_visibility["ground"] = False
        self._layer_visibility["row_structural_band"] = False
        self._layer_visibility["refined_aisle"] = False
        self._center = np.zeros(3, dtype=np.float64)
        self._yaw = math.radians(-35.0)
        self._pitch = math.radians(48.0)
        self._scale = 18.0
        self._zoom = 1.0
        self._pan = np.zeros(2, dtype=np.float64)
        self._point_size_px = 1.0
        self._color_mode = "height"
        self.selection_label = ''
        self._last_pos = None
        self._drag_button = Qt.NoButton
        self._dragging = False

    # --------------------------------------------------------------- data
    def set_cloud(self, cloud, *, sample_limit: int = 150_000) -> None:
        sample_limit = int(sample_limit)
        if sample_limit <= 0:
            raise ValueError("sample_limit must be > 0")
        point_data = cloud if isinstance(cloud, np.ndarray) else cloud.points
        count = int(point_data.shape[0])
        if count == 0:
            self._points = np.empty((0, 3), dtype=np.float64)
            self._intensity = np.empty(0, dtype=np.float64)
            self._has_intensity = False
            self.update()
            return
        points = deterministic_display_sample(point_data, sample_limit)
        if isinstance(cloud, np.ndarray):
            xyz = np.asarray(points[:, :3], dtype=np.float64)
            finite = np.all(np.isfinite(xyz), axis=1)
            self._points = xyz[finite]
            if cloud.ndim == 2 and cloud.shape[1] > 3:
                self._intensity = np.asarray(points[:, 3], dtype=np.float64)[finite]
                self._has_intensity = True
            else:
                self._intensity = np.empty(self._points.shape[0], dtype=np.float64)
                self._has_intensity = False
        else:
            xyz = np.column_stack(
                (
                    np.asarray(points["x"], dtype=np.float64),
                    np.asarray(points["y"], dtype=np.float64),
                    np.asarray(points["z"], dtype=np.float64),
                )
            )
            finite = np.all(np.isfinite(xyz), axis=1)
            self._points = xyz[finite]
            if "intensity" in cloud.schema.fields:
                index = cloud.schema.fields.index("intensity")
                if int(cloud.schema.counts[index]) == 1:
                    values = np.asarray(points["intensity"], dtype=np.float64)
                    self._intensity = values[finite]
                    self._has_intensity = True
                else:
                    self._intensity = np.empty(self._points.shape[0], dtype=np.float64)
                    self._has_intensity = False
            else:
                self._intensity = np.empty(self._points.shape[0], dtype=np.float64)
                self._has_intensity = False
        if self._points.shape[0]:
            minimum = np.min(self._points, axis=0)
            maximum = np.max(self._points, axis=0)
            self._center = 0.5 * (minimum + maximum)
        self._zoom = 1.0
        self._pan[:] = 0.0
        self.fit_to_data()
        self.update()

    def set_layer_xyz(self, name: str, xyz: np.ndarray | None) -> None:
        if name not in _LAYER_COLORS:
            raise ValueError(f"unknown 3D review layer: {name}")
        if xyz is None:
            self._layers.pop(name, None)
        else:
            values = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
            values = values[np.all(np.isfinite(values), axis=1)]
            self._layers[name] = values
        self.update()

    def clear_analysis(self) -> None:
        self._layers.clear()
        self.update()

    def set_layer_opacity(self, opacity: float) -> None:
        value = float(opacity)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError("layer opacity must be in [0, 1]")
        self._layer_opacity = value
        self.update()

    def set_layer_visible(self, name: str, visible: bool) -> None:
        self._layer_visibility[name] = bool(visible)
        self.update()

    def set_display_options(self, *, color_mode: str, point_size_px: float) -> None:
        if color_mode not in {"height", "intensity", "mono"}:
            raise ValueError(f"unsupported 3D color mode: {color_mode}")
        self._color_mode = str(color_mode)
        self._point_size_px = max(1.0, min(5.0, float(point_size_px)))
        self.update()

    def set_height_filter(self, bounds=None) -> None:
        self._height_range = bounds
        self.update()

    def set_review_region(self, polygon=None):
        self._review_region = polygon
        self.fit_to_data()
        self.update()

    def visible_cloud(self):
        mask = np.ones(len(self._points), dtype=bool)
        if self._height_range is not None:
            low, high = self._height_range
            mask &= (self._points[:, 2] >= low) & (self._points[:, 2] <= high)
        if self._review_region is not None:
            mask &= points_in_polygon(self._points[:, :2], self._review_region)
        return self._points[mask], self._intensity[mask]

    def sample_count(self) -> int:
        return len(self.visible_cloud()[0])

    # --------------------------------------------------------------- camera
    def _rotation(self) -> np.ndarray:
        cy, sy = math.cos(self._yaw), math.sin(self._yaw)
        cp, sp = math.cos(self._pitch), math.sin(self._pitch)
        rz = np.array(
            [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        rx = np.array(
            [[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]],
            dtype=np.float64,
        )
        return rx @ rz

    def set_preset(self, preset: str) -> None:
        if preset == "top":
            self._yaw = 0.0
            self._pitch = 0.0
        elif preset == "front":
            self._yaw = 0.0
            self._pitch = math.radians(90.0)
        elif preset == "side":
            self._yaw = math.radians(90.0)
            self._pitch = math.radians(90.0)
        elif preset == "iso":
            self._yaw = math.radians(-35.0)
            self._pitch = math.radians(48.0)
        else:
            raise ValueError(f"unknown 3D view preset: {preset}")
        self._pan[:] = 0.0
        self._zoom = 1.0
        self.fit_to_data()
        self.update()

    def fit_to_data(self) -> None:
        points, _ = self.visible_cloud()
        if points.shape[0] == 0 or self.width() <= 20 or self.height() <= 20:
            return
        self._center = 0.5 * (points.min(axis=0) + points.max(axis=0))
        rotated = (points - self._center) @ self._rotation().T
        span = np.ptp(rotated[:, :2], axis=0)
        span = np.maximum(span, 1e-3)
        self._scale = 0.86 * min(
            float(self.width()) / float(span[0]),
            float(self.height()) / float(span[1]),
        )
        self._zoom = 1.0
        self._pan[:] = 0.0
        self.update()

    def _project(self, xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if xyz.shape[0] == 0:
            return np.empty((0, 2)), np.empty(0)
        rotated = (xyz - self._center) @ self._rotation().T
        scale = float(self._scale * self._zoom)
        xy = np.empty((xyz.shape[0], 2), dtype=np.float64)
        xy[:, 0] = rotated[:, 0] * scale + 0.5 * self.width() + self._pan[0]
        xy[:, 1] = -rotated[:, 1] * scale + 0.5 * self.height() + self._pan[1]
        return xy, rotated[:, 2]

    # --------------------------------------------------------------- drawing
    @staticmethod
    def _bucket(values: np.ndarray) -> np.ndarray:
        finite = np.isfinite(values)
        result = np.zeros(values.shape[0], dtype=np.int32)
        if not np.any(finite):
            return result
        valid = values[finite]
        low = float(np.min(valid))
        high = float(np.max(valid))
        scale = max(1e-9, high - low)
        result[finite] = np.clip(((valid - low) / scale * 15.0).astype(np.int32), 0, 15)
        return result

    @staticmethod
    def _cap_deterministic(values: np.ndarray, limit: int) -> np.ndarray:
        if values.shape[0] <= limit:
            return values
        stride = int(math.ceil(values.shape[0] / float(limit)))
        return values[::stride]

    def _draw_screen_points(
        self,
        painter: QPainter,
        xy: np.ndarray,
        color: QColor,
        *,
        width_px: float,
    ) -> None:
        if xy.shape[0] == 0:
            return
        pen = QPen(color)
        pen.setCosmetic(True)
        pen.setWidthF(float(width_px))
        painter.setPen(pen)
        painter.drawPoints(QPolygonF([QPointF(float(x), float(y)) for x, y in xy]))

    def _draw_cloud(self, painter: QPainter) -> None:
        points, intensity = self.visible_cloud()
        if self._dragging and points.shape[0] > _INTERACTIVE_CLOUD_LIMIT:
            stride = int(math.ceil(points.shape[0] / float(_INTERACTIVE_CLOUD_LIMIT)))
            points = points[::stride]
            if intensity.shape[0] >= points.shape[0]:
                intensity = intensity[::stride]
        xy, _ = self._project(points)
        inside = (
            (xy[:, 0] >= -2)
            & (xy[:, 0] <= self.width() + 2)
            & (xy[:, 1] >= -2)
            & (xy[:, 1] <= self.height() + 2)
        )
        if not np.any(inside):
            return
        xy = xy[inside]
        points = points[inside]

        # Mouse interaction prioritizes frame rate over stationary color detail.
        # The selected height/intensity rendering returns immediately on release.
        if self._dragging:
            self._draw_screen_points(
                painter,
                xy,
                QColor(120, 185, 205, 150),
                width_px=max(1.0, self._point_size_px),
            )
            return

        if self._color_mode == "mono":
            self._draw_screen_points(
                painter, xy, QColor(205, 215, 225, 185), width_px=self._point_size_px
            )
            return
        if self._color_mode == "intensity" and self._has_intensity:
            values = intensity[inside]
            bucket = self._bucket(values)
            for index in range(16):
                selected = xy[bucket == index]
                if selected.shape[0] == 0:
                    continue
                gray = int(45 + 210 * index / 15.0)
                self._draw_screen_points(
                    painter,
                    selected,
                    QColor(gray, gray, gray, 190),
                    width_px=self._point_size_px,
                )
            return
        bucket = self._bucket(points[:, 2])
        for index in range(16):
            selected = xy[bucket == index]
            if selected.shape[0] == 0:
                continue
            t = index / 15.0
            color = QColor.fromHsvF((2.0 / 3.0) * (1.0 - t), 0.88, 0.95, 0.78)
            self._draw_screen_points(
                painter, selected, color, width_px=self._point_size_px
            )

    def _draw_layers(self, painter: QPainter) -> None:
        for name in (
            "ground",
            "row_structural_band",
            "refined_aisle",
            "vehicle_corridor",
            "row_centerline",
            "geometric_centerline",
            "safe_centerline",
            "aisle_centerline",
            "selected_reference",
        ):
            if not self._layer_visibility.get(name, True):
                continue
            xyz = self._layers.get(name)
            if xyz is None or xyz.shape[0] == 0:
                continue
            if self._review_region is not None:
                xyz = xyz[points_in_polygon(xyz[:, :2], self._review_region)]
            if self._dragging and name in _DENSE_LAYERS:
                xyz = self._cap_deterministic(xyz, _INTERACTIVE_DENSE_LAYER_LIMIT)
            xy, _ = self._project(xyz)
            if name == 'selected_reference':
                width = 5.0
            elif name in {"row_centerline", "geometric_centerline", "safe_centerline", "aisle_centerline"}:
                width = 2.4
            elif name == "vehicle_corridor":
                width = 2.0
            else:
                width = 1.5
            color = QColor(_LAYER_COLORS[name])
            if name != 'selected_reference':
                color.setAlphaF(color.alphaF() * self._layer_opacity)
            self._draw_screen_points(painter, xy, color, width_px=width)

    def _draw_axis_gizmo(self, painter: QPainter) -> None:
        origin = np.array([64.0, float(self.height() - 62)], dtype=np.float64)
        rotation = self._rotation()
        axes = (
            ("X", np.array([1.0, 0.0, 0.0]), QColor(245, 75, 75)),
            ("Y", np.array([0.0, 1.0, 0.0]), QColor(70, 235, 120)),
            ("Z", np.array([0.0, 0.0, 1.0]), QColor(80, 150, 255)),
        )
        for label, axis, color in axes:
            projected = rotation @ axis
            endpoint = origin + np.array([projected[0], -projected[1]]) * 32.0
            pen = QPen(color)
            pen.setWidth(2)
            painter.setPen(pen)
            painter.drawLine(
                QPointF(float(origin[0]), float(origin[1])),
                QPointF(float(endpoint[0]), float(endpoint[1])),
            )
            painter.drawText(
                QPointF(float(endpoint[0] + 3), float(endpoint[1] - 3)), label
            )

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        del event
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(0, 0, 0))
        painter.setRenderHint(QPainter.Antialiasing, False)
        self._draw_cloud(painter)
        self._draw_layers(painter)
        painter.setRenderHint(QPainter.Antialiasing, True)
        self._draw_axis_gizmo(painter)
        painter.setPen(QColor(220, 225, 230))
        mode = "轻量交互预览" if self._dragging else "完整审查采样"
        painter.drawText(
            12,
            22,
            f"左键旋转 | 右键平移 | 滚轮缩放 | 双击适配 | {mode}",
        )
        if self.selection_label:
            painter.setPen(QColor(255, 255, 255))
            painter.drawText(12, 44, self.selection_label)
        painter.end()

    # --------------------------------------------------------------- events
    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() in {Qt.LeftButton, Qt.RightButton, Qt.MiddleButton}:
            self._last_pos = event.pos()
            self._drag_button = event.button()
            self._dragging = True
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt API
        if self._last_pos is None or self._drag_button == Qt.NoButton:
            super().mouseMoveEvent(event)
            return
        delta = event.pos() - self._last_pos
        self._last_pos = event.pos()
        if self._drag_button == Qt.LeftButton:
            self._yaw += math.radians(float(delta.x()) * 0.35)
            self._pitch += math.radians(float(delta.y()) * 0.35)
            self._pitch = max(math.radians(-89.0), min(math.radians(89.0), self._pitch))
        else:
            self._pan += np.array([float(delta.x()), float(delta.y())])
        self.update()
        event.accept()

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 - Qt API
        self._last_pos = None
        self._drag_button = Qt.NoButton
        self._dragging = False
        self.update()
        event.accept()

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt API
        factor = 1.12 if event.angleDelta().y() > 0 else 1.0 / 1.12
        self._zoom = max(0.05, min(80.0, self._zoom * factor))
        self.update()
        event.accept()

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.fit_to_data()
        self.update()
        event.accept()

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt API
        super().resizeEvent(event)
        if self._points.shape[0] and self._scale <= 0.0:
            self.fit_to_data()


class ThreeDReviewWidget(QWidget):
    """Review-only controls around :class:`Review3DCanvas`."""

    vehicleProfileChanged = pyqtSignal(float, float)
    sampleLimitChanged = pyqtSignal(int)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.canvas = Review3DCanvas(self)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        top = QHBoxLayout()
        for label, preset in (
            ("顶视", "top"),
            ("前视", "front"),
            ("侧视", "side"),
            ("等轴测", "iso"),
        ):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, p=preset: self.canvas.set_preset(p))
            top.addWidget(button)
        fit = QPushButton("适配")
        fit.clicked.connect(self.canvas.fit_to_data)
        top.addWidget(fit)

        self.sample_limit = QComboBox()
        for label, value in (
            ("3D 6 万点", 60_000),
            ("3D 15 万点", 150_000),
            ("3D 30 万点", 300_000),
        ):
            self.sample_limit.addItem(label, value)
        self.sample_limit.setCurrentIndex(1)
        self.sample_limit.currentIndexChanged.connect(
            lambda: self.sampleLimitChanged.emit(int(self.sample_limit.currentData()))
        )
        top.addWidget(self.sample_limit)

        self.color_mode = QComboBox()
        self.color_mode.addItem("高度着色", "height")
        self.color_mode.addItem("强度", "intensity")
        self.color_mode.addItem("单色", "mono")
        self.color_mode.currentIndexChanged.connect(self._update_display_options)
        top.addWidget(self.color_mode)

        self.point_size = QComboBox()
        for value in (1, 2, 3):
            self.point_size.addItem(f"{value}px", float(value))
        self.point_size.currentIndexChanged.connect(self._update_display_options)
        self.point_size.hide()

        self.layer_opacity = QSlider(Qt.Horizontal)
        self.layer_opacity.setRange(5, 100)
        self.layer_opacity.setValue(70)
        self.layer_opacity.valueChanged.connect(
            lambda value: self.canvas.set_layer_opacity(float(value) / 100.0)
        )
        self.layer_opacity.hide()
        layout.addLayout(top)

        advanced = QGroupBox('更多三维显示选项')
        advanced.setCheckable(True)
        advanced.setChecked(False)
        advanced_layout = QVBoxLayout(advanced)
        advanced_content = QWidget()
        content_layout = QVBoxLayout(advanced_content)
        advanced_layout.addWidget(advanced_content)
        advanced_content.hide()
        advanced.toggled.connect(advanced_content.setVisible)
        layers = QHBoxLayout()
        self.layer_checks: dict[str, QCheckBox] = {}
        for name, label, checked in (
            ("ground", "地面", False),
            ("row_centerline", "垄中心", True),
            ("row_structural_band", "垄结构带", False),
            ("refined_aisle", "精炼行道", False),
            ("geometric_centerline", "几何中心线", True),
            ("safe_centerline", "安全中心线", False),
            ("aisle_centerline", "安全行道带", False),
            ("vehicle_corridor", "车辆通行廊道", False),
        ):
            box = QCheckBox(label)
            box.setChecked(checked)
            self.canvas.set_layer_visible(name, checked)
            box.toggled.connect(
                lambda state, layer=name: self.canvas.set_layer_visible(layer, state)
            )
            self.layer_checks[name] = box
            layers.addWidget(box)
        layers.addStretch(1)
        content_layout.addLayout(layers)

        vehicle = QHBoxLayout()
        vehicle.addWidget(QLabel("车辆通行廊道（仅供审查，不修改最终 PGM）："))
        self.vehicle_width = QDoubleSpinBox()
        self.vehicle_width.setPrefix("车宽 ")
        self.vehicle_width.setSuffix(" 米")
        self.vehicle_width.setDecimals(2)
        self.vehicle_width.setRange(0.20, 3.00)
        self.vehicle_width.setSingleStep(0.05)
        self.vehicle_width.setValue(0.60)
        self.vehicle_margin = QDoubleSpinBox()
        self.vehicle_margin.setPrefix("单侧安全余量 ")
        self.vehicle_margin.setSuffix(" 米")
        self.vehicle_margin.setDecimals(2)
        self.vehicle_margin.setRange(0.0, 1.0)
        self.vehicle_margin.setSingleStep(0.02)
        self.vehicle_margin.setValue(0.10)
        for box in (self.vehicle_width, self.vehicle_margin):
            box.valueChanged.connect(self._emit_vehicle_profile)
        vehicle.addWidget(self.vehicle_width)
        vehicle.addWidget(self.vehicle_margin)
        self.vehicle_status = QLabel("要求宽度：0.80 米")
        vehicle.addWidget(self.vehicle_status)
        vehicle.addStretch(1)
        content_layout.addLayout(vehicle)
        layout.addWidget(advanced)

        self.status = QLabel(
            "三维审查：等待点云。二维视图仍是编辑权威；三维视图用于旋转查看空间几何和路线包络。"
        )
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addWidget(self.canvas, 1)

    def _update_display_options(self) -> None:
        self.canvas.set_display_options(
            color_mode=str(self.color_mode.currentData()),
            point_size_px=float(self.point_size.currentData()),
        )

    def _emit_vehicle_profile(self) -> None:
        width = float(self.vehicle_width.value())
        margin = float(self.vehicle_margin.value())
        self.vehicle_status.setText(f"要求宽度：{width + 2.0 * margin:.2f} 米")
        self.vehicleProfileChanged.emit(width, margin)

    def set_cloud(self, cloud) -> None:
        self.canvas.set_cloud(cloud, sample_limit=int(self.sample_limit.currentData()))
        self._update_display_options()
        self.status.setText(
            "左键旋转、右键平移、滚轮缩放；白色线为选中对象。"
        )

    def reload_cloud_sample(self, cloud) -> None:
        if cloud is None:
            return
        self.set_cloud(cloud)

    @staticmethod
    def _grid_xyz(navigation, mask: np.ndarray, *, z_offset_m: float) -> np.ndarray:
        mask = np.asarray(mask, dtype=bool)
        valid = mask & np.asarray(navigation.ground_valid, dtype=bool)
        rows, columns = np.nonzero(valid)
        if rows.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        x = navigation.origin_x_m + (columns.astype(np.float64) + 0.5) * navigation.resolution_m
        y = navigation.origin_y_m + (rows.astype(np.float64) + 0.5) * navigation.resolution_m
        z = np.asarray(navigation.ground_height_m, dtype=np.float64)[rows, columns] + float(z_offset_m)
        return np.column_stack((x, y, z))

    def set_analysis(self, navigation, corridor=None, geometric=None, vehicle=None) -> None:
        self.canvas.clear_analysis()
        if navigation is None:
            return
        ground_valid = np.asarray(navigation.ground_valid, dtype=bool)
        stride = max(1, int(math.ceil(math.sqrt(np.count_nonzero(ground_valid) / 70_000.0))))
        ground_mask = np.zeros_like(ground_valid)
        ground_mask[::stride, ::stride] = ground_valid[::stride, ::stride]
        self.canvas.set_layer_xyz(
            "ground", self._grid_xyz(navigation, ground_mask, z_offset_m=0.0)
        )
        if corridor is not None:
            self.canvas.set_layer_xyz(
                "row_centerline",
                self._grid_xyz(navigation, corridor.row_centerline, z_offset_m=0.06),
            )
            self.canvas.set_layer_xyz(
                "row_structural_band",
                self._grid_xyz(navigation, corridor.row_structural_band, z_offset_m=0.035),
            )
            self.canvas.set_layer_xyz(
                "refined_aisle",
                self._grid_xyz(navigation, corridor.aisle_candidate, z_offset_m=0.02),
            )
            self.canvas.set_layer_xyz(
                "aisle_centerline",
                self._grid_xyz(navigation, corridor.aisle_centerline, z_offset_m=0.075),
            )
            self.canvas.set_layer_xyz(
                "safe_centerline",
                self._grid_xyz(navigation, corridor.aisle_centerline, z_offset_m=0.085),
            )
        if geometric is not None:
            self.canvas.set_layer_xyz(
                "geometric_centerline",
                self._grid_xyz(navigation, geometric.mask, z_offset_m=0.07),
            )
        if vehicle is not None:
            self.canvas.set_layer_xyz(
                "vehicle_corridor",
                self._grid_xyz(navigation, vehicle.corridor_mask, z_offset_m=0.10),
            )
            self.vehicle_status.setText(
                f"要求宽度：{vehicle.required_width_m:.2f} 米 | "
                f"审查廊道：{vehicle.corridor_cells:,} 个网格"
            )
        self.canvas.update()
