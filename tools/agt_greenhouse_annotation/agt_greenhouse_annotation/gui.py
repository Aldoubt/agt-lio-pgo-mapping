"""PyQt5 top-down manual row/headland editor for existing map packages."""
import argparse
import copy
from dataclasses import replace
from types import SimpleNamespace
from .reference_review import load_reference_draft, simplify_reference_line
from .reference_editor import ReferenceEditorMixin
import json
import math
from pathlib import Path
import sys
import time

import yaml

import numpy as np
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QLibraryInfo, QLocale, QTranslator
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

from .topology import (align_backend, label_pose, load_map_package, load_topology,
                       new_topology, now_iso, save_topology, sha256_file)
from .validator import validate_topology
from .point_cloud import read_pcd_cloud
from .review_3d import ThreeDReviewWidget
from .spatial_regions import points_in_polygon, validate_polygon
from .structure_artifacts import (
    GlobalRowProposal, AisleProposal, GreenhouseStructureConfig, analyze_greenhouse_structure,
    decision_for_proposal, freeze_accepted_proposals, make_boundary_aisle_proposal,
    merge_row_proposals, proposal_aisles_for_rows, save_proposal_revision, save_review_revision,
    split_row_proposal,
)
from .workbench_panel import EvidenceLayerPanel, StructureProposalPanel


def _localize_dialog_buttons(buttons):
    buttons.button(QtWidgets.QDialogButtonBox.Ok).setText('确定')
    buttons.button(QtWidgets.QDialogButtonBox.Cancel).setText('取消')


class CloudLoader(QtCore.QThread):
    loaded = QtCore.pyqtSignal(object, int)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, path, voxel, maximum):
        super().__init__()
        self.path, self.voxel, self.maximum = path, voxel, maximum

    def run(self):
        try:
            import open3d as o3d
            cloud = o3d.io.read_point_cloud(str(self.path))
            total = len(cloud.points)
            if not total:
                raise ValueError('map.pcd contains no readable points')
            cloud = cloud.voxel_down_sample(self.voxel)
            points = np.asarray(cloud.points)
            # Deterministic cap after spatial voxel reduction, never draw millions.
            if len(points) > self.maximum:
                points = points[np.linspace(0, len(points)-1, self.maximum, dtype=int)]
            self.loaded.emit(points.copy(), total)
        except Exception as exc:
            self.failed.emit(str(exc))


class StructureAnalyzer(QtCore.QThread):
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, package, mode, config_path, parent=None, row_exclusions=None):
        super().__init__(parent)
        self.package, self.mode, self.config_path = package, mode, config_path
        self.row_exclusions = copy.deepcopy(row_exclusions)

    def run(self):
        try:
            mapping = {}
            if self.config_path:
                mapping = yaml.safe_load(Path(self.config_path).read_text(encoding='utf-8')) or {}
            if self.row_exclusions is not None:
                mapping['row_exclusion_polygons_xy'] = self.row_exclusions
            config = GreenhouseStructureConfig.from_mapping(mapping)
            cloud = read_pcd_cloud(self.package.path / 'map.pcd')
            analysis = analyze_greenhouse_structure(
                cloud, self.package.map_package_hash, mode=self.mode, config=config
            )
            self.completed.emit(analysis)
        except Exception as exc:
            self.failed.emit(str(exc))


class GeometryDialog(QtWidgets.QDialog):
    def __init__(self, kind, item, parent=None):
        super().__init__(parent)
        self.setWindowTitle('人工编辑'+('垄线' if kind == 'row' else '地头'))
        self.kind = kind
        form = QtWidgets.QFormLayout(self)
        self.identifier = QtWidgets.QLineEdit(item.get('id', ''))
        self.vertices = QtWidgets.QPlainTextEdit(json.dumps(item.get('centerline' if kind == 'row' else 'polygon', []), indent=2))
        self.vertices.setMinimumSize(350, 180)
        self.width = QtWidgets.QDoubleSpinBox()
        self.width.setRange(.01, 100)
        self.width.setDecimals(3)
        self.width.setValue(item.get('nominal_width_m', 1.0))
        self.direction = QtWidgets.QComboBox()
        for label, value in (('双向通行', 'bidirectional'), ('正向通行', 'forward'),
                             ('反向通行', 'reverse'), ('方向未知', 'unknown')):
            self.direction.addItem(label, value)
        self.direction.setCurrentIndex(self.direction.findData(item.get('direction', 'bidirectional')))
        self.confidence = QtWidgets.QComboBox()
        for label, value in (('已人工确认', 'confirmed'), ('尚未确认', 'unconfirmed')):
            self.confidence.addItem(label, value)
        self.confidence.setCurrentIndex(self.confidence.findData(item.get('confidence', 'confirmed')))
        self.notes = QtWidgets.QPlainTextEdit(item.get('notes', ''))
        self.notes.setMaximumHeight(80)
        form.addRow('实际编号（手动填写）', self.identifier)
        form.addRow('XY 坐标点 [米]', self.vertices)
        if kind == 'row':
            form.addRow('标称宽度 [米]', self.width)
            form.addRow('允许通行方向', self.direction)
        form.addRow('人工确认状态', self.confidence)
        form.addRow('备注', self.notes)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        _localize_dialog_buttons(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def value(self):
        vertices = json.loads(self.vertices.toPlainText())
        item = dict(id=self.identifier.text().strip(), confidence=self.confidence.currentData(),
                    notes=self.notes.toPlainText())
        item['centerline' if self.kind == 'row' else 'polygon'] = vertices
        if self.kind == 'row':
            item.update(nominal_width_m=self.width.value(), direction=self.direction.currentData())
        return item


class SceneDialog(QtWidgets.QDialog):
    def __init__(self, topology, pose, scene=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle('人工场景标记')
        self.pose, self.scene = pose, scene or {}
        form = QtWidgets.QFormLayout(self)
        self.identifier = QtWidgets.QLineEdit(self.scene.get('scene_id', ''))
        self.scene_type = QtWidgets.QComboBox()
        for label, value in (('垄线入口', 'ROW_ENTRY'), ('垄线中段', 'ROW_MIDDLE'),
                             ('垄线末端', 'ROW_END'), ('地头', 'HEADLAND'), ('其他', 'OTHER')):
            self.scene_type.addItem(label, value)
        self.scene_type.setCurrentIndex(self.scene_type.findData(self.scene.get('scene_type', 'ROW_MIDDLE')))
        self.row = QtWidgets.QComboBox()
        self.row.addItem('未知', 'UNKNOWN')
        for row in topology['rows']:
            self.row.addItem(row['id'], row['id'])
        self.row.setCurrentIndex(self.row.findData(self.scene.get('manual_physical_row_id', 'UNKNOWN')))
        self.headland = QtWidgets.QComboBox()
        self.headland.addItem('未知', 'UNKNOWN')
        for item in topology['headlands']:
            self.headland.addItem(item['id'], item['id'])
        self.headland.setCurrentIndex(self.headland.findData(self.scene.get('manual_headland_id', 'UNKNOWN')))
        self.notes = QtWidgets.QLineEdit(self.scene.get('notes', ''))
        label = label_pose(topology, pose['x'], pose['y'], pose['yaw_rad'])
        projected = label['physical_row_id']
        text = (f"关键帧={pose['keyframe']}\n数据包时间戳={pose['timestamp']:.9f}\n"
                f"x/y={pose['x']:.3f}/{pose['y']:.3f} 米，航向角={pose['yaw_deg']:.2f} 度\n"
                f"最近垄线={projected}，沿垄距离={label['along_row_s_m']}，横向距离={label['lateral_d_m']}\n"
                '关联关系由人工明确指定；允许选择“未知”。')
        info = QtWidgets.QLabel(text)
        info.setWordWrap(True)
        form.addRow(info)
        form.addRow('场景编号（手动填写）', self.identifier)
        form.addRow('场景类型', self.scene_type)
        form.addRow('已确认的实际垄线', self.row)
        form.addRow('已确认的地头', self.headland)
        form.addRow('备注', self.notes)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        _localize_dialog_buttons(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def value(self):
        pose = self.pose
        return dict(scene_id=self.identifier.text().strip(), scene_type=self.scene_type.currentData(),
                    keyframe=pose['keyframe'], bag_timestamp=pose['timestamp'],
                    x=pose['x'], y=pose['y'], yaw_rad=pose['yaw_rad'],
                    manual_physical_row_id=self.row.currentData(),
                    manual_headland_id=self.headland.currentData(),
                    notes=self.notes.text(), confirmation='manual')


class ProposalGeometryDialog(QtWidgets.QDialog):
    """Edit AUTO proposal vertices while keeping the AUTO identity as provenance."""

    def __init__(self, proposal, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f'编辑 {proposal.auto_id} 提议折线')
        form = QtWidgets.QFormLayout(self)
        self.vertices = QtWidgets.QPlainTextEdit(json.dumps(proposal.centerline_xy, indent=2))
        self.vertices.setMinimumSize(360, 220)
        self.width = QtWidgets.QDoubleSpinBox()
        self.width.setRange(.01, 100.0)
        self.width.setDecimals(3)
        self.width.setValue(2.0 * proposal.half_width_m)
        form.addRow('自动提议编号（保留原编号）', QtWidgets.QLabel(proposal.auto_id))
        form.addRow('折线顶点 [地图 XY 坐标，米]', self.vertices)
        form.addRow('标称结构宽度 [米]', self.width)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        _localize_dialog_buttons(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def value(self):
        return {'centerline_xy': json.loads(self.vertices.toPlainText()),
                'width_m': self.width.value()}


class Annotator(ReferenceEditorMixin, QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.gui_load_started = time.perf_counter()
        self.gui_load_seconds = None
        self.args = args
        self.package = load_map_package(args.map_package)
        self.other = load_map_package(args.other_map_package) if args.other_map_package else None
        self.output = Path(args.output).expanduser().resolve()
        self.topology = load_topology(self.output) if self.output.exists() else new_topology(self.package, args.max_row_assignment_distance)
        initial = validate_topology(self.topology, self.package)
        if not initial['valid']:
            raise ValueError('; '.join(initial['errors']))
        if self.other and self.other.backend_id not in self.topology.get('backend_transforms', {}):
            if self.topology['annotation']['status'] == 'frozen':
                raise ValueError('Frozen topology lacks requested other backend transform; create a new draft version')
            self.topology['backend_transforms'][self.other.backend_id] = align_backend(self.package, self.other, args.correspondence_max_dt)
        self.history, self.future = [], []
        self.cloud = None
        self.cloud_reference_z = 0.0
        self.review_region = None
        self.row_exclusions_path = self._site_workspace() / 'greenhouse_structure' / 'row_exclusions.yaml'
        exclusion_mapping = {}
        if self.row_exclusions_path.exists():
            exclusion_mapping = yaml.safe_load(self.row_exclusions_path.read_text()) or {}
            if exclusion_mapping.get('map_package_hash') != self.package.map_package_hash:
                raise ValueError('墙壁排除区域与当前地图哈希不一致')
        elif getattr(args, 'structure_config', None):
            exclusion_mapping = yaml.safe_load(Path(args.structure_config).read_text()) or {}
        self.row_exclusion_polygons = list(exclusion_mapping.get('row_exclusion_polygons_xy', []))
        self.wall_exclusions_enabled = bool(exclusion_mapping.get('enabled', False))
        self.workflow_history, self.workflow_future = [], []
        self.reference_drag = None
        self.reference_draft_path = self._site_workspace() / 'greenhouse_structure' / 'reference_draft.yaml'
        for polygon in self.row_exclusion_polygons:
            validate_polygon(polygon)
        self.pending, self.mode = [], 'inspect'
        self.drag, self.drag_before = None, None
        self.dirty = False
        self.setWindowTitle('AGT 温室结构工作台 — '+self.package.backend_id)
        self.resize(1360, 900)
        self.figure = Figure(figsize=(10, 7), facecolor='#000000')
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.axes = self.figure.add_subplot(111)
        self.review3d = ThreeDReviewWidget(self)
        self.analysis = None
        self.analysis_worker = None
        self.rows_by_source = {'GLOBAL_PROFILE': [], 'LOCAL_TRACKS': []}
        self.original_rows_by_source = {'GLOBAL_PROFILE': [], 'LOCAL_TRACKS': []}
        self.proposal_history = []
        self.proposal_future = []
        self.row_proposals = []
        self.aisle_proposals = []
        self.manual_boundary_aisles = []
        self.active_proposal_source = 'GLOBAL_PROFILE'
        self.structure_review_paths = []
        self.cloud_visible = True
        self.trajectory_visible = False
        self.proposals_visible = True
        self.global_rows_visible = True
        self.local_rows_visible = True
        self.aisles_visible = True
        self.physical_topology_visible = True
        self.physical_rows_visible = True
        self.headlands_visible = True
        self.keyframes_visible = False
        self.evidence_layer_visible = {
            'ground': False, 'ground_confidence': False, 'slope': False,
            'plane_residual': False, 'terrain_ridge': False,
            'terrain_depression': False, 'terrain_step': False,
            'global_row_support': False, 'local_row_observations': False,
            'row_structural_band': False, 'aisle_geometry': False,
            'geometric_centerline': False, 'safe_aisle': False,
            'safe_centerline': False,
        }
        self.overlay_opacity = 0.80
        self.selected_proposal_indices = set()
        self.selected_aisle_index = -1
        self._selection_kind = 'row'
        self._refreshing_proposals = False
        self.list = QtWidgets.QListWidget()
        self.list.currentRowChanged.connect(self.select_item)
        self.info = QtWidgets.QLabel('正在后台加载降采样地图…')
        self.info.setWordWrap(True)
        self.info.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.help = QtWidgets.QLabel('绘制垄线/地头：左键添加顶点，右键或回车完成。\n'
            '标记场景：点击轨迹。编辑：拖动选中的顶点，或点击“编辑详情”。\n'
            '实际编号必须由人工填写并确认。\n'
            '坐标依据：人工拓扑与同次建图前端；非绝对真值。')
        self.help.setWordWrap(True)
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(['查看', '绘制垄线中心线', '绘制地头边界', '添加人工场景标记', '编辑顶点', '设置边界通道锚点', '框选局部审查（两点）', '圈出墙壁 / 非垄区域', '新增参考垄', '新增参考中心线'])
        self.mode_combo.currentIndexChanged.connect(self.change_mode)
        self.distance = QtWidgets.QDoubleSpinBox()
        self.distance.setRange(.01, 100)
        self.distance.setValue(self.topology['assignment']['max_row_assignment_distance_m'])
        self.distance.valueChanged.connect(self.assignment_changed)
        self.mode_combo.hide()
        self.list.hide()
        self.help.hide()
        self.distance.hide()
        self.info.hide()
        side = QtWidgets.QWidget()
        side_layout = QtWidgets.QVBoxLayout(side)
        self.structure_panel = StructureProposalPanel(self)
        self.structure_panel.analysisRequested.connect(self.run_structure_analysis)
        self.structure_panel.sourceChanged.connect(self.select_proposal_source)
        self.structure_panel.rowSelectionChanged.connect(self.set_selected_proposals)
        self.structure_panel.aisleSelectionChanged.connect(self.set_selected_aisle)
        self.structure_panel.referenceReviewed.connect(self.mark_reference_reviewed)
        self.structure_panel.focusRequested.connect(self.focus_reference)
        self.structure_panel.deleteReferenceRequested.connect(self.delete_reference)
        self.structure_panel.clearExclusionsRequested.connect(self.clear_wall_exclusions)
        self.structure_panel.undoRequested.connect(self.undo)
        self.structure_panel.redoRequested.connect(self.redo)
        self.structure_panel.freezeRequested.connect(self.export_reference_dialog)
        self.structure_panel.clear_exclusions.setVisible(bool(self.row_exclusion_polygons))
        side_layout.addWidget(self.structure_panel)
        self.evidence_panel = EvidenceLayerPanel({
            'point_cloud': True, 'global_row_proposal': True,
            'local_row_tracks': True, 'aisle_proposals': True,
            'physical_row_labels': True, 'headland': True,
            **self.evidence_layer_visible}, self)
        self.evidence_panel.layerVisibilityChanged.connect(self.set_layer_visibility)
        self.evidence_panel.opacityChanged.connect(self.set_overlay_opacity)
        self.evidence_panel.hide()
        self.side = side
        center = QtWidgets.QWidget()
        center_layout = QtWidgets.QVBoxLayout(center)
        height_controls = QtWidgets.QHBoxLayout()
        self.height_filter = QtWidgets.QCheckBox('点云高度过滤（二维 / 三维）')
        height_controls.addWidget(self.height_filter)
        self.height_min = QtWidgets.QDoubleSpinBox()
        self.height_max = QtWidgets.QDoubleSpinBox()
        for box, label, value in ((self.height_min, 'Z 下限 ', -2.0),
                                  (self.height_max, 'Z 上限 ', 1.5)):
            box.setRange(-10000.0, 10000.0)
            box.setDecimals(2)
            box.setSingleStep(0.1)
            box.setPrefix(label)
            box.setSuffix(' 米')
            box.setValue(value)
            height_controls.addWidget(box)
        self.height_status = QtWidgets.QLabel('地图坐标系 Z；仅过滤显示点云')
        height_controls.addWidget(self.height_status)
        center_layout.addLayout(height_controls)
        region_controls = QtWidgets.QHBoxLayout()
        for label, callback in (
            ('新增参考垄', lambda: self.start_region_mode(8)),
            ('新增中心线', lambda: self.start_region_mode(9)),
            ('框选局部', lambda: self.start_region_mode(6)),
            ('恢复全图', self.clear_review_region),
        ):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(callback)
            region_controls.addWidget(button)
        self.isolate_selection = QtWidgets.QCheckBox('只看所选参考线')
        self.isolate_selection.toggled.connect(lambda: self.draw())
        region_controls.addWidget(self.isolate_selection)
        center_layout.addLayout(region_controls)
        self.region_status = QtWidgets.QLabel('旧墙壁区域已停用，可在左侧清除；Ctrl+Z 撤销编辑。' if self.row_exclusion_polygons else '点选参考线，拖动编辑；选中对象在二维 / 三维同步高亮。')
        self.region_status.setWordWrap(True)
        center_layout.addWidget(self.region_status)
        self.height_filter.toggled.connect(self.update_height_filter)
        self.height_min.valueChanged.connect(lambda: self.update_height_filter('min'))
        self.height_max.valueChanged.connect(lambda: self.update_height_filter('max'))
        view_tabs = QtWidgets.QTabWidget()
        self.view_tabs = view_tabs
        editor_view = QtWidgets.QWidget()
        editor_layout = QtWidgets.QVBoxLayout(editor_view)
        toolbar = NavigationToolbar2QT(self.canvas, self)
        toolbar_labels = {
            'Home': '回到初始视图', 'Back': '上一步', 'Forward': '下一步',
            'Pan': '平移', 'Zoom': '缩放', 'Subplots': '调整子图',
            'Save': '保存图像', 'Customize': '自定义图表',
        }
        for action in toolbar.actions():
            if action.text() in ('Subplots', 'Customize'):
                action.setVisible(False)
            label = toolbar_labels.get(action.text())
            if label:
                action.setText(label)
                action.setToolTip(label)
        editor_layout.addWidget(toolbar)
        cloud_style = QtWidgets.QHBoxLayout()
        cloud_style.addWidget(QtWidgets.QLabel('二维点云着色'))
        self.cloud_color_mode = QtWidgets.QComboBox()
        self.cloud_color_mode.addItem('高度 Z', 'height')
        self.cloud_color_mode.addItem('单色', 'mono')
        self.cloud_color_mode.setToolTip('高度色标使用显示点云的 2–98% 范围，减少孤立高点影响；过滤时使用所选 Z 范围。')
        self.cloud_color_mode.currentIndexChanged.connect(lambda: self.draw())
        cloud_style.addWidget(self.cloud_color_mode)
        cloud_style.addWidget(QtWidgets.QLabel('滚轮向上放大、向下缩小；以鼠标位置为中心'))
        cloud_style.addStretch(1)
        editor_layout.addLayout(cloud_style)
        editor_layout.addWidget(self.canvas)
        view_tabs.addTab(editor_view, '二维编辑')
        review_view = QtWidgets.QWidget()
        review_layout = QtWidgets.QVBoxLayout(review_view)
        review_layout.addWidget(self.review3d)
        view_tabs.addTab(review_view, '三维审查')
        center_layout.addWidget(view_tabs)
        split = QtWidgets.QSplitter()
        side.setMinimumWidth(260)
        side.setMaximumWidth(380)
        split.addWidget(side)
        split.addWidget(center)
        split.setSizes([300, 1060])
        self.setCentralWidget(split)
        QtWidgets.QShortcut('Ctrl+Z', self, activated=self.undo)
        QtWidgets.QShortcut('Ctrl+Shift+Z', self, activated=self.redo)
        QtWidgets.QShortcut('Ctrl+S', self, activated=self.save)
        self.canvas.mpl_connect('button_press_event', self.click)
        self.canvas.mpl_connect('motion_notify_event', self.motion)
        self.canvas.mpl_connect('button_release_event', self.release)
        self.canvas.mpl_connect('key_press_event', self.key)
        self.canvas.mpl_connect('scroll_event', self.scroll_zoom)
        self.refresh_list()
        if self.row_exclusion_polygons:
            before = self._workflow_snapshot()
            before['walls'] = []
            self.workflow_history.append(before)
        if self.reference_draft_path.exists():
            rows, aisles, config, analysis_hash = load_reference_draft(self.reference_draft_path, self.package)
            self.analysis = SimpleNamespace(config=config, analysis_hash=analysis_hash, navigation=None)
            self.row_proposals, self.aisle_proposals = rows, aisles
            self.rows_by_source['GLOBAL_PROFILE'] = list(rows)
            self._refresh_proposal_panel()
            self.structure_panel.status.setText('已恢复上次编辑草稿；可继续审查或重新生成。')
        else:
            self.structure_panel.status.setText('新建地图草稿：地图加载后将自动生成待人工审核的参考垄线和行道中心线。')
        self.draw(reset=True)
        self.loader = CloudLoader(self.package.path/'map.pcd', args.voxel, args.max_display_points)
        self.loader.loaded.connect(self.cloud_loaded)
        self.loader.failed.connect(self.cloud_failed)
        self.loader.start()
        self.statusBar().showMessage(str(self.output))

    def _site_workspace(self):
        configured = getattr(self.args, 'site_workspace', None)
        return Path(configured).expanduser().resolve() if configured else self.output.parent

    def update_height_filter(self, changed=None):
        if self.height_min.value() > self.height_max.value():
            target = self.height_max if changed == 'min' else self.height_min
            target.blockSignals(True)
            target.setValue(self.height_min.value() if changed == 'min' else self.height_max.value())
            target.blockSignals(False)
        bounds = (self.height_min.value(), self.height_max.value()) if self.height_filter.isChecked() else None
        self.review3d.canvas.set_height_filter(bounds)
        if self.cloud is not None:
            count = len(self.display_cloud())
            self.height_status.setText(f'{count:,} 点')
            self.height_status.setToolTip(f'地图 Z；保留 {count:,} / {len(self.cloud):,} 点，仅影响显示')
        self.draw()

    def display_cloud(self):
        if self.cloud is None:
            return None
        cloud = self.cloud
        mask = np.ones(len(cloud), dtype=bool)
        if self.height_filter.isChecked():
            mask &= (cloud[:, 2] >= self.height_min.value()) & (cloud[:, 2] <= self.height_max.value())
        if self.review_region is not None:
            mask &= points_in_polygon(cloud[:, :2], self.review_region)
        return cloud[mask]

    def sync_review_proposals(self):
        navigation = getattr(self.analysis, 'navigation', None)
        def lines_xyz(lines):
            pieces = []
            for line in lines:
                xy = np.asarray(line, dtype=float).reshape(-1, 2)
                for start, end in zip(xy[:-1], xy[1:]):
                    n = max(2, min(10000, int(np.linalg.norm(end-start) / 0.05) + 1))
                    samples = np.linspace(start, end, n)
                    z = np.full(n, self.cloud_reference_z)
                    if navigation is not None:
                        cols = np.floor((samples[:, 0]-navigation.origin_x_m)/navigation.resolution_m).astype(int)
                        rows = np.floor((samples[:, 1]-navigation.origin_y_m)/navigation.resolution_m).astype(int)
                        h, w = navigation.ground_valid.shape
                        inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
                        indices = np.flatnonzero(inside)
                        valid = navigation.ground_valid[rows[indices], cols[indices]]
                        indices = indices[valid]
                        z[indices] = navigation.ground_height_m[rows[indices], cols[indices]]
                    pieces.append(np.column_stack((samples, z + 0.08)))
            return np.concatenate(pieces) if pieces else np.empty((0, 3))
        visible_rows = self.global_rows_visible if self.active_proposal_source == 'GLOBAL_PROFILE' else self.local_rows_visible
        self.review3d.canvas.set_layer_xyz('row_centerline', lines_xyz(
            [row.centerline_xy for index, row in enumerate(self.row_proposals) if row.decision != 'rejected'
             and (not self.isolate_selection.isChecked() or index in self.selected_proposal_indices)] if visible_rows else []))
        self.review3d.canvas.set_layer_xyz('geometric_centerline', lines_xyz(
            [aisle.centerline_xy for index, aisle in enumerate(self.aisle_proposals) if aisle.decision != 'rejected'
             and (not self.isolate_selection.isChecked() or index == self.selected_aisle_index)] if self.aisles_visible else []))
        self.review3d.canvas.set_layer_xyz('safe_centerline', lines_xyz(
            [aisle.safe_centerline_xy for index, aisle in enumerate(self.aisle_proposals) if aisle.decision != 'rejected'
             and (not self.isolate_selection.isChecked() or index == self.selected_aisle_index)] if self.aisles_visible else []))

        selected = self.active_reference()
        self.review3d.canvas.set_layer_xyz('selected_reference', lines_xyz([selected[2].centerline_xy]) if selected else None)
        self.review3d.canvas.selection_label = (f'已选：{"垄" if selected[0] == "row" else "行道"} {selected[1]+1:02d} · {selected[2].auto_id}' if selected else '')

    def set_cloud_visible(self, visible):
        self.cloud_visible = bool(visible)
        self.draw()

    def set_trajectory_visible(self, visible):
        self.trajectory_visible = bool(visible)
        self.draw()

    def set_proposals_visible(self, visible):
        self.proposals_visible = bool(visible)
        self.draw()

    def set_physical_topology_visible(self, visible):
        self.physical_topology_visible = bool(visible)
        self.draw()

    def set_global_rows_visible(self, visible):
        self.global_rows_visible = bool(visible)
        self.draw()

    def set_local_rows_visible(self, visible):
        self.local_rows_visible = bool(visible)
        self.draw()

    def set_aisles_visible(self, visible):
        self.aisles_visible = bool(visible)
        self.draw()

    def set_overlay_opacity(self, opacity):
        self.overlay_opacity = float(np.clip(opacity, 0.0, 1.0))
        self.draw()

    def set_layer_visibility(self, layer, visible):
        visible = bool(visible)
        if layer == 'point_cloud':
            self.cloud_visible = visible
        elif layer == 'trajectory':
            self.trajectory_visible = visible
        elif layer == 'keyframes':
            self.keyframes_visible = visible
        elif layer == 'global_row_proposal':
            self.global_rows_visible = visible
        elif layer == 'local_row_tracks':
            self.local_rows_visible = visible
        elif layer == 'aisle_proposals':
            self.aisles_visible = visible
        elif layer == 'aisle_geometry':
            self.evidence_layer_visible[layer] = visible
        elif layer == 'physical_row_labels':
            self.physical_rows_visible = visible
        elif layer == 'headland':
            self.headlands_visible = visible
        elif layer in self.evidence_layer_visible:
            self.evidence_layer_visible[layer] = visible
        else:
            raise ValueError(f'unknown greenhouse workbench layer: {layer}')
        self.draw()

    def set_selected_proposals(self, indices):
        if self._refreshing_proposals:
            return
        self.selected_proposal_indices = {int(index) for index in indices}
        if indices:
            self._selection_kind = 'row'
            self.selected_aisle_index = -1
            self.structure_panel.aisle_list.blockSignals(True)
            self.structure_panel.aisle_list.setCurrentRow(-1)
            self.structure_panel.aisle_list.blockSignals(False)
        self.update_selection_description()
        self.draw()

    def set_selected_aisle(self, index):
        if self._refreshing_proposals:
            return
        self.selected_aisle_index = int(index)
        if index >= 0:
            self._selection_kind = 'aisle'
            self.selected_proposal_indices.clear()
            self.structure_panel.row_list.blockSignals(True)
            self.structure_panel.row_list.clearSelection()
            self.structure_panel.row_list.blockSignals(False)
        self.update_selection_description()
        self.draw()

    def run_structure_analysis(self, mode):
        if self.analysis_worker is not None and self.analysis_worker.isRunning():
            return
        self.structure_panel.set_busy(True)
        config_path = getattr(self.args, 'structure_config', None)
        self.analysis_worker = StructureAnalyzer(self.package, mode, config_path, self,
            row_exclusions=self.row_exclusion_polygons if self.wall_exclusions_enabled else [])
        self.analysis_worker.completed.connect(self.structure_analysis_completed)
        self.analysis_worker.failed.connect(self.structure_analysis_failed)
        self.analysis_worker.start()

    def structure_analysis_completed(self, analysis):
        self._record_workflow_state()
        self.analysis = analysis
        self.rows_by_source = {
            'GLOBAL_PROFILE': list(analysis.global_rows),
            'LOCAL_TRACKS': list(analysis.local_rows),
        }
        self.original_rows_by_source = {key: list(rows) for key, rows in self.rows_by_source.items()}
        self.proposal_history.clear()
        self.proposal_future.clear()
        self.manual_boundary_aisles = []
        self.structure_panel.set_busy(False)
        self.structure_panel.source.setEnabled(bool(analysis.global_rows and analysis.local_rows))
        self.structure_panel.status.setText(
            f'分析模式：{analysis.mode}；全局剖面 {len(analysis.global_rows)} 条，局部轨迹 {len(analysis.local_rows)} 条；'
            f'墙壁排除 {len(analysis.config.row_exclusion_polygons_xy)} 个区域；分析哈希={analysis.analysis_hash[:12]}。'
        )
        workspace = self._site_workspace()
        try:
            artifact = save_proposal_revision(analysis, workspace, self.topology.get('source', {}))
            self.structure_review_paths.append(str(artifact))
        except OSError as exc:
            QtWidgets.QMessageBox.warning(self, '保存提议失败', str(exc))
        source = self.structure_panel.source.currentData()
        available = self.rows_by_source.get(source, [])
        if not available:
            source = 'GLOBAL_PROFILE' if self.rows_by_source['GLOBAL_PROFILE'] else 'LOCAL_TRACKS'
            self.structure_panel.source.blockSignals(True)
            self.structure_panel.source.setCurrentIndex(self.structure_panel.source.findData(source))
            self.structure_panel.source.blockSignals(False)
        self.select_proposal_source(source)
        self._save_reference_draft()
        self.structure_panel.status.setText(f'已生成 {len(self.row_proposals)} 条垄线、{len(self.aisle_proposals)} 条中心线。点击对象开始编辑。')
        if analysis.navigation is not None:
            self.review3d.set_analysis(analysis.navigation, analysis.corridor_result,
                                       analysis.geometric_aisle_result)
        self.draw(reset=False)

    def structure_analysis_failed(self, message):
        self.structure_panel.set_busy(False)
        self.structure_panel.status.setText('结构分析失败：' + message)
        QtWidgets.QMessageBox.warning(self, '结构分析失败', message)

    def select_proposal_source(self, source):
        if source not in self.rows_by_source:
            return
        self.active_proposal_source = source
        self.selected_proposal_indices.clear()
        self.row_proposals = list(self.rows_by_source.get(source, []))
        if self.analysis and getattr(self.analysis, 'aisle_proposals', None) and source == 'GLOBAL_PROFILE':
            self.aisle_proposals = [replace(aisle, centerline_xy=simplify_reference_line(aisle.centerline_xy),
                diagnostics={**dict(aisle.diagnostics), 'reference_simplification_tolerance_m': .08})
                for aisle in self.analysis.aisle_proposals]
        else:
            self.aisle_proposals = list(proposal_aisles_for_rows(self.row_proposals, self.analysis.config)) if self.analysis else []
        self.aisle_proposals.extend(aisle for aisle in self.manual_boundary_aisles
                                     if aisle.left_row_auto_id in {row.auto_id for row in self.row_proposals})
        self._refresh_proposal_panel()

    def _refresh_proposal_panel(self):
        self._refreshing_proposals = True
        self.structure_panel.set_rows(self.row_proposals)
        self.structure_panel.set_aisles(self.aisle_proposals)
        self._refreshing_proposals = False
        self.selected_proposal_indices = set(self.structure_panel.selected_row_indices())
        self.selected_aisle_index = self.structure_panel.selected_aisle_index()
        self.update_selection_description()
        self.draw()

    def _save_proposal_review(self):
        self._save_reference_draft()
        if self.analysis is None or not self.analysis.analysis_hash:
            return None
        path = save_review_revision(self._site_workspace(), self.analysis.analysis_hash,
                                    self.row_proposals, self.aisle_proposals)
        self.structure_review_paths.append(str(path))
        return path

    def _record_proposal_state(self):
        self._record_workflow_state()
        self.proposal_history.append((self.active_proposal_source, copy.deepcopy(self.row_proposals),
                                      copy.deepcopy(self.aisle_proposals),
                                      copy.deepcopy(self.manual_boundary_aisles)))
        self.proposal_future.clear()

    def undo_proposals(self):
        if not self.proposal_history:
            return
        self.proposal_future.append((self.active_proposal_source, copy.deepcopy(self.row_proposals),
                                     copy.deepcopy(self.aisle_proposals), copy.deepcopy(self.manual_boundary_aisles)))
        source, rows, aisles, boundaries = self.proposal_history.pop()
        self.active_proposal_source = source
        self.row_proposals, self.aisle_proposals, self.manual_boundary_aisles = rows, aisles, boundaries
        self.rows_by_source[source] = list(rows)
        self._refresh_proposal_panel()
        self._save_proposal_review()

    def redo_proposals(self):
        if not self.proposal_future:
            return
        self.proposal_history.append((self.active_proposal_source, copy.deepcopy(self.row_proposals),
                                      copy.deepcopy(self.aisle_proposals), copy.deepcopy(self.manual_boundary_aisles)))
        source, rows, aisles, boundaries = self.proposal_future.pop()
        self.active_proposal_source = source
        self.row_proposals, self.aisle_proposals, self.manual_boundary_aisles = rows, aisles, boundaries
        self.rows_by_source[source] = list(rows)
        self._refresh_proposal_panel()
        self._save_proposal_review()

    def _replace_rows(self, rows, record=True):
        if record:
            self._record_proposal_state()
        manual_reference_aisles = [item for item in self.aisle_proposals if item.aisle_kind == 'REFERENCE']
        old_aisles = {(item.left_row_auto_id, item.right_row_auto_id): item for item in self.aisle_proposals if item.aisle_kind != 'REFERENCE'}
        old_rows = {row.auto_id: row for row in (self.workflow_history[-1]['rows'] if self.workflow_history else self.row_proposals)}
        unchanged = {row.auto_id for row in rows if row.auto_id in old_rows
            and row.centerline_xy == old_rows[row.auto_id].centerline_xy
            and row.decision == old_rows[row.auto_id].decision}
        self.row_proposals = list(rows)
        self.rows_by_source[self.active_proposal_source] = list(rows)
        generated = list(proposal_aisles_for_rows(rows, self.analysis.config)) if self.analysis else []
        self.aisle_proposals = []
        reserved = {item.auto_id for item in self.aisle_proposals+list(old_aisles.values())+manual_reference_aisles}
        for item in generated:
            old = old_aisles.get((item.left_row_auto_id, item.right_row_auto_id))
            if old and item.left_row_auto_id in unchanged and item.right_row_auto_id in unchanged:
                self.aisle_proposals.append(old)
                continue
            identifier = old.auto_id if old else item.auto_id
            if old is None and identifier in reserved:
                number = 1
                while f'AUTO-EDIT-A{number:03d}' in reserved:
                    number += 1
                identifier = f'AUTO-EDIT-A{number:03d}'
            reserved.add(identifier)
            self.aisle_proposals.append(replace(item, auto_id=identifier,
                centerline_xy=simplify_reference_line(item.centerline_xy)))
        self.aisle_proposals.extend(manual_reference_aisles)
        self.aisle_proposals.extend(aisle for aisle in self.manual_boundary_aisles
                                     if aisle.left_row_auto_id in {row.auto_id for row in rows})
        self._refresh_proposal_panel()
        self._save_proposal_review()

    def _active_aisle_minimum_width(self):
        return self.analysis.config.corridor.aisle_minimum_width_m if self.analysis else 0.45

    def accept_row_proposals(self, indices):
        rows = list(self.row_proposals)
        for index in sorted(set(indices)):
            if not 0 <= index < len(rows):
                continue
            row = rows[index]
            existing_ids = {item.get('id') for item in self.topology.get('rows', [])}
            existing_ids.update(item.physical_row_id for item in rows if item.physical_row_id)
            physical_id, ok = QtWidgets.QInputDialog.getText(
                self, '填写实际垄编号', f'为 {row.auto_id} 指定实际编号（AUTO 编号会作为来源记录保留）：'
            )
            if not ok:
                break
            if not physical_id.strip() or physical_id.strip() in existing_ids:
                QtWidgets.QMessageBox.warning(self, '实际垄编号', '请输入非空且唯一的实际垄编号。')
                continue
            direction, ok = QtWidgets.QInputDialog.getItem(
                self, '确认实际垄线通行方向', '请选择人工确认的通行方向：',
                ['正向通行', '反向通行', '双向通行'], 2, False
            )
            if not ok:
                break
            direction_value = {'正向通行': 'forward', '反向通行': 'reverse', '双向通行': 'bidirectional'}[direction]
            rows[index] = decision_for_proposal(
                row, 'accepted', physical_row_id=physical_id, direction=direction_value
            )
        self._replace_rows(rows)

    def reject_row_proposals(self, indices):
        rows = list(self.row_proposals)
        for index in set(indices):
            if 0 <= index < len(rows):
                rows[index] = decision_for_proposal(rows[index], 'rejected')
        self._replace_rows(rows)

    def merge_row_selection(self, indices):
        indices = sorted(set(indices))
        if len(indices) < 2:
            self.structure_panel.status.setText('请至少选择两条垄线提议再合并。')
            return
        merged = merge_row_proposals([self.row_proposals[index] for index in indices],
                                     f'AUTO-M{len(self.row_proposals)+1:03d}')
        rows = [row for index, row in enumerate(self.row_proposals) if index not in set(indices)] + [merged]
        self._replace_rows(sorted(rows, key=lambda row: row.lateral_v_m))

    def split_row_selection(self, index, fraction):
        if not 0 <= index < len(self.row_proposals):
            return
        row = self.row_proposals[index]
        suffix = len(self.row_proposals) + 1
        first, second = split_row_proposal(row, split_fraction=fraction,
                                           first_auto_id=f'AUTO-S{suffix:03d}A',
                                           second_auto_id=f'AUTO-S{suffix:03d}B')
        rows = [item for i, item in enumerate(self.row_proposals) if i != index] + [first, second]
        self._replace_rows(sorted(rows, key=lambda item: (item.lateral_v_m, item.centerline_xy[0])))

    def edit_row_proposal(self, index):
        if not 0 <= index < len(self.row_proposals):
            return
        row = self.row_proposals[index]
        dialog = ProposalGeometryDialog(row, self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        value = dialog.value()
        centerline = np.asarray(value['centerline_xy'], dtype=float)
        if centerline.ndim != 2 or centerline.shape[1] != 2 or len(centerline) < 2 or not np.isfinite(centerline).all():
            QtWidgets.QMessageBox.warning(self, '提议几何形状', '请至少输入两个有效的 XY 坐标点。')
            return
        direction = np.asarray(row.direction_xy, dtype=float)
        perpendicular = np.array([-direction[1], direction[0]])
        lateral_v = float(np.median(centerline @ perpendicular))
        updated = replace(row, lateral_v_m=lateral_v,
                          centerline_xy=tuple(tuple(float(v) for v in point) for point in centerline),
                          half_width_m=float(value['width_m']) / 2.0, decision='pending',
                          physical_row_id=None, confirmed_direction=None,
                          diagnostics={**dict(row.diagnostics), 'manually_edited': True})
        rows = list(self.row_proposals)
        rows[index] = updated
        self._replace_rows(rows)

    def delete_row_selection(self, indices):
        remove = set(indices)
        self._replace_rows([row for index, row in enumerate(self.row_proposals) if index not in remove])

    def reset_proposals(self):
        if self.analysis is None:
            return
        self._record_proposal_state()
        self.manual_boundary_aisles = []
        self.row_proposals = list(self.original_rows_by_source.get(self.active_proposal_source, []))
        self.rows_by_source[self.active_proposal_source] = list(self.row_proposals)
        self.aisle_proposals = list(proposal_aisles_for_rows(self.row_proposals, self.analysis.config))
        self._refresh_proposal_panel()
        self._save_proposal_review()
        self.structure_panel.status.setText('草稿提议的审查决定和修改已重置为检测器输出。')

    def accept_aisle_proposal(self, index):
        if not 0 <= index < len(self.aisle_proposals):
            return
        aisle = self.aisle_proposals[index]
        by_id = {row.auto_id: row for row in self.row_proposals}
        if aisle.left_row_auto_id not in by_id or by_id[aisle.left_row_auto_id].decision != 'accepted':
            QtWidgets.QMessageBox.warning(self, '接受通道', '请先人工接受相邻垄线。')
            return
        if aisle.aisle_kind == 'INTERIOR' and (
            aisle.right_row_auto_id not in by_id or by_id[aisle.right_row_auto_id].decision != 'accepted'
        ):
            QtWidgets.QMessageBox.warning(self, '接受通道', '两侧相邻垄线都必须先经人工接受。')
            return
        physical_id, ok = QtWidgets.QInputDialog.getText(
            self, '填写实际通道编号', f'为 {aisle.auto_id} 指定实际通道编号：'
        )
        if not ok or not physical_id.strip():
            return
        self._record_proposal_state()
        self.aisle_proposals[index] = replace(aisle, decision='accepted', physical_aisle_id=physical_id.strip())
        self._refresh_proposal_panel()
        self._save_proposal_review()

    def reject_aisle_proposal(self, index):
        if 0 <= index < len(self.aisle_proposals):
            self._record_proposal_state()
            self.aisle_proposals[index] = replace(self.aisle_proposals[index], decision='rejected', physical_aisle_id=None)
            self._refresh_proposal_panel()
            self._save_proposal_review()

    def freeze_structure_proposals(self):
        if self.analysis is None:
            QtWidgets.QMessageBox.warning(self, '固化拓扑', '请先运行结构分析。')
            return
        workspace = self._site_workspace()
        default = workspace / 'annotation' / 'structure_frozen_v1' / 'greenhouse_topology.yaml'
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, '将已审查拓扑另存为新版本', str(default), '拓扑 YAML 文件 (*.yaml)'
        )
        if not filename:
            return
        target = Path(filename).expanduser().resolve()
        try:
            freeze_accepted_proposals(self.topology, self.row_proposals, self.aisle_proposals,
                                      target, package=self.package)
            validation = validate_topology(
                load_topology(target), self.package, target, target.parent / 'keyframe_topology_labels.csv'
            )
            if not validation['valid']:
                raise ValueError('; '.join(validation['errors']))
            QtWidgets.QMessageBox.information(self, '拓扑已固化',
                                               f'已校验并保存 schema-v1 拓扑：{target}')
        except (ValueError, OSError, FileExistsError) as exc:
            QtWidgets.QMessageBox.warning(self, '固化未完成', str(exc))

    def editable(self):
        if self.topology['annotation']['status'] == 'frozen':
            self.statusBar().showMessage('已冻结的标注为只读状态。请创建新的草稿版本后再编辑。')
            return False
        return True

    def mutate(self, callback):
        if not self.editable():
            return
        before = copy.deepcopy(self.topology)
        self._record_workflow_state()
        callback()
        self.history.append(before)
        self.future.clear()
        self.topology['annotation']['manual_review_confirmed'] = False
        self.dirty = True
        self.refresh_list()
        self.draw()

    def cloud_loaded(self, cloud, total):
        self.cloud = cloud
        finite_z = cloud[:, 2][np.isfinite(cloud[:, 2])]
        self.cloud_reference_z = float(np.median(finite_z)) if len(finite_z) else 0.0
        self.review3d.set_cloud(cloud)
        self.update_height_filter()
        self.info.setText(f'地图点数：{total:,} → 显示 {len(cloud):,} 点\n'
                          f'关键帧：{len(self.package.poses)}\n坐标系：{self.package.map_frame}\n'
                          f'主后端：{self.package.backend_id}')
        if self.other:
            residual = self.topology['backend_transforms'][self.other.backend_id]['residual_xy_m']
            self.info.setText(self.info.text()+f'\n另一后端 SE2 残差中位数/P95：{residual["median"]:.3f}/{residual["p95"]:.3f} 米')
        self.draw(reset=True)
        self.gui_load_seconds = time.perf_counter() - self.gui_load_started
        self.statusBar().showMessage(f'地图已加载，用时 {self.gui_load_seconds:.3f} 秒')
        if self.args.smoke_output:
            if self.args.smoke_save_empty:
                if self.topology['rows'] or self.topology['headlands'] or self.topology['scenes']:
                    self.cloud_failed('--smoke-save-empty 不允许覆盖已有人工标注')
                    return
                self.save(interactive=False)
            QtCore.QTimer.singleShot(200, self.finish_smoke)
        elif self.isVisible() and not self.reference_draft_path.exists():
            # First open of a new draft starts the offline proposal pass. It
            # runs in StructureAnalyzer (QThread); proposals remain a separate
            # editable reference layer and never become physical topology IDs.
            QtCore.QTimer.singleShot(50, lambda: self.run_structure_analysis('COMPARE'))

    def finish_smoke(self):
        self.figure.savefig(self.args.smoke_output, dpi=150)
        screenshot = Path(self.args.smoke_output)
        self.grab().save(str(screenshot.with_name(screenshot.stem+'_window.png')))
        report = dict(map_package=str(self.package.path), output=str(self.output),
            screenshot=self.args.smoke_output, rows=len(self.topology['rows']), headlands=len(self.topology['headlands']),
            scene_markers=len(self.topology['scenes']), display_points=len(self.cloud),
            gui_load_seconds=self.gui_load_seconds,
            annotation_status=self.topology['annotation']['status'],
            map_manifest_sha256=self.package.manifest_sha256)
        screenshot.with_suffix('.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        print(json.dumps(report), flush=True)
        self.dirty = False
        self.close()

    def cloud_failed(self, reason):
        self.info.setText('地图加载失败：'+reason)
        if self.args.smoke_output:
            print('图形界面冒烟检查失败：'+reason, file=sys.stderr, flush=True)
            QtWidgets.QApplication.exit(2)
        else:
            QtWidgets.QMessageBox.critical(self, '地图加载', reason)

    def entries(self):
        return [(collection, i, item) for collection in ('rows', 'headlands', 'scenes')
                for i, item in enumerate(self.topology[collection])]

    def refresh_list(self):
        self.distance.blockSignals(True)
        self.distance.setValue(self.topology['assignment']['max_row_assignment_distance_m'])
        self.distance.setReadOnly(self.topology['annotation']['status'] == 'frozen')
        self.distance.blockSignals(False)
        selected = self.list.currentRow()
        self.list.blockSignals(True)
        self.list.clear()
        for collection, _, item in self.entries():
            name = item.get('id', item.get('scene_id'))
            collection_names = {'rows': '垄线', 'headlands': '地头', 'scenes': '场景'}
            self.list.addItem(collection_names.get(collection, collection)+': '+name)
        self.list.setCurrentRow(min(selected, self.list.count()-1))
        self.list.blockSignals(False)

    def selected(self):
        index = self.list.currentRow()
        entries = self.entries()
        return entries[index] if 0 <= index < len(entries) else None

    def select_item(self, _):
        item = self.selected()
        if item:
            self.info.setText(json.dumps(item[2], ensure_ascii=False, indent=2))
        self.draw()

    def draw_evidence_layers(self):
        if self.analysis is None or getattr(self.analysis, 'navigation', None) is None:
            return
        analysis = self.analysis
        navigation = analysis.navigation
        extent = navigation.bounds_m()
        extent = (extent[0], extent[2], extent[1], extent[3])
        terrain = analysis.terrain
        corridor = analysis.corridor_result
        local = analysis.local_result
        geometric = analysis.geometric_aisle_result
        layers = (
            ('ground', terrain.ground_height_m, 'terrain', None, None, False),
            ('ground_confidence', terrain.ground_confidence, 'viridis', 0.0, 1.0, False),
            ('slope', terrain.robust_slope_deg, 'magma', 0.0,
             float(analysis.config.navigation.maximum_slope_deg), False),
            ('plane_residual', terrain.robust_plane_residual_m, 'coolwarm', None, None, True),
            ('terrain_ridge', terrain.ridge_evidence, 'YlOrRd', 0.0, 1.0, False),
            ('terrain_depression', terrain.depression_evidence, 'PuBu', 0.0, 1.0, False),
            ('terrain_step', terrain.step_evidence, 'inferno', 0.0, 1.0, False),
            ('global_row_support', analysis.global_result.row_support
             if analysis.global_result is not None else None, 'plasma', 0.0, 1.0, False),
            ('row_structural_band',
             local.row_structural_band if local is not None else
             corridor.row_structural_band if corridor is not None else None,
             'Purples', 0.0, 1.0, False),
            ('aisle_geometry', corridor.aisle_geometric_envelope
             if corridor is not None else None, 'winter', 0.0, 1.0, False),
            ('geometric_centerline', geometric.mask if geometric is not None else None,
             'winter', 0.0, 1.0, False),
            ('safe_aisle', corridor.aisle_candidate if corridor is not None else
             local.aisle_candidate if local is not None else None,
             'summer', 0.0, 1.0, False),
            ('safe_centerline', corridor.aisle_centerline if corridor is not None else
             local.aisle_centerline if local is not None else None,
             'autumn', 0.0, 1.0, False),
        )
        for name, values, color_map, minimum, maximum, diverging in layers:
            if not self.evidence_layer_visible.get(name, False) or values is None:
                continue
            data = np.asarray(values)
            if data.shape != np.asarray(navigation.occupancy).shape:
                continue
            if data.dtype == np.bool_:
                valid = data
                image = np.ma.masked_where(~valid, np.ones(data.shape, dtype=np.float32))
                minimum, maximum = 0.0, 1.0
            else:
                valid = np.isfinite(data)
                if name in {'ground', 'ground_confidence', 'slope', 'plane_residual'}:
                    valid &= np.asarray(terrain.ground_valid, dtype=bool)
                if not np.any(valid):
                    continue
                image = np.ma.masked_where(~valid, data)
                values_valid = data[valid]
                if diverging:
                    limit = float(np.percentile(np.abs(values_valid), 98))
                    if not np.isfinite(limit) or limit <= 1e-12:
                        limit = 1e-12
                    minimum, maximum = -limit, limit
                elif minimum is None or maximum is None:
                    minimum, maximum = (float(value) for value in np.percentile(values_valid, [2, 98]))
                    if not np.isfinite(minimum) or not np.isfinite(maximum) or maximum <= minimum:
                        maximum = minimum + 1e-12
            self.axes.imshow(
                image, origin='lower', extent=extent, interpolation='nearest',
                cmap=color_map, vmin=minimum, vmax=maximum,
                alpha=0.55 * self.overlay_opacity, zorder=1,
            )
        if self.evidence_layer_visible.get('local_row_observations', False) and local is not None:
            direction = np.asarray(analysis.diagnostics.get('local_direction_xy') or (1.0, 0.0), dtype=float)
            direction /= max(float(np.linalg.norm(direction)), 1e-12)
            perpendicular = np.array([-direction[1], direction[0]])
            u = np.asarray([item.u_center_m for item in local.observations], dtype=float)
            v = np.asarray([item.v_center_m for item in local.observations], dtype=float)
            xy = u[:, None] * direction[None, :] + v[:, None] * perpendicular[None, :]
            supports = np.asarray([item.support for item in local.observations], dtype=float)
            self.axes.scatter(
                xy[:, 0], xy[:, 1], c=supports, cmap='cool', vmin=0.0, vmax=1.0,
                s=10.0 + 24.0 * supports, alpha=self.overlay_opacity,
                edgecolors='none', zorder=4, label='局部垄线观测',
            )

    def draw(self, reset=False):
        xlim, ylim = self.axes.get_xlim(), self.axes.get_ylim()
        self.axes.clear()
        self.axes.set_facecolor('#000000')
        self.axes.tick_params(colors='#dddddd')
        self.axes.xaxis.label.set_color('#dddddd')
        self.axes.yaxis.label.set_color('#dddddd')
        self.axes.title.set_color('#eeeeee')
        for spine in self.axes.spines.values():
            spine.set_color('#888888')
        self.sync_review_proposals()
        if self.cloud is not None and self.cloud_visible:
            cloud = self.display_cloud()
            cloud = cloud[np.all(np.isfinite(cloud[:, :3]), axis=1)]
            if len(cloud) and self.cloud_color_mode.currentData() == 'height':
                if self.height_filter.isChecked():
                    low, high = self.height_min.value(), self.height_max.value()
                else:
                    low, high = (float(value) for value in np.quantile(cloud[:, 2], [.02, .98]))
                high = max(high, low + 0.01)
                artist = self.axes.scatter(cloud[:, 0], cloud[:, 1], s=1.0,
                    c=cloud[:, 2], cmap='turbo', vmin=low, vmax=high,
                    alpha=.95, linewidths=0, rasterized=True)
                color_axes = self.axes.inset_axes([.88, .07, .025, .25])
                colorbar = self.figure.colorbar(artist, cax=color_axes)
                colorbar.ax.set_facecolor('#000000')
                colorbar.ax.set_title('Z [米]', fontsize=8, color='#dddddd')
                colorbar.outline.set_edgecolor('#888888')
                colorbar.ax.tick_params(labelsize=8, colors='#dddddd')
            else:
                self.axes.scatter(cloud[:, 0], cloud[:, 1], s=.25,
                    c='#cdd5dd', alpha=.7, rasterized=True)
        xy = np.array([[p['x'], p['y']] for p in self.package.poses])
        if self.trajectory_visible:
            self.axes.plot(xy[:, 0], xy[:, 1], color='#71b4ef', lw=.8, label='前端轨迹')
        if self.keyframes_visible:
            self.axes.scatter(xy[:, 0], xy[:, 1], s=4, c='#71b4ef', alpha=.7, label='关键帧')
        self.draw_evidence_layers()
        selected = self.selected()
        for collection, index, item in self.entries():
            if collection == 'rows' and not (self.physical_topology_visible and self.physical_rows_visible):
                continue
            if collection == 'headlands' and not (self.physical_topology_visible and self.headlands_visible):
                continue
            if collection == 'scenes' and not self.physical_topology_visible:
                continue
            highlight = selected is not None and selected[:2] == (collection, index)
            if collection == 'rows':
                points = np.array(item['centerline'])
                self.axes.plot(points[:, 0], points[:, 1], '-o', ms=5 if highlight else 2,
                               lw=2 if highlight else 1.3, color='#d65032')
                self.axes.annotate(item['id'], points[len(points)//2], color='#ff956e')
            elif collection == 'headlands':
                points = np.array(item['polygon'])
                self.axes.fill(points[:, 0], points[:, 1], color='#3ba76d', alpha=.17)
                points = np.vstack([points, points[0]])
                self.axes.plot(points[:, 0], points[:, 1], '-o', ms=5 if highlight else 2, color='#248855')
                self.axes.annotate(item['id'], points.mean(axis=0), color='#73dca0')
            else:
                self.axes.scatter([item['x']], [item['y']], s=60 if highlight else 25, marker='*', color='#d58cef')
                self.axes.annotate(item['scene_id'], (item['x'], item['y']), fontsize=8)
        if self.proposals_visible:
            for index, proposal in enumerate(self.row_proposals):
                if proposal.decision == 'rejected' or (self.isolate_selection.isChecked() and index not in self.selected_proposal_indices):
                    continue
                is_source_visible = (proposal.source_mode == 'LOCAL_TRACKS' and self.local_rows_visible
                                     or proposal.source_mode != 'LOCAL_TRACKS' and self.global_rows_visible)
                if not is_source_visible:
                    continue
                points = np.asarray(proposal.centerline_xy, dtype=float)
                color = '#a0aaff' if proposal.source_mode == 'LOCAL_TRACKS' else '#e1a32e'
                selected = index in self.selected_proposal_indices
                self.axes.plot(points[:, 0], points[:, 1], '-o' if selected else '-',
                    lw=3.5 if selected else 1.3, ms=6 if selected else 0,
                    color='#ffffff' if selected else color, alpha=1.0 if selected else .7, zorder=12 if selected else 5)
                if selected:
                    self.axes.annotate(f'垄 {index+1:02d} · {proposal.auto_id}', points.mean(axis=0),
                        color='#ffffff', fontsize=10, zorder=13, bbox=dict(facecolor='black', alpha=.75, edgecolor='none'))
            for index, aisle in enumerate(self.aisle_proposals):
                if not self.aisles_visible or aisle.decision == 'rejected' or (self.isolate_selection.isChecked() and index != self.selected_aisle_index):
                    continue
                points = np.asarray(aisle.centerline_xy, dtype=float)
                if len(points) >= 2:
                    selected = index == self.selected_aisle_index
                    self.axes.plot(points[:, 0], points[:, 1], '-o' if selected else '-',
                        lw=3.5 if selected else 1.3, ms=6 if selected else 0,
                        color='#ffffff' if selected else '#45cfff', alpha=1.0 if selected else .7, zorder=12 if selected else 5)
                    if selected:
                        self.axes.annotate(f'行道 {index+1:02d} · {aisle.auto_id}', points.mean(axis=0),
                            color='#ffffff', fontsize=10, zorder=13, bbox=dict(facecolor='black', alpha=.75, edgecolor='none'))
        for polygon in self.row_exclusion_polygons if self.wall_exclusions_enabled else []:
            points = np.asarray(polygon)
            self.axes.fill(points[:, 0], points[:, 1], facecolor='none',
                edgecolor='#ff5275', alpha=.8, zorder=8)
            self.axes.annotate('非垄区域', points[0], color='#ff92a8', fontsize=8, zorder=9)
        if self.pending:
            points = np.array(self.pending)
            self.axes.plot(points[:, 0], points[:, 1], 'o--', color='#db9933')
        self.axes.set_aspect('equal', adjustable='box')
        self.axes.set_xlabel(f'x [米] — {self.package.map_frame}')
        self.axes.set_ylabel('y [米]')
        status = {'draft': '草稿', 'frozen': '已冻结'}.get(
            self.topology['annotation']['status'], self.topology['annotation']['status']
        )
        self.axes.set_title(f'{self.package.backend_id} | 参考垄线与行道')
        if self.review_region is not None and reset:
            region = np.asarray(self.review_region)
            self.axes.set_xlim(region[:, 0].min(), region[:, 0].max())
            self.axes.set_ylim(region[:, 1].min(), region[:, 1].max())
        elif not reset:
            self.axes.set_xlim(xlim)
            self.axes.set_ylim(ylim)
        self.figure.tight_layout()
        self.canvas.draw_idle()

    def scroll_zoom(self, event):
        if (event.inaxes != self.axes or event.xdata is None or event.ydata is None
                or self.drag is not None):
            return
        step = float(getattr(event, 'step', 0.0))
        if not math.isfinite(step) or step == 0.0:
            return
        anchor = np.array([event.xdata, event.ydata], dtype=float)
        if not np.all(np.isfinite(anchor)):
            return
        factor = 1.2 ** (-max(-20.0, min(20.0, step)))
        limits = np.array([self.axes.get_xlim(), self.axes.get_ylim()])
        updated = anchor[:, None] + (limits - anchor[:, None]) * factor
        spans = np.abs(updated[:, 1] - updated[:, 0])
        if not np.all(np.isfinite(updated)) or np.any(spans < 1e-4) or np.any(spans > 1e7):
            return
        toolbar = self.canvas.toolbar
        if toolbar is not None and toolbar._nav_stack() is None:
            toolbar.push_current()
        self.axes.set_xlim(updated[0])
        self.axes.set_ylim(updated[1])
        if toolbar is not None:
            toolbar.push_current()
        self.canvas.draw_idle()

    def start_region_mode(self, index):
        self.view_tabs.setCurrentIndex(0)
        if self.canvas.toolbar.mode:
            if self.canvas.toolbar.mode.name == 'PAN':
                self.canvas.toolbar.pan()
            else:
                self.canvas.toolbar.zoom()
        self.mode_combo.setCurrentIndex(index)
        self.region_status.setText('局部审查：左键点击矩形两个对角。' if index == 6 else
            '左键添加参考线顶点，右键或回车完成，Esc 取消。' if index in (8, 9) else
            '沿墙体圈狭窄区域，右键或回车完成；不能圈整片温室。')

    def clear_review_region(self):
        self.review_region = None
        self.review3d.canvas.set_review_region(None)
        self.mode_combo.setCurrentIndex(0)
        self.draw(reset=True)
        self.update_height_filter()
        self.region_status.setText('已恢复全图显示；墙壁排除区域仍保留。')

    def save_row_exclusions(self):
        self.row_exclusions_path.parent.mkdir(parents=True, exist_ok=True)
        self.row_exclusions_path.write_text(yaml.safe_dump({
            'map_package_hash': self.package.map_package_hash,
            'row_exclusion_polygons_xy': self.row_exclusion_polygons,
            'enabled': self.wall_exclusions_enabled,
        }, allow_unicode=True), encoding='utf-8')
        self.structure_panel.clear_exclusions.setVisible(bool(self.row_exclusion_polygons))
        self.region_status.setText(f'墙壁区域 {len(self.row_exclusion_polygons)} 个已保存；' +
            ('重新分析后生效。' if self.wall_exclusions_enabled else '已停用，不影响生成。'))

    def undo_row_exclusion(self):
        if self.row_exclusion_polygons:
            self._record_workflow_state()
            polygons = self.row_exclusion_polygons[:-1]
            previous = self.row_exclusion_polygons
            self.row_exclusion_polygons = polygons
            try:
                self.save_row_exclusions()
            except OSError as exc:
                self.row_exclusion_polygons = previous
                self.region_status.setText(f'保存墙壁区域失败：{exc}')
            self.draw()

    def finish_row_exclusion(self):
        try:
            polygon = validate_polygon(self.pending).tolist()
            if self.cloud is not None and len(self.cloud) and np.mean(points_in_polygon(self.cloud[:, :2], polygon)) > .35:
                raise ValueError('圈选覆盖过大，像是整片温室；请只沿墙体圈狭窄区域。')
            self._record_workflow_state()
            self.wall_exclusions_enabled = True
            self.row_exclusion_polygons.append(polygon)
            try:
                self.save_row_exclusions()
            except OSError:
                self.row_exclusion_polygons.pop()
                raise
            self.pending = []
            self.mode_combo.setCurrentIndex(0)
            self.draw()
        except (ValueError, OSError) as exc:
            self.region_status.setText(str(exc))

    def change_mode(self, index):
        self.mode = ['inspect', 'row', 'headland', 'scene', 'edit', 'boundary_anchor', 'review_region', 'row_exclusion', 'reference_row', 'reference_aisle'][index]
        self.pending = []
        self.draw()

    def assignment_changed(self, value):
        self.mutate(lambda: self.topology['assignment'].update(max_row_assignment_distance_m=float(value)))

    def nearest_pose(self, x, y):
        return min(self.package.poses, key=lambda p: (p['x']-x)**2+(p['y']-y)**2)

    def click(self, event):
        if event.inaxes != self.axes or event.xdata is None:
            return
        if self.canvas.toolbar.mode:
            return
        if self.mode == 'inspect' and self.reference_click(event):
            return
        if self.mode in ('reference_row', 'reference_aisle'):
            if event.button == 3 or getattr(event, 'dblclick', False):
                self.finish_reference_line()
            elif event.button == 1:
                self.pending.append([float(event.xdata), float(event.ydata)])
                self.draw()
        elif self.mode == 'review_region' and event.button == 1:
            self.pending.append([float(event.xdata), float(event.ydata)])
            if len(self.pending) == 2:
                a, b = np.asarray(self.pending)
                low, high = np.minimum(a, b), np.maximum(a, b)
                if np.any(high-low < .05):
                    self.pending = []
                    self.region_status.setText('局部框选需要宽、高均大于 5 厘米，请重选。')
                    return
                self.review_region = [low.tolist(), [high[0], low[1]], high.tolist(), [low[0], high[1]]]
                self.review3d.canvas.set_review_region(self.review_region)
                self.mode_combo.setCurrentIndex(0)
                self.draw(reset=True)
                self.update_height_filter()
                self.region_status.setText('已裁剪二维 / 三维局部显示；可切换三维侧视并降低 Z 上限查看垄体。')
            else:
                self.draw()
        elif self.mode == 'row_exclusion':
            if event.button == 3 or event.dblclick:
                self.finish_row_exclusion()
            elif event.button == 1:
                self.pending.append([float(event.xdata), float(event.ydata)])
                self.draw()
        elif self.mode in ('row', 'headland') and self.editable():
            if event.button == 3 or event.dblclick:
                self.finish_geometry()
            elif event.button == 1:
                self.pending.append([float(event.xdata), float(event.ydata)])
                self.draw()
        elif self.mode == 'boundary_anchor' and event.button == 1:
            selected_indices = self.structure_panel.selected_row_indices()
            if len(selected_indices) != 1 or selected_indices[0] >= len(self.row_proposals):
                self.statusBar().showMessage('请先选择一条已审查的垄线提议，再放置边界锚点。')
                return
            row = self.row_proposals[selected_indices[0]]
            if row.decision != 'accepted':
                self.statusBar().showMessage('边界通道锚点需要关联到已接受的实际垄线。')
                return
            try:
                item = make_boundary_aisle_proposal(
                    row, (event.xdata, event.ydata),
                    auto_id=f'AUTO-AB{len(self.manual_boundary_aisles)+1:03d}',
                    minimum_width_m=self._active_aisle_minimum_width(),
                )
                self._record_proposal_state()
                self.manual_boundary_aisles.append(item)
                self.aisle_proposals.append(item)
                self._refresh_proposal_panel()
                self.statusBar().showMessage('已将指定边界锚点加入通道提议草稿。')
            except ValueError as exc:
                self.statusBar().showMessage(str(exc))
        elif self.mode == 'scene' and event.button == 1 and self.editable():
            pose = self.nearest_pose(event.xdata, event.ydata)
            dialog = SceneDialog(self.topology, pose, parent=self)
            if dialog.exec_() == QtWidgets.QDialog.Accepted:
                self.add_scene(dialog.value())
        elif self.mode == 'edit' and event.button == 1 and self.editable():
            selected = self.selected()
            if selected and selected[0] in ('rows', 'headlands'):
                geometry = 'centerline' if selected[0] == 'rows' else 'polygon'
                vertices = np.array(selected[2][geometry])
                pixels = self.axes.transData.transform(vertices)
                distances = np.linalg.norm(pixels-np.array([event.x, event.y]), axis=1)
                if distances.min() < 14:
                    self.drag = (selected[0], selected[1], geometry, int(distances.argmin()))
                    self.drag_before = copy.deepcopy(self.topology)
        elif self.mode == 'inspect':
            pose = self.nearest_pose(event.xdata, event.ydata)
            label = label_pose(self.topology, pose['x'], pose['y'], pose['yaw_rad'])
            self.info.setText(f'关键帧 {pose["keyframe"]}\n数据包时间戳 {pose["timestamp"]:.9f}\n'
                f'x={pose["x"]:.3f} y={pose["y"]:.3f} 航向角={pose["yaw_deg"]:.2f} 度\n'+json.dumps(label, indent=2, ensure_ascii=False))

    def motion(self, event):
        if self.reference_drag is not None:
            self.reference_motion(event)
            return
        if self.drag and event.inaxes == self.axes and event.xdata is not None:
            collection, index, geometry, vertex = self.drag
            self.topology[collection][index][geometry][vertex] = [float(event.xdata), float(event.ydata)]
            self.draw()

    def release(self, _):
        if self.reference_drag is not None:
            self.reference_release()
            return
        if self.drag:
            self.history.append(self.drag_before)
            self.future.clear()
            self.drag, self.drag_before = None, None
            self.dirty = True
            self.topology['annotation']['manual_review_confirmed'] = False
            self.refresh_list()
            self.draw()

    def key(self, event):
        if event.key in ('ctrl+z', 'control+z'):
            self.undo()
        elif event.key in ('ctrl+shift+z', 'control+shift+z'):
            self.redo()
        elif event.key == 'enter' and self.mode in ('reference_row', 'reference_aisle'):
            self.finish_reference_line()
        elif event.key == 'enter' and self.mode == 'row_exclusion':
            self.finish_row_exclusion()
        elif event.key == 'enter' and self.mode in ('row', 'headland'):
            self.finish_geometry()
        elif event.key == 'escape':
            self.pending = []
            self.draw()
        elif event.key == 'delete':
            self.delete_reference() if self.active_reference() else self.delete_selected()

    def finish_geometry(self):
        minimum = 2 if self.mode == 'row' else 3
        if len(self.pending) < minimum:
            self.statusBar().showMessage(f'至少需要 {minimum} 个顶点。')
            return
        dialog = GeometryDialog(self.mode, {'centerline' if self.mode == 'row' else 'polygon': self.pending}, self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            try:
                item = dialog.value()
                collection = 'rows' if self.mode == 'row' else 'headlands'
                proposed = copy.deepcopy(self.topology)
                proposed[collection].append(item)
                result = validate_topology(proposed, self.package)
                if not result['valid']:
                    raise ValueError('; '.join(result['errors']))
                self.mutate(lambda: self.topology[collection].append(item))
                self.pending = []
                self.draw()
            except (ValueError, TypeError) as exc:
                QtWidgets.QMessageBox.warning(self, '实际几何形状无效', str(exc))

    def add_scene(self, scene):
        if not scene['scene_id'] or scene['scene_id'] in {s['scene_id'] for s in self.topology['scenes']}:
            QtWidgets.QMessageBox.warning(self, '场景编号', '请输入非空且唯一的场景编号。')
            return
        self.mutate(lambda: self.topology['scenes'].append(scene))

    def edit_selected(self):
        selected = self.selected()
        if not selected or not self.editable():
            return
        collection, index, item = selected
        if collection == 'scenes':
            pose = next(p for p in self.package.poses if p['keyframe'] == item['keyframe'])
            dialog = SceneDialog(self.topology, pose, item, self)
        else:
            dialog = GeometryDialog('row' if collection == 'rows' else 'headland', item, self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        try:
            replacement = dialog.value()
            proposed = copy.deepcopy(self.topology)
            proposed[collection][index] = replacement
            old_id, new_id = item.get('id'), replacement.get('id')
            if old_id != new_id and collection != 'scenes':
                key = 'manual_physical_row_id' if collection == 'rows' else 'manual_headland_id'
                for scene in proposed['scenes']:
                    if scene.get(key) == old_id:
                        scene[key] = new_id
            result = validate_topology(proposed, self.package)
            if not result['valid']:
                raise ValueError('; '.join(result['errors']))
            self.mutate(lambda: self.replace_topology(proposed))
        except (ValueError, TypeError) as exc:
            QtWidgets.QMessageBox.warning(self, '修改未通过校验', str(exc))

    def replace_topology(self, topology):
        self.topology = topology

    def delete_selected(self):
        selected = self.selected()
        if not selected or not self.editable():
            return
        collection, index, item = selected
        def deletion():
            del self.topology[collection][index]
            if collection != 'scenes':
                key = 'manual_physical_row_id' if collection == 'rows' else 'manual_headland_id'
                for scene in self.topology['scenes']:
                    if scene.get(key) == item['id']:
                        scene[key] = 'UNKNOWN'
        self.mutate(deletion)

    def undo(self):
        if self.pending:
            self.pending.pop()
            self.draw()
        elif self.workflow_history:
            self.reference_drag = None
            self.workflow_future.append(self._workflow_snapshot())
            self._restore_workflow(self.workflow_history.pop())

    def redo(self):
        if self.workflow_future:
            self.workflow_history.append(self._workflow_snapshot())
            self._restore_workflow(self.workflow_future.pop())

    def validate(self):
        result = validate_topology(self.topology, self.package)
        path = self.output.parent/'annotation_validation.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2))
        QtWidgets.QMessageBox.information(self, '拓扑校验结果', json.dumps(result, indent=2, ensure_ascii=False))
        return result

    def save(self, checked=False, interactive=True):
        del checked
        try:
            result = validate_topology(self.topology, self.package)
            if not result['valid']:
                raise ValueError('; '.join(result['errors']))
            save_topology(self.topology, self.output, self.package, self.other)
            self.output.parent.joinpath('annotation_validation.json').write_text(json.dumps(
                validate_topology(self.topology, self.package, self.output,
                    self.output.parent/'keyframe_topology_labels.csv'), indent=2))
            self.dirty = False
            if self.analysis is not None:
                self._save_proposal_review()
            self.statusBar().showMessage('已保存 YAML、GeoJSON、标签、场景建议和哈希清单：'+str(self.output))
            return True
        except (ValueError, OSError) as exc:
            if interactive:
                QtWidgets.QMessageBox.warning(self, '保存失败', str(exc))
            else:
                raise
            return False

    def freeze(self):
        if not self.editable():
            return
        candidate = copy.deepcopy(self.topology)
        candidate['annotation'].update(status='frozen', manual_review_confirmed=True,
                                       frozen_at=now_iso())
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            result = validate_topology(candidate, self.package, verify_map_files=True)
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        if not result['valid']:
            QtWidgets.QMessageBox.warning(self, '无法冻结', '; '.join(result['errors']))
            return
        response = QtWidgets.QMessageBox.question(self, '人工审查确认',
            '请确认所有实际垄线、地头和场景关联均已完成审查。\n'
            '冻结后的文件将不可修改；未知关联的位姿仍会保留。\n'
            f'未知关联覆盖率：{result.get("coverage", {}).get("unknown_percentage", 100):.1f}%')
        if response != QtWidgets.QMessageBox.Yes:
            return
        previous = self.topology
        self.topology = candidate
        if not self.save():
            self.topology = previous
        else:
            self.history, self.future = [], []
            self.draw()

    def new_draft(self):
        # Separate directory avoids accidentally overwriting frozen label sidecars.
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, '新建标注版本（请选择新目录）',
            str(self.output.parent.parent/(self.output.parent.name+'_v2')/self.output.name), '拓扑 YAML 文件 (*.yaml)')
        if not filename:
            return
        target = Path(filename).resolve()
        if target == self.output or target.exists() or target.parent == self.output.parent:
            QtWidgets.QMessageBox.warning(self, '版本路径', '请在新目录中选择一个新文件名。')
            return
        self.topology = copy.deepcopy(self.topology)
        self.topology['annotation'].update(status='draft', manual_review_confirmed=False,
            derived_from_file=str(self.output), derived_from_sha256=sha256_file(self.output) if self.output.exists() else None)
        self.topology['annotation'].pop('frozen_at', None)
        self.output = target
        self.history, self.future = [], []
        self.dirty = True
        self.draw()

    def closeEvent(self, event):
        if self.analysis_worker is not None and self.analysis_worker.isRunning():
            event.ignore()
            self.statusBar().showMessage('结构分析仍在运行，请等待完成后再关闭。')
            return
        if self.loader.isRunning():
            event.ignore()
            self.statusBar().showMessage('地图仍在加载，请等待加载完成后再关闭。')
            return
        if self.dirty and not self.args.smoke_output:
            answer = QtWidgets.QMessageBox.question(self, '尚有未保存的修改', '关闭前是否保存标注？',
                QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel)
            if answer == QtWidgets.QMessageBox.Cancel or (answer == QtWidgets.QMessageBox.Save and not self.save()):
                event.ignore()
                return
        event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-package', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--site-workspace')
    parser.add_argument('--structure-config')
    parser.add_argument('--other-map-package')
    parser.add_argument('--voxel', type=float, default=.2)
    parser.add_argument('--max-display-points', type=int, default=120000)
    parser.add_argument('--max-row-assignment-distance', type=float, default=1.0)
    parser.add_argument('--correspondence-max-dt', type=float, default=.5)
    parser.add_argument('--smoke-output', help='Save a real GUI map screenshot and exit after load')
    parser.add_argument('--smoke-save-empty', action='store_true', help='Save empty draft only; never creates physical rows')
    args = parser.parse_args(argv)
    if args.voxel <= 0 or args.max_display_points <= 0:
        parser.error('voxel and max-display-points must be positive')
    if args.smoke_save_empty and not args.smoke_output:
        parser.error('--smoke-save-empty requires --smoke-output')
    QLocale.setDefault(QLocale(QLocale.Chinese, QLocale.China))
    app = QtWidgets.QApplication(sys.argv[:1])
    translator = QTranslator(app)
    translations = QLibraryInfo.location(QLibraryInfo.TranslationsPath)
    if translator.load('qtbase_zh_CN', translations):
        app.installTranslator(translator)
    window = Annotator(args)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
