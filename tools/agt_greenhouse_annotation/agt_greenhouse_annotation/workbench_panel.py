"""Qt controls for reviewable greenhouse structure proposals."""
from __future__ import annotations

from PyQt5 import QtCore, QtWidgets


class EvidenceLayerPanel(QtWidgets.QWidget):
    """Visibility controls for map, structure evidence, and review layers."""

    layerVisibilityChanged = QtCore.pyqtSignal(str, bool)
    opacityChanged = QtCore.pyqtSignal(float)

    LAYERS = (
        ("point_cloud", "原始点云"),
        ("ground", "地面"),
        ("ground_confidence", "地面置信度"),
        ("slope", "坡度"),
        ("plane_residual", "平面残差"),
        ("terrain_ridge", "地形脊线"),
        ("terrain_depression", "地形凹陷"),
        ("terrain_step", "地形台阶"),
        ("global_row_support", "全局行结构证据"),
        ("global_row_proposal", "全局垄线提议"),
        ("local_row_observations", "局部垄线观测"),
        ("local_row_tracks", "局部垄线轨迹"),
        ("row_structural_band", "垄结构带"),
        ("aisle_geometry", "通道几何区域"),
        ("aisle_proposals", "通道提议"),
        ("geometric_centerline", "几何中心线"),
        ("safe_aisle", "安全通道"),
        ("safe_centerline", "安全中心线"),
        ("physical_row_labels", "人工确认的垄编号"),
        ("headland", "地头"),
        ("trajectory", "轨迹"),
        ("keyframes", "关键帧"),
    )

    def __init__(self, initial_visibility=None, parent=None):
        super().__init__(parent)
        initial_visibility = dict(initial_visibility or {})
        self.layer_checks: dict[str, QtWidgets.QCheckBox] = {}
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel(
            "显示或隐藏地图与证据图层。分析始终使用完整地图数据。"
        ))
        grid = QtWidgets.QGridLayout()
        for index, (name, label) in enumerate(self.LAYERS):
            checkbox = QtWidgets.QCheckBox(label)
            checkbox.setChecked(bool(initial_visibility.get(name, False)))
            checkbox.toggled.connect(
                lambda checked, layer=name: self.layerVisibilityChanged.emit(layer, checked)
            )
            self.layer_checks[name] = checkbox
            grid.addWidget(checkbox, index // 2, index % 2)
        layout.addLayout(grid)
        opacity_row = QtWidgets.QHBoxLayout()
        opacity_row.addWidget(QtWidgets.QLabel("叠加图层透明度"))
        self.opacity = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.opacity.setRange(10, 100)
        self.opacity.setValue(80)
        self.opacity.valueChanged.connect(
            lambda value: self.opacityChanged.emit(float(value) / 100.0)
        )
        opacity_row.addWidget(self.opacity)
        layout.addLayout(opacity_row)
        layout.addStretch(1)


class StructureProposalPanel(QtWidgets.QWidget):
    analysisRequested = QtCore.pyqtSignal(str)
    sourceChanged = QtCore.pyqtSignal(str)
    acceptRowsRequested = QtCore.pyqtSignal(object)
    rejectRowsRequested = QtCore.pyqtSignal(object)
    mergeRowsRequested = QtCore.pyqtSignal(object)
    splitRowRequested = QtCore.pyqtSignal(int, float)
    editRowRequested = QtCore.pyqtSignal(int)
    deleteRowsRequested = QtCore.pyqtSignal(object)
    resetRequested = QtCore.pyqtSignal()
    undoRequested = QtCore.pyqtSignal()
    redoRequested = QtCore.pyqtSignal()
    acceptAisleRequested = QtCore.pyqtSignal(int)
    rejectAisleRequested = QtCore.pyqtSignal(int)
    freezeRequested = QtCore.pyqtSignal()
    rowSelectionChanged = QtCore.pyqtSignal(object)
    aisleSelectionChanged = QtCore.pyqtSignal(int)

    referenceReviewed = QtCore.pyqtSignal(str, int, bool)
    focusRequested = QtCore.pyqtSignal()
    deleteReferenceRequested = QtCore.pyqtSignal()
    clearExclusionsRequested = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        title = QtWidgets.QLabel('参考垄线与行道')
        title.setStyleSheet('font-size: 18px; font-weight: bold;')
        layout.addWidget(title)
        self.run_button = QtWidgets.QPushButton('一键生成垄线和中心线')
        self.run_button.setMinimumHeight(38)
        self.run_button.clicked.connect(lambda: self.analysisRequested.emit('COMPARE'))
        layout.addWidget(self.run_button)
        self.clear_exclusions = QtWidgets.QPushButton('清除之前误圈的墙壁区域')
        self.clear_exclusions.clicked.connect(self.clearExclusionsRequested.emit)
        self.clear_exclusions.hide()
        layout.addWidget(self.clear_exclusions)
        # Kept for the existing analysis API, away from the primary workflow.
        self.mode = QtWidgets.QComboBox(self)
        self.mode.addItems(['GLOBAL_PROFILE', 'LOCAL_TRACKS', 'COMPARE'])
        self.mode.hide()
        self.source = QtWidgets.QComboBox(self)
        self.source.addItem('全局参考', 'GLOBAL_PROFILE')
        self.source.addItem('局部参考', 'LOCAL_TRACKS')
        self.source.currentIndexChanged.connect(lambda: self.sourceChanged.emit(self.source.currentData()))
        self.source.hide()
        self.object_tabs = QtWidgets.QTabWidget()
        self.row_list = QtWidgets.QListWidget()
        self.row_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.row_list.itemSelectionChanged.connect(self._show_row_diagnostics)
        self.row_list.itemChanged.connect(lambda item: self.referenceReviewed.emit(
            'row', self.row_list.row(item), item.checkState() == QtCore.Qt.Checked))
        self.aisle_list = QtWidgets.QListWidget()
        self.aisle_list.currentRowChanged.connect(self._show_aisle_diagnostics)
        self.aisle_list.itemChanged.connect(lambda item: self.referenceReviewed.emit(
            'aisle', self.aisle_list.row(item), item.checkState() == QtCore.Qt.Checked))
        self.object_tabs.addTab(self.row_list, '垄线')
        self.object_tabs.addTab(self.aisle_list, '行道中心线')
        layout.addWidget(self.object_tabs, 1)
        self.selection_info = QtWidgets.QLabel('点击列表或图中的线选择；选中对象以白色高亮。')
        self.selection_info.setWordWrap(True)
        layout.addWidget(self.selection_info)
        focus = QtWidgets.QPushButton('定位到所选对象')
        focus.clicked.connect(self.focusRequested.emit)
        layout.addWidget(focus)
        actions = QtWidgets.QHBoxLayout()
        delete = QtWidgets.QPushButton('删除所选')
        delete.clicked.connect(self.deleteReferenceRequested.emit)
        actions.addWidget(delete)
        undo = QtWidgets.QPushButton('撤销')
        undo.clicked.connect(self.undoRequested.emit)
        actions.addWidget(undo)
        redo = QtWidgets.QPushButton('重做')
        redo.clicked.connect(self.redoRequested.emit)
        actions.addWidget(redo)
        layout.addLayout(actions)
        hint = QtWidgets.QLabel('二维：点线选择，拖动顶点或线段编辑。\n双击线段加顶点，右键顶点删除。\n逐条勾选审查完成，最后导出。')
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.freeze_button = QtWidgets.QPushButton('导出已审查参考线')
        self.freeze_button.setMinimumHeight(38)
        self.freeze_button.clicked.connect(self.freezeRequested.emit)
        layout.addWidget(self.freeze_button)
        self.status = QtWidgets.QLabel('点击一键生成开始；编辑会自动保存草稿。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.row_diagnostics = QtWidgets.QPlainTextEdit(self)
        self.aisle_diagnostics = QtWidgets.QPlainTextEdit(self)
        self.row_diagnostics.hide()
        self.aisle_diagnostics.hide()
        self._rows, self._aisles = [], []

    def set_busy(self, busy: bool):
        self.run_button.setEnabled(not busy)
        self.status.setText('正在生成参考线，完成后可直接编辑…' if busy else self.status.text())

    def set_rows(self, rows):
        selected_ids = {self._rows[index].auto_id for index in self.selected_row_indices() if index < len(self._rows)}
        self._rows = list(rows)
        self.row_list.blockSignals(True)
        self.row_list.clear()
        for index, row in enumerate(self._rows):
            label = f'垄 {index+1:02d} · {row.auto_id}'
            if row.decision == 'rejected':
                label += '（已删除）'
            item = QtWidgets.QListWidgetItem(label)
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(QtCore.Qt.Checked if row.diagnostics.get('reference_reviewed') else QtCore.Qt.Unchecked)
            self.row_list.addItem(item)
            item.setSelected(row.auto_id in selected_ids)
        self.row_list.blockSignals(False)
        self._show_row_diagnostics()

    def set_aisles(self, aisles):
        selected = self.aisle_list.currentRow()
        selected_id = self._aisles[selected].auto_id if 0 <= selected < len(self._aisles) else None
        self._aisles = list(aisles)
        self.aisle_list.blockSignals(True)
        self.aisle_list.clear()
        new_index = -1
        for index, aisle in enumerate(self._aisles):
            item = QtWidgets.QListWidgetItem(f'行道 {index+1:02d} · {aisle.auto_id}')
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(QtCore.Qt.Checked if aisle.diagnostics.get('reference_reviewed') else QtCore.Qt.Unchecked)
            self.aisle_list.addItem(item)
            if aisle.auto_id == selected_id:
                new_index = index
        self.aisle_list.setCurrentRow(new_index)
        self.aisle_list.blockSignals(False)
        self._show_aisle_diagnostics()

    def selected_row_indices(self):
        return [self.row_list.row(item) for item in self.row_list.selectedItems()]

    def selected_aisle_index(self):
        return self.aisle_list.currentRow()

    def _show_row_diagnostics(self):
        index = self.row_list.currentRow()
        self.rowSelectionChanged.emit(self.selected_row_indices())
        if 0 <= index < len(self._rows):
            from dataclasses import asdict
            import json
            self.row_diagnostics.setPlainText(json.dumps(asdict(self._rows[index]), indent=2, ensure_ascii=False))

    def _show_aisle_diagnostics(self, index=None):
        index = self.aisle_list.currentRow()
        self.aisleSelectionChanged.emit(index)
        if 0 <= index < len(self._aisles):
            from dataclasses import asdict
            import json
            self.aisle_diagnostics.setPlainText(json.dumps(asdict(self._aisles[index]), indent=2, ensure_ascii=False))

    def _accept_rows(self):
        self.acceptRowsRequested.emit(self.selected_row_indices())

    def _reject_rows(self):
        self.rejectRowsRequested.emit(self.selected_row_indices())

    def _merge_rows(self):
        self.mergeRowsRequested.emit(self.selected_row_indices())

    def _split_row(self):
        indices = self.selected_row_indices()
        if len(indices) != 1:
            return
        fraction, ok = QtWidgets.QInputDialog.getDouble(self, '拆分垄线提议',
                                                        '纵向拆分比例', .5, .05, .95, 2)
        if ok:
            self.splitRowRequested.emit(indices[0], fraction)

    def _edit_row(self):
        indices = self.selected_row_indices()
        if len(indices) != 1 or indices[0] >= len(self._rows):
            return
        self.editRowRequested.emit(indices[0])

    def _delete_rows(self):
        self.deleteRowsRequested.emit(self.selected_row_indices())

    def _accept_aisle(self):
        index = self.selected_aisle_index()
        if index >= 0:
            self.acceptAisleRequested.emit(index)

    def _reject_aisle(self):
        index = self.selected_aisle_index()
        if index >= 0:
            self.rejectAisleRequested.emit(index)
