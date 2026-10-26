"""Optional Qt review flow connecting structure evidence to coverage authoring."""
from dataclasses import replace
from pathlib import Path

import numpy as np
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from shapely.geometry import shape

from .coverage_export import freeze_and_export_coverage, reviewed_corridor


class CoverageReviewMixin:
    def review_coverage_aisle(self, index):
        if not 0 <= index < len(self.aisle_proposals):
            return
        aisle = self.aisle_proposals[index]
        by_id = {row.auto_id: row for row in self.row_proposals}
        neighbors = ([aisle.left_row_auto_id, aisle.right_row_auto_id]
                     if aisle.aisle_kind == 'INTERIOR' else
                     [aisle.left_row_auto_id] if aisle.aisle_kind == 'BOUNDARY' else [])
        if any(identity not in by_id or by_id[identity].decision != 'accepted' for identity in neighbors):
            self.structure_panel.status.setText('请先确认相邻作物垄参考编号；参考垄中心线不会作为行驶路线。')
            return
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle('单独审查可通行行道')
        layout = QtWidgets.QVBoxLayout(dialog)
        form = QtWidgets.QFormLayout()
        identity = QtWidgets.QLineEdit(aisle.physical_aisle_id or '')
        identity.setObjectName('coverageAisleId')
        width = QtWidgets.QDoubleSpinBox()
        width.setRange(.01, 50.)
        width.setObjectName('coverageAisleWidth')
        width.setDecimals(3)
        previous = aisle.diagnostics.get('coverage_review', {})
        width.setValue(previous.get('width_m', max(.01, aisle.geometric_width_m)))
        direction = QtWidgets.QComboBox()
        for label, key in [('双向', 'bidirectional'), ('按中心线顶点顺序', 'along'), ('逆中心线顶点顺序', 'against')]:
            direction.addItem(label, key)
        direction.setCurrentIndex(max(0, direction.findData(previous.get('allowed_direction'))))
        form.addRow('实际行道编号（必填）', identity)
        form.addRow('人工确认的通道宽度 [m]', width)
        form.addRow('允许行驶方向', direction)
        layout.addLayout(form)
        endpoint_hint = QtWidgets.QLabel('绿色宽度带止于中心线端点。请另绘制并在导出时选择两端地头，覆盖车体伸出部分及转弯空间；软件不会自动扩大已审查区域。')
        endpoint_hint.setWordWrap(True)
        layout.addWidget(endpoint_hint)
        figure = Figure(figsize=(7, 4))
        canvas = FigureCanvasQTAgg(figure)
        layout.addWidget(canvas)
        confirm = QtWidgets.QCheckBox('已审查绿色通道边界与中心线；这里是行驶空间，不是作物垄')
        confirm.setObjectName('coverageAisleReviewConfirmed')
        layout.addWidget(confirm)

        def preview():
            figure.clear()
            ax = figure.add_subplot(111)
            try:
                review = reviewed_corridor(aisle.centerline_xy, width.value(),
                                          direction.currentData(), confirmed=True)
                polygon = np.asarray(shape(review['corridor']).exterior.coords)
                line = np.asarray(aisle.centerline_xy)
                if self.cloud is not None:
                    points = self.display_cloud()
                    low, high = polygon.min(axis=0)-1., polygon.max(axis=0)+1.
                    points = points[((points[:, :2] >= low) & (points[:, :2] <= high)).all(axis=1)]
                    step = max(1, len(points)//20000)
                    ax.scatter(points[::step, 0], points[::step, 1], s=.3, c='#777777')
                ax.fill(polygon[:, 0], polygon[:, 1], alpha=.25, color='green')
                ax.plot(line[:, 0], line[:, 1], '-o', color='blue', markersize=3)
                ax.annotate('START', line[0])
                ax.set_aspect('equal', adjustable='datalim')
                ax.set_xlabel(f'{self.package.map_frame} x [m]')
                ax.set_ylabel('y [m]')
                confirm.setChecked(False)
            except ValueError as exc:
                ax.set_title(str(exc))
            canvas.draw_idle()

        width.valueChanged.connect(lambda _: preview())
        direction.currentIndexChanged.connect(lambda _: preview())
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        preview()
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        try:
            value = identity.text().strip()
            if not value or any(i != index and item.physical_aisle_id == value
                                for i, item in enumerate(self.aisle_proposals)):
                raise ValueError('请输入非空且唯一的实际行道编号')
            review = reviewed_corridor(aisle.centerline_xy, width.value(), direction.currentData(),
                                       confirmed=confirm.isChecked())
            self._record_workflow_state()
            self.aisle_proposals[index] = replace(aisle, decision='accepted', physical_aisle_id=value,
                geometric_width_m=width.value(), diagnostics={**dict(aisle.diagnostics),
                    'coverage_review': review, 'reference_reviewed': True})
            self._refresh_proposal_panel()
            self._save_reference_draft()
        except ValueError as exc:
            self.structure_panel.status.setText(str(exc))

    def export_coverage_dialog(self):
        if self.topology.get('coverage_boundary', {}).get('confidence') != 'confirmed':
            self.structure_panel.status.setText('请先绘制并确认温室作业边界，再审查行道并导出。')
            return
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle('冻结结构并导出覆盖规划输入')
        form = QtWidgets.QFormLayout(dialog)
        map_path = QtWidgets.QLineEdit()
        map_path.setObjectName('coverageMapYaml')
        output = QtWidgets.QLineEdit(str(self._site_workspace() / 'annotation' / 'coverage_v1'))
        output.setObjectName('coverageOutputDirectory')

        def picker(field, title, directory=False):
            widget = QtWidgets.QWidget()
            row = QtWidgets.QHBoxLayout(widget)
            row.setContentsMargins(0, 0, 0, 0)
            row.addWidget(field)
            button = QtWidgets.QPushButton('选择…')
            def choose():
                selected = (QtWidgets.QFileDialog.getExistingDirectory(dialog, title, field.text())
                            if directory else QtWidgets.QFileDialog.getOpenFileName(dialog, title, field.text(), 'YAML (*.yaml *.yml)')[0])
                if selected:
                    field.setText(selected)
            button.clicked.connect(choose)
            row.addWidget(button)
            return widget

        form.addRow('已审查地图 YAML', picker(map_path, '选择地图 YAML'))
        form.addRow('新的版本目录', picker(output, '选择输出版本目录', True))
        form.addRow('源坐标系', QtWidgets.QLabel(self.package.map_frame))
        transform = []
        for label in ('map←source dx [m]', 'map←source dy [m]', 'map←source yaw [rad]'):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-100000., 100000.)
            spin.setDecimals(6)
            spin.setEnabled(self.package.map_frame != 'map')
            form.addRow(label, spin)
            transform.append(spin)
        transform_confirm = QtWidgets.QCheckBox('已核对源坐标系到 map 的变换（零变换也需确认）')
        transform_confirm.setObjectName('coverageTransformConfirmed')
        transform_confirm.setEnabled(self.package.map_frame != 'map')
        form.addRow(transform_confirm)
        headlands = QtWidgets.QListWidget()
        for item in self.topology.get('headlands', []):
            entry = QtWidgets.QListWidgetItem(item['id'])
            entry.setFlags(entry.flags() | QtCore.Qt.ItemIsUserCheckable)
            entry.setCheckState(QtCore.Qt.Unchecked)
            headlands.addItem(entry)
        form.addRow('选作连接空间的地头', headlands)
        endpoint_hint = QtWidgets.QLabel('请勾选覆盖起点和终点整车包络的已审查地头。未选或空间不足时，规划会保留未覆盖行道并报告 AISLE_TOO_NARROW；不会将未知端外空间当作可通行。')
        endpoint_hint.setWordWrap(True)
        form.addRow(endpoint_hint)
        review = QtWidgets.QCheckBox('已审查边界、所选地头和全部已接受行道；不包含作物垄中心线')
        review.setObjectName('coverageStructureReviewConfirmed')
        form.addRow(review)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        try:
            if not review.isChecked() or not map_path.text().strip() or not output.text().strip():
                raise ValueError('请选择地图、新版本目录并明确确认结构审查')
            target = Path(output.text()).expanduser().resolve()
            result = freeze_and_export_coverage(self.topology, self.row_proposals, self.aisle_proposals,
                target / 'annotation' / 'greenhouse_topology.yaml', map_path.text(), target / 'coverage_structure.geojson',
                package=self.package,
                map_from_reference=[spin.value() for spin in transform] if self.package.map_frame != 'map' else None,
                transform_review_confirmed=transform_confirm.isChecked(),
                headland_ids=[headlands.item(i).text() for i in range(headlands.count())
                              if headlands.item(i).checkState() == QtCore.Qt.Checked])
            self.structure_panel.status.setText(f'已冻结结构并导出：{result}。在覆盖路线窗口选择此 GeoJSON。')
        except (ValueError, OSError) as exc:
            self.structure_panel.status.setText('覆盖结构未导出：' + str(exc))
