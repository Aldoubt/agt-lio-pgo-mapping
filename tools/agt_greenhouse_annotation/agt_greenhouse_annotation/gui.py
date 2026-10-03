"""PyQt5 top-down manual row/headland editor for existing map packages."""
import argparse
import copy
import json
import math
from pathlib import Path
import sys

import numpy as np
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

from .topology import (align_backend, label_pose, load_map_package, load_topology,
                       new_topology, now_iso, save_topology, sha256_file)
from .validator import validate_topology


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


class GeometryDialog(QtWidgets.QDialog):
    def __init__(self, kind, item, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Manual physical '+kind)
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
        self.direction.addItems(['bidirectional', 'forward', 'reverse', 'unknown'])
        self.direction.setCurrentText(item.get('direction', 'bidirectional'))
        self.confidence = QtWidgets.QComboBox()
        self.confidence.addItems(['confirmed', 'unconfirmed'])
        self.confidence.setCurrentText(item.get('confidence', 'confirmed'))
        self.notes = QtWidgets.QPlainTextEdit(item.get('notes', ''))
        self.notes.setMaximumHeight(80)
        form.addRow('Physical ID (enter manually)', self.identifier)
        form.addRow('XY vertices [metres]', self.vertices)
        if kind == 'row':
            form.addRow('Nominal width [m]', self.width)
            form.addRow('Allowed travel direction', self.direction)
        form.addRow('Manual confidence', self.confidence)
        form.addRow('Notes', self.notes)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def value(self):
        vertices = json.loads(self.vertices.toPlainText())
        item = dict(id=self.identifier.text().strip(), confidence=self.confidence.currentText(),
                    notes=self.notes.toPlainText())
        item['centerline' if self.kind == 'row' else 'polygon'] = vertices
        if self.kind == 'row':
            item.update(nominal_width_m=self.width.value(), direction=self.direction.currentText())
        return item


class SceneDialog(QtWidgets.QDialog):
    def __init__(self, topology, pose, scene=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Manual scene marker')
        self.pose, self.scene = pose, scene or {}
        form = QtWidgets.QFormLayout(self)
        self.identifier = QtWidgets.QLineEdit(self.scene.get('scene_id', ''))
        self.scene_type = QtWidgets.QComboBox()
        self.scene_type.addItems(['ROW_ENTRY', 'ROW_MIDDLE', 'ROW_END', 'HEADLAND', 'OTHER'])
        self.scene_type.setCurrentText(self.scene.get('scene_type', 'ROW_MIDDLE'))
        self.row = QtWidgets.QComboBox()
        self.row.addItems(['UNKNOWN']+[r['id'] for r in topology['rows']])
        self.row.setCurrentText(self.scene.get('manual_physical_row_id', 'UNKNOWN'))
        self.headland = QtWidgets.QComboBox()
        self.headland.addItems(['UNKNOWN']+[h['id'] for h in topology['headlands']])
        self.headland.setCurrentText(self.scene.get('manual_headland_id', 'UNKNOWN'))
        self.notes = QtWidgets.QLineEdit(self.scene.get('notes', ''))
        label = label_pose(topology, pose['x'], pose['y'], pose['yaw_rad'])
        projected = label['physical_row_id']
        text = (f"keyframe={pose['keyframe']}\nbag timestamp={pose['timestamp']:.9f}\n"
                f"x/y={pose['x']:.3f}/{pose['y']:.3f} m, yaw={pose['yaw_deg']:.2f} deg\n"
                f"corridor row={projected}, s={label['along_row_s_m']}, d={label['lateral_d_m']}\n"
                'Manual association is explicit; UNKNOWN is allowed.')
        info = QtWidgets.QLabel(text)
        info.setWordWrap(True)
        form.addRow(info)
        form.addRow('Scene ID (enter manually)', self.identifier)
        form.addRow('Scene state', self.scene_type)
        form.addRow('Confirmed physical row', self.row)
        form.addRow('Confirmed headland', self.headland)
        form.addRow('Notes', self.notes)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def value(self):
        pose = self.pose
        return dict(scene_id=self.identifier.text().strip(), scene_type=self.scene_type.currentText(),
                    keyframe=pose['keyframe'], bag_timestamp=pose['timestamp'],
                    x=pose['x'], y=pose['y'], yaw_rad=pose['yaw_rad'],
                    manual_physical_row_id=self.row.currentText(),
                    manual_headland_id=self.headland.currentText(),
                    notes=self.notes.text(), confirmation='manual')


class Annotator(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
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
        self.pending, self.mode = [], 'inspect'
        self.drag, self.drag_before = None, None
        self.dirty = False
        self.setWindowTitle('AGT Greenhouse topology — '+self.package.backend_id)
        self.resize(1360, 900)
        self.figure = Figure(figsize=(10, 7))
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.axes = self.figure.add_subplot(111)
        self.list = QtWidgets.QListWidget()
        self.list.currentRowChanged.connect(self.select_item)
        self.info = QtWidgets.QLabel('Loading downsampled map in background…')
        self.info.setWordWrap(True)
        self.info.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.help = QtWidgets.QLabel('Row/polygon: left click vertices; right click/Enter finishes.\n'
            'Scene: click trajectory. Edit: drag a selected vertex or click Edit details.\n'
            'Physical IDs must be entered and confirmed by a person.\n'
            'Coordinates: manual topology + same-session frontend; absolute GT=false.')
        self.help.setWordWrap(True)
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(['Inspect', 'Draw row centerline', 'Draw headland polygon', 'Manual scene marker', 'Edit vertices'])
        self.mode_combo.currentIndexChanged.connect(self.change_mode)
        self.distance = QtWidgets.QDoubleSpinBox()
        self.distance.setRange(.01, 100)
        self.distance.setValue(self.topology['assignment']['max_row_assignment_distance_m'])
        self.distance.valueChanged.connect(self.assignment_changed)
        side = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(side)
        layout.addWidget(self.mode_combo)
        layout.addWidget(self.help)
        layout.addWidget(QtWidgets.QLabel('Max row assignment distance [m]'))
        layout.addWidget(self.distance)
        layout.addWidget(self.list, 2)
        for text, callback in [('Edit details / vertices', self.edit_selected), ('Delete selected', self.delete_selected),
                                ('Undo (Ctrl+Z)', self.undo), ('Redo (Ctrl+Shift+Z)', self.redo),
                                ('Save draft (Ctrl+S)', self.save), ('Validate', self.validate),
                                ('Freeze after manual review', self.freeze), ('New draft version', self.new_draft)]:
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(callback)
            layout.addWidget(button)
        layout.addWidget(self.info, 1)
        self.side = side
        center = QtWidgets.QWidget()
        center_layout = QtWidgets.QVBoxLayout(center)
        center_layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        center_layout.addWidget(self.canvas)
        split = QtWidgets.QSplitter()
        split.addWidget(center)
        split.addWidget(side)
        split.setSizes([1000, 360])
        self.setCentralWidget(split)
        QtWidgets.QShortcut('Ctrl+Z', self, activated=self.undo)
        QtWidgets.QShortcut('Ctrl+Shift+Z', self, activated=self.redo)
        QtWidgets.QShortcut('Ctrl+S', self, activated=self.save)
        self.canvas.mpl_connect('button_press_event', self.click)
        self.canvas.mpl_connect('motion_notify_event', self.motion)
        self.canvas.mpl_connect('button_release_event', self.release)
        self.canvas.mpl_connect('key_press_event', self.key)
        self.refresh_list()
        self.draw(reset=True)
        self.loader = CloudLoader(self.package.path/'map.pcd', args.voxel, args.max_display_points)
        self.loader.loaded.connect(self.cloud_loaded)
        self.loader.failed.connect(self.cloud_failed)
        self.loader.start()
        self.statusBar().showMessage(str(self.output))

    def editable(self):
        if self.topology['annotation']['status'] == 'frozen':
            self.statusBar().showMessage('Frozen annotation is read-only. Create a new draft version to edit.')
            return False
        return True

    def mutate(self, callback):
        if not self.editable():
            return
        before = copy.deepcopy(self.topology)
        callback()
        self.history.append(before)
        self.future.clear()
        self.topology['annotation']['manual_review_confirmed'] = False
        self.dirty = True
        self.refresh_list()
        self.draw()

    def cloud_loaded(self, cloud, total):
        self.cloud = cloud
        self.info.setText(f'Map: {total:,} points → {len(cloud):,} displayed\n'
                          f'{len(self.package.poses)} keyframes\nFrame: {self.package.map_frame}\n'
                          f'Canonical backend: {self.package.backend_id}')
        if self.other:
            residual = self.topology['backend_transforms'][self.other.backend_id]['residual_xy_m']
            self.info.setText(self.info.text()+f'\nOther backend SE2 residual median/P95: {residual["median"]:.3f}/{residual["p95"]:.3f} m')
        self.draw(reset=True)
        if self.args.smoke_output:
            if self.args.smoke_save_empty:
                if self.topology['rows'] or self.topology['headlands'] or self.topology['scenes']:
                    self.cloud_failed('--smoke-save-empty refuses existing physical annotations')
                    return
                self.save(interactive=False)
            QtCore.QTimer.singleShot(200, self.finish_smoke)

    def finish_smoke(self):
        self.figure.savefig(self.args.smoke_output, dpi=150)
        screenshot = Path(self.args.smoke_output)
        self.grab().save(str(screenshot.with_name(screenshot.stem+'_window.png')))
        print(json.dumps(dict(map_package=str(self.package.path), output=str(self.output),
            screenshot=self.args.smoke_output, rows=len(self.topology['rows']), headlands=len(self.topology['headlands']),
            scene_markers=len(self.topology['scenes']), display_points=len(self.cloud),
            annotation_status=self.topology['annotation']['status'])), flush=True)
        self.dirty = False
        self.close()

    def cloud_failed(self, reason):
        self.info.setText('Map load failed: '+reason)
        if self.args.smoke_output:
            print('GUI smoke failed: '+reason, file=sys.stderr, flush=True)
            QtWidgets.QApplication.exit(2)
        else:
            QtWidgets.QMessageBox.critical(self, 'Map load', reason)

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
            self.list.addItem(collection+': '+name)
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

    def draw(self, reset=False):
        xlim, ylim = self.axes.get_xlim(), self.axes.get_ylim()
        self.axes.clear()
        if self.cloud is not None:
            self.axes.scatter(self.cloud[:, 0], self.cloud[:, 1], s=.25, c='#9ba0a4', alpha=.45, rasterized=True)
        xy = np.array([[p['x'], p['y']] for p in self.package.poses])
        self.axes.plot(xy[:, 0], xy[:, 1], color='#315b89', lw=.8, label='frontend trajectory')
        self.axes.scatter(xy[:, 0], xy[:, 1], s=4, c='#315b89', alpha=.7, label='keyframes')
        selected = self.selected()
        for collection, index, item in self.entries():
            highlight = selected is not None and selected[:2] == (collection, index)
            if collection == 'rows':
                points = np.array(item['centerline'])
                self.axes.plot(points[:, 0], points[:, 1], '-o', ms=5 if highlight else 2,
                               lw=2 if highlight else 1.3, color='#d65032')
                self.axes.annotate(item['id'], points[len(points)//2], color='#a3361d')
            elif collection == 'headlands':
                points = np.array(item['polygon'])
                self.axes.fill(points[:, 0], points[:, 1], color='#3ba76d', alpha=.17)
                points = np.vstack([points, points[0]])
                self.axes.plot(points[:, 0], points[:, 1], '-o', ms=5 if highlight else 2, color='#248855')
                self.axes.annotate(item['id'], points.mean(axis=0), color='#156238')
            else:
                self.axes.scatter([item['x']], [item['y']], s=60 if highlight else 25, marker='*', color='#8735a6')
                self.axes.annotate(item['scene_id'], (item['x'], item['y']), fontsize=8)
        if self.pending:
            points = np.array(self.pending)
            self.axes.plot(points[:, 0], points[:, 1], 'o--', color='#db9933')
        self.axes.set_aspect('equal', adjustable='box')
        self.axes.set_xlabel(f'x [m] — {self.package.map_frame}')
        self.axes.set_ylabel('y [m]')
        self.axes.set_title(f'{self.package.backend_id} | {self.topology["annotation"]["status"]} | manual topology + same-session frontend')
        if not reset:
            self.axes.set_xlim(xlim)
            self.axes.set_ylim(ylim)
        self.figure.tight_layout()
        self.canvas.draw_idle()

    def change_mode(self, index):
        self.mode = ['inspect', 'row', 'headland', 'scene', 'edit'][index]
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
        if self.mode in ('row', 'headland') and self.editable():
            if event.button == 3 or event.dblclick:
                self.finish_geometry()
            elif event.button == 1:
                self.pending.append([float(event.xdata), float(event.ydata)])
                self.draw()
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
            self.info.setText(f'keyframe {pose["keyframe"]}\nbag timestamp {pose["timestamp"]:.9f}\n'
                f'x={pose["x"]:.3f} y={pose["y"]:.3f} yaw={pose["yaw_deg"]:.2f} deg\n'+json.dumps(label, indent=2))

    def motion(self, event):
        if self.drag and event.inaxes == self.axes and event.xdata is not None:
            collection, index, geometry, vertex = self.drag
            self.topology[collection][index][geometry][vertex] = [float(event.xdata), float(event.ydata)]
            self.draw()

    def release(self, _):
        if self.drag:
            self.history.append(self.drag_before)
            self.future.clear()
            self.drag, self.drag_before = None, None
            self.dirty = True
            self.topology['annotation']['manual_review_confirmed'] = False
            self.refresh_list()
            self.draw()

    def key(self, event):
        if event.key == 'enter' and self.mode in ('row', 'headland'):
            self.finish_geometry()
        elif event.key == 'escape':
            self.pending = []
            self.draw()
        elif event.key == 'delete':
            self.delete_selected()

    def finish_geometry(self):
        minimum = 2 if self.mode == 'row' else 3
        if len(self.pending) < minimum:
            self.statusBar().showMessage(f'At least {minimum} vertices required')
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
                QtWidgets.QMessageBox.warning(self, 'Invalid physical geometry', str(exc))

    def add_scene(self, scene):
        if not scene['scene_id'] or scene['scene_id'] in {s['scene_id'] for s in self.topology['scenes']}:
            QtWidgets.QMessageBox.warning(self, 'Scene ID', 'Enter a unique nonempty scene ID')
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
            QtWidgets.QMessageBox.warning(self, 'Edit rejected', str(exc))

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
        if self.history and self.editable():
            self.future.append(copy.deepcopy(self.topology))
            self.topology = self.history.pop()
            self.dirty = True
            self.refresh_list()
            self.draw()

    def redo(self):
        if self.future and self.editable():
            self.history.append(copy.deepcopy(self.topology))
            self.topology = self.future.pop()
            self.dirty = True
            self.refresh_list()
            self.draw()

    def validate(self):
        result = validate_topology(self.topology, self.package)
        path = self.output.parent/'annotation_validation.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2))
        QtWidgets.QMessageBox.information(self, 'Topology validation', json.dumps(result, indent=2))
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
            self.statusBar().showMessage('Saved YAML, GeoJSON, labels, scene suggestions and hash manifest: '+str(self.output))
            return True
        except (ValueError, OSError) as exc:
            if interactive:
                QtWidgets.QMessageBox.warning(self, 'Save failed', str(exc))
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
            QtWidgets.QMessageBox.warning(self, 'Cannot freeze', '; '.join(result['errors']))
            return
        response = QtWidgets.QMessageBox.question(self, 'Manual review',
            'Confirm every physical row/headland and scene association has been reviewed.\n'
            'The frozen artifact becomes immutable. UNKNOWN poses are retained.\n'
            f'UNKNOWN coverage: {result.get("coverage", {}).get("unknown_percentage", 100):.1f}%')
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
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'New annotation version (use a new directory)',
            str(self.output.parent.parent/(self.output.parent.name+'_v2')/self.output.name), 'Topology YAML (*.yaml)')
        if not filename:
            return
        target = Path(filename).resolve()
        if target == self.output or target.exists() or target.parent == self.output.parent:
            QtWidgets.QMessageBox.warning(self, 'Version path', 'Choose a new filename in a new directory')
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
        if self.loader.isRunning():
            event.ignore()
            self.statusBar().showMessage('Map load is still running; close after load completes')
            return
        if self.dirty and not self.args.smoke_output:
            answer = QtWidgets.QMessageBox.question(self, 'Unsaved changes', 'Save annotation before closing?',
                QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel)
            if answer == QtWidgets.QMessageBox.Cancel or (answer == QtWidgets.QMessageBox.Save and not self.save()):
                event.ignore()
                return
        event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-package', required=True)
    parser.add_argument('--output', required=True)
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
    app = QtWidgets.QApplication(sys.argv[:1])
    window = Annotator(args)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
