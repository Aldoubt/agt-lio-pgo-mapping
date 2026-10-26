"""Selection, direct geometry editing and unified history for reference workbench."""
import copy
from dataclasses import replace
from pathlib import Path

import numpy as np
from PyQt5 import QtCore, QtWidgets

from .reference_review import save_reference_draft, export_reviewed_reference
from .structure_artifacts import GlobalRowProposal, AisleProposal


class ReferenceEditorMixin:
    def _workflow_snapshot(self):
        return {
            'topology': copy.deepcopy(self.topology), 'dirty': self.dirty,
            'rows': copy.deepcopy(self.row_proposals), 'aisles': copy.deepcopy(self.aisle_proposals),
            'sources': copy.deepcopy(self.rows_by_source), 'source': self.active_proposal_source,
            'boundaries': copy.deepcopy(self.manual_boundary_aisles), 'analysis': self.analysis,
            'walls': copy.deepcopy(self.row_exclusion_polygons), 'walls_enabled': self.wall_exclusions_enabled,
        }

    def _record_workflow_state(self):
        self.workflow_history.append(self._workflow_snapshot())
        self.workflow_future.clear()

    def _restore_workflow(self, state):
        self.topology = state['topology']
        self.row_proposals, self.aisle_proposals = state['rows'], state['aisles']
        self.rows_by_source, self.active_proposal_source = state['sources'], state['source']
        self.manual_boundary_aisles, self.analysis = state['boundaries'], state['analysis']
        self.row_exclusion_polygons, self.wall_exclusions_enabled = state['walls'], state['walls_enabled']
        self.save_row_exclusions()
        self.review3d.canvas.clear_analysis()
        if getattr(self.analysis, 'navigation', None) is not None:
            self.review3d.set_analysis(self.analysis.navigation,
                getattr(self.analysis, 'corridor_result', None), getattr(self.analysis, 'geometric_aisle_result', None))
        self.refresh_list()
        self._refresh_proposal_panel()
        self._save_reference_draft()
        self.dirty = state['dirty']

    def _save_reference_draft(self):
        try:
            save_reference_draft(self.reference_draft_path, self.package, self.analysis,
                self.row_proposals, self.aisle_proposals)
        except OSError as exc:
            self.structure_panel.status.setText(f'草稿保存失败：{exc}')

    def active_reference(self):
        if self._selection_kind == 'aisle' and 0 <= self.selected_aisle_index < len(self.aisle_proposals):
            return 'aisle', self.selected_aisle_index, self.aisle_proposals[self.selected_aisle_index]
        indices = sorted(self.selected_proposal_indices)
        if indices and 0 <= indices[0] < len(self.row_proposals):
            return 'row', indices[0], self.row_proposals[indices[0]]
        return None

    def update_selection_description(self):
        selected = self.active_reference()
        if not selected:
            self.structure_panel.selection_info.setText('点击列表或图中的线选择；白色高亮为选中对象。')
            return
        kind, index, item = selected
        xy = np.asarray(item.centerline_xy)
        length = np.linalg.norm(np.diff(xy, axis=0), axis=1).sum() if len(xy) > 1 else 0.
        label = f'{"垄" if kind == "row" else "行道"} {index+1:02d} · {item.auto_id}'
        self.structure_panel.selection_info.setText(f'已选：{label}\n长度 {length:.2f} 米；拖动顶点或线段编辑')

    def focus_reference(self):
        selected = self.active_reference()
        if not selected:
            return
        xy = np.asarray(selected[2].centerline_xy)
        if not len(xy):
            return
        low, high = xy.min(axis=0)-1.0, xy.max(axis=0)+1.0
        self.review_region = [low.tolist(), [high[0], low[1]], high.tolist(), [low[0], high[1]]]
        self.review3d.canvas.set_review_region(self.review_region)
        self.draw(reset=True)
        self.update_height_filter()

    def mark_reference_reviewed(self, kind, index, checked):
        items = self.row_proposals if kind == 'row' else self.aisle_proposals
        if not 0 <= index < len(items):
            return
        if bool(items[index].diagnostics.get('reference_reviewed', False)) == checked:
            return
        self._record_workflow_state()
        items[index] = replace(items[index], diagnostics={**dict(items[index].diagnostics), 'reference_reviewed': checked})
        self.rows_by_source[self.active_proposal_source] = list(self.row_proposals)
        self._save_reference_draft()
        self.draw()
        total = sum(item.decision != 'rejected' for item in self.row_proposals+self.aisle_proposals)
        reviewed = sum(item.decision != 'rejected' and item.diagnostics.get('reference_reviewed', False)
            for item in self.row_proposals+self.aisle_proposals)
        self.structure_panel.status.setText(f'审查完成 {reviewed} / {total}；草稿已保存。')

    def delete_reference(self):
        selected = self.active_reference()
        if not selected:
            return
        kind, index, _ = selected
        if kind == 'row':
            self._replace_rows([item for i, item in enumerate(self.row_proposals) if i not in self.selected_proposal_indices])
        else:
            self._record_workflow_state()
            self.aisle_proposals.pop(index)
            self._refresh_proposal_panel()
            self._save_reference_draft()

    def clear_wall_exclusions(self):
        if not self.row_exclusion_polygons:
            return
        self._record_workflow_state()
        self.row_exclusion_polygons = []
        self.wall_exclusions_enabled = False
        self.save_row_exclusions()
        self.draw()
        self.region_status.setText('已清除误圈区域；Ctrl+Z 可以恢复。')

    def export_reference_dialog(self):
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, '导出已审查参考线',
            str(self._site_workspace()/'exports'/'greenhouse_reference.yaml'), '参考 YAML (*.yaml)')
        if not filename:
            return
        try:
            paths = export_reviewed_reference(Path(filename), self.package, self.analysis,
                self.row_proposals, self.aisle_proposals)
            self.structure_panel.status.setText('已导出 YAML 和 GeoJSON：'+str(paths[0]))
        except (ValueError, OSError) as exc:
            self.structure_panel.status.setText(str(exc))

    def _reference_hit(self, event):
        point = np.asarray(self.axes.transData.transform([event.xdata, event.ydata]))
        candidates = []
        for kind, items in (('row', self.row_proposals), ('aisle', self.aisle_proposals)):
            for index, item in enumerate(items):
                if item.decision == 'rejected' or len(item.centerline_xy) < 2:
                    continue
                if self.isolate_selection.isChecked():
                    if kind == 'row' and index not in self.selected_proposal_indices:
                        continue
                    if kind == 'aisle' and index != self.selected_aisle_index:
                        continue
                pixels = self.axes.transData.transform(item.centerline_xy)
                vertex = int(np.argmin(np.linalg.norm(pixels-point, axis=1)))
                vertex_distance = np.linalg.norm(pixels[vertex]-point)
                delta = pixels[1:]-pixels[:-1]
                t = np.clip(np.sum((point-pixels[:-1])*delta, axis=1)/np.maximum(np.sum(delta*delta, axis=1), 1e-12), 0., 1.)
                distances = np.linalg.norm(pixels[:-1]+t[:, None]*delta-point, axis=1)
                segment = int(np.argmin(distances))
                distance = min(vertex_distance, distances[segment])
                candidates.append((distance, kind, index, vertex if vertex_distance <= 10 else None, segment))
        if not candidates:
            return None
        selected = self.active_reference()
        # Prefer handles of an already selected line, avoiding accidental switches at intersections.
        if selected:
            handles = [hit for hit in candidates if hit[1:3] == selected[:2] and hit[3] is not None and hit[0] <= 10]
            if handles:
                return min(handles)
        hit = min(candidates)
        return hit if hit[0] <= 10 else None

    def _select_reference_hit(self, kind, index):
        panel = self.structure_panel
        panel.object_tabs.setCurrentIndex(0 if kind == 'row' else 1)
        if kind == 'row':
            panel.row_list.clearSelection()
            panel.row_list.setCurrentRow(index)
            panel.row_list.item(index).setSelected(True)
        else:
            panel.aisle_list.setCurrentRow(index)
        self.update_selection_description()

    def reference_click(self, event):
        hit = self._reference_hit(event)
        if hit is None:
            return False
        _, kind, index, vertex, segment = hit
        self._select_reference_hit(kind, index)
        items = self.row_proposals if kind == 'row' else self.aisle_proposals
        original = np.asarray(items[index].centerline_xy, dtype=float)
        if event.button == 3:
            if vertex is not None and len(original) > 2:
                self._record_workflow_state()
                self._commit_reference_geometry(kind, index, np.delete(original, vertex, axis=0))
            return True
        if event.button != 1:
            return True
        if getattr(event, 'dblclick', False):
            self.reference_drag = None
            if vertex is not None:
                return True
            self._record_workflow_state()
            self._commit_reference_geometry(kind, index, np.insert(original, segment+1, [event.xdata, event.ydata], axis=0))
            return True
        self.reference_drag = {'kind': kind, 'index': index, 'vertex': vertex,
            'original': original, 'anchor': np.array([event.xdata, event.ydata]),
            'before': self._workflow_snapshot(), 'moved': False}
        return True

    def _set_reference_geometry(self, kind, index, xy):
        line = tuple(tuple(float(v) for v in point) for point in xy)
        items = self.row_proposals if kind == 'row' else self.aisle_proposals
        item = items[index]
        diagnostics = {**dict(item.diagnostics), 'manually_edited': True, 'reference_reviewed': False}
        if kind == 'row':
            perpendicular = np.array([-item.direction_xy[1], item.direction_xy[0]])
            items[index] = replace(item, centerline_xy=line, lateral_v_m=float(np.median(xy@perpendicular)),
                diagnostics=diagnostics, decision='pending', physical_row_id=None, confirmed_direction=None)
        else:
            # Edited reference geometry carries no detector-certified safe centerline.
            items[index] = replace(item, centerline_xy=line, safe_centerline_xy=(),
                diagnostics={**diagnostics, 'safety_status': 'NOT_RECOMPUTED_AFTER_MANUAL_EDIT'},
                decision='pending', physical_aisle_id=None)

    def _commit_reference_geometry(self, kind, index, xy):
        if np.linalg.norm(np.diff(xy, axis=0), axis=1).sum() < .01:
            self.structure_panel.status.setText('参考线长度不能小于 1 厘米。')
            return
        self._set_reference_geometry(kind, index, xy)
        if kind == 'row':
            self._replace_rows(list(self.row_proposals), record=False)
        else:
            self._refresh_proposal_panel()
            self._save_reference_draft()

    def reference_motion(self, event):
        drag = self.reference_drag
        if drag is None or event.inaxes != self.axes or event.xdata is None or event.ydata is None:
            return
        xy = drag['original'].copy()
        delta = np.array([event.xdata, event.ydata])-drag['anchor']
        if not np.isfinite(delta).all():
            return
        if drag['vertex'] is None:
            xy += delta
        else:
            xy[drag['vertex']] += delta
        drag['moved'] = np.linalg.norm(delta) > 1e-5
        self._set_reference_geometry(drag['kind'], drag['index'], xy)
        self.draw()

    def reference_release(self):
        drag = self.reference_drag
        if drag is None:
            return
        self.reference_drag = None
        if drag['moved']:
            self.workflow_history.append(drag['before'])
            self.workflow_future.clear()
            items = self.row_proposals if drag['kind'] == 'row' else self.aisle_proposals
            xy = np.asarray(items[drag['index']].centerline_xy)
            if np.linalg.norm(np.diff(xy, axis=0), axis=1).sum() < .01:
                self._restore_workflow(self.workflow_history.pop())
                return
            self._commit_reference_geometry(drag['kind'], drag['index'], xy)

    def finish_reference_line(self):
        if len(self.pending) < 2:
            self.region_status.setText('至少点击两个不同的点，右键或回车完成。')
            return
        xy = np.asarray(self.pending)
        delta = xy[-1]-xy[0]
        length = np.linalg.norm(delta)
        if length < .01:
            return
        direction = delta/length
        if direction[0] < 0:
            direction *= -1
        self._record_workflow_state()
        line = tuple(tuple(float(v) for v in p) for p in xy)
        if self.mode == 'reference_row':
            ids = {row.auto_id for row in self.row_proposals}
            n = 1
            while f'MANUAL-R{n:03d}' in ids:
                n += 1
            row = GlobalRowProposal(f'MANUAL-R{n:03d}', tuple(direction),
                float(np.median(xy@np.array([-direction[1], direction[0]]))), line,
                .22, 0., 0., {'manually_edited': True, 'reference_reviewed': False})
            self._replace_rows(self.row_proposals+[row], record=False)
        else:
            ids = {aisle.auto_id for aisle in self.aisle_proposals}
            n = 1
            while f'MANUAL-A{n:03d}' in ids:
                n += 1
            self.aisle_proposals.append(AisleProposal(f'MANUAL-A{n:03d}', '', '', 'REFERENCE',
                line, (), 0., 0., {'manually_edited': True, 'reference_reviewed': False}))
            self._refresh_proposal_panel()
        self.mode_combo.setCurrentIndex(0)
        self.pending = []
        self._save_reference_draft()
        self.draw()
