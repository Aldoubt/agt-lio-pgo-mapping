"""Qt offscreen interaction checks on a small synthetic package."""
import argparse
import copy
import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
import numpy as np
from PyQt5 import QtWidgets

from agt_greenhouse_annotation import gui
from agt_greenhouse_annotation.structure_artifacts import GlobalRowProposal, GreenhouseStructureConfig
from test_topology import make_package


@pytest.fixture
def editor(tmp_path, monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    package = make_package(tmp_path/'synthetic_gui_map')
    monkeypatch.setattr(gui.CloudLoader, 'start', lambda self: None)
    args = argparse.Namespace(map_package=str(package.path), output=str(tmp_path/'draft'/'topology.yaml'),
        other_map_package=None, max_row_assignment_distance=1., correspondence_max_dt=.5,
        voxel=.2, max_display_points=100, smoke_output=None, smoke_save_empty=False)
    window = gui.Annotator(args)
    yield window
    window.dirty = False
    window.close()
    app.processEvents()


def synthetic_row():
    return dict(id='SYNTHETIC_R1', centerline=[[0., 0.], [10., 0.]],
                nominal_width_m=1., direction='bidirectional', confidence='confirmed', notes='test only')


def test_gui_undo_redo_and_delete(editor):
    editor.mutate(lambda: editor.topology['rows'].append(synthetic_row()))
    assert len(editor.topology['rows']) == 1
    editor.undo()
    assert editor.topology['rows'] == []
    editor.redo()
    assert len(editor.topology['rows']) == 1
    editor.list.setCurrentRow(0)
    editor.delete_selected()
    assert editor.topology['rows'] == []
    editor.undo()
    assert editor.topology['rows'][0]['id'] == 'SYNTHETIC_R1'


def test_gui_edit_id_width_vertices_and_scene_reference(editor, monkeypatch):
    editor.mutate(lambda: editor.topology['rows'].append(synthetic_row()))
    editor.topology['scenes'].append(dict(scene_id='SYNTHETIC_SCENE', scene_type='ROW_MIDDLE',
        keyframe=1, bag_timestamp=101., x=2., y=0., yaw_rad=0.,
        manual_physical_row_id='SYNTHETIC_R1', manual_headland_id='UNKNOWN'))
    editor.refresh_list()
    editor.list.setCurrentRow(0)
    replacement = synthetic_row()
    replacement.update(id='SYNTHETIC_R2', nominal_width_m=2., centerline=[[0., 0.], [5., 0.], [10., 1.]])
    class Dialog:
        def __init__(self, *args):
            pass
        def exec_(self):
            return QtWidgets.QDialog.Accepted
        def value(self):
            return copy.deepcopy(replacement)
    monkeypatch.setattr(gui, 'GeometryDialog', Dialog)
    editor.edit_selected()
    assert editor.topology['rows'][0] == replacement
    assert editor.topology['scenes'][0]['manual_physical_row_id'] == 'SYNTHETIC_R2'
    editor.undo()
    assert editor.topology['rows'][0]['id'] == 'SYNTHETIC_R1'
    assert editor.topology['rows'][0]['nominal_width_m'] == 1.


def test_gui_frozen_is_read_only(editor):
    editor.topology['annotation'].update(status='frozen', manual_review_confirmed=True)
    editor.mutate(lambda: editor.topology['rows'].append(synthetic_row()))
    assert editor.topology['rows'] == []
    assert editor.history == []


def test_auto_proposal_polyline_edit_keeps_auto_id_and_supports_undo(editor, monkeypatch):
    row = GlobalRowProposal('AUTO-G007', (1., 0.), 1., ((0., 1.), (5., 1.)), .2,
                            .7, .8, {'fixture': True})
    editor.analysis = SimpleNamespace(config=GreenhouseStructureConfig(), analysis_hash='fixture-analysis')
    editor.row_proposals = [row]
    editor.rows_by_source['GLOBAL_PROFILE'] = [row]

    class Dialog:
        def __init__(self, *_args):
            pass
        def exec_(self):
            return QtWidgets.QDialog.Accepted
        def value(self):
            return {'centerline_xy': [[0., 1.], [2.5, 1.4], [5., 1.]], 'width_m': .6}

    monkeypatch.setattr(gui, 'ProposalGeometryDialog', Dialog)
    editor.edit_row_proposal(0)
    edited = editor.row_proposals[0]
    assert edited.auto_id == 'AUTO-G007'
    assert len(edited.centerline_xy) == 3
    assert edited.half_width_m == pytest.approx(.3)
    assert edited.decision == 'pending'
    editor.undo_proposals()
    assert editor.row_proposals[0].centerline_xy == row.centerline_xy


def test_evidence_layer_panel_exposes_and_controls_independent_layers(editor):
    required = {
        'point_cloud', 'ground', 'ground_confidence', 'slope', 'plane_residual',
        'terrain_ridge', 'terrain_depression', 'terrain_step', 'global_row_support',
        'global_row_proposal', 'local_row_observations', 'local_row_tracks',
        'row_structural_band', 'aisle_geometry', 'aisle_proposals', 'geometric_centerline',
        'safe_aisle', 'safe_centerline', 'physical_row_labels', 'headland',
        'trajectory', 'keyframes',
    }
    assert required <= set(editor.evidence_panel.layer_checks)
    editor.evidence_panel.layer_checks['ground_confidence'].setChecked(True)
    editor.evidence_panel.layer_checks['plane_residual'].setChecked(True)
    editor.evidence_panel.layer_checks['keyframes'].setChecked(False)
    editor.evidence_panel.layer_checks['headland'].setChecked(False)
    assert editor.evidence_layer_visible['ground_confidence'] is True
    assert editor.evidence_layer_visible['plane_residual'] is True
    assert editor.keyframes_visible is False
    assert editor.headlands_visible is False


def test_2d_evidence_layer_renderer_draws_independent_rasters(editor):
    from agt_greenhouse_annotation.structure_artifacts import GreenhouseStructureConfig

    shape = (3, 3)
    valid = np.ones(shape, dtype=bool)
    line = np.eye(3, dtype=bool)
    navigation = SimpleNamespace(
        occupancy=np.zeros(shape, dtype=np.uint8), ground_valid=valid,
        ground_height_m=np.arange(9, dtype=float).reshape(shape),
        resolution_m=.1, bounds_m=lambda: (0., 0., .3, .3),
    )
    terrain = SimpleNamespace(
        ground_height_m=navigation.ground_height_m, ground_valid=valid,
        ground_confidence=np.full(shape, .8), robust_slope_deg=np.full(shape, 2.),
        robust_plane_residual_m=np.linspace(-.02, .02, 9).reshape(shape),
        ridge_evidence=np.full(shape, .2), depression_evidence=np.full(shape, .1),
        step_evidence=np.full(shape, .3),
    )
    observation = SimpleNamespace(u_center_m=.1, v_center_m=.2, support=.8)
    local = SimpleNamespace(
        row_structural_band=valid, aisle_candidate=line, aisle_centerline=line,
        observations=(observation,),
    )
    corridor = SimpleNamespace(
        row_structural_band=valid, aisle_geometric_envelope=line,
        aisle_candidate=valid, aisle_centerline=line,
    )
    editor.analysis = SimpleNamespace(
        navigation=navigation, terrain=terrain,
        global_result=SimpleNamespace(row_support=np.full(shape, .5)),
        local_result=local, corridor_result=corridor,
        geometric_aisle_result=SimpleNamespace(mask=line),
        config=GreenhouseStructureConfig(),
        diagnostics={'local_direction_xy': [1., 0.]},
    )
    editor.evidence_layer_visible.update({
        name: True for name in (
            'ground', 'ground_confidence', 'slope', 'plane_residual', 'terrain_ridge',
            'terrain_depression', 'terrain_step', 'global_row_support',
            'local_row_observations', 'row_structural_band', 'aisle_geometry',
            'geometric_centerline', 'safe_aisle', 'safe_centerline',
        )
    })
    editor.axes.clear()
    editor.draw_evidence_layers()
    assert len(editor.axes.images) == 13
    assert len(editor.axes.collections) == 1


def test_height_filter_shared_with_3d_and_preserves_cloud(editor):
    cloud = np.array([[0., 0., -1.], [1., 0., .5], [2., 0., 3.]])
    editor.cloud_loaded(cloud, 3)
    editor.height_min.setValue(0.)
    editor.height_max.setValue(1.)
    editor.height_filter.setChecked(True)
    np.testing.assert_array_equal(editor.display_cloud(), cloud[1:2])
    np.testing.assert_array_equal(editor.review3d.canvas.visible_cloud()[0], cloud[1:2])
    np.testing.assert_array_equal(editor.cloud, cloud)
    editor.height_filter.setChecked(False)
    assert editor.review3d.canvas.sample_count() == 3


def test_3d_proposals_are_dense_and_follow_ground_height(editor):
    from agt_greenhouse_annotation.structure_artifacts import AisleProposal
    navigation = SimpleNamespace(origin_x_m=0., origin_y_m=0., resolution_m=1.,
        ground_valid=np.ones((3, 6), dtype=bool), ground_height_m=np.full((3, 6), -1.2))
    editor.analysis = SimpleNamespace(navigation=navigation)
    editor.row_proposals = [GlobalRowProposal('AUTO-G1', (1., 0.), 1.,
        ((0., 1.), (5., 1.)), .2, .7, .8, {})]
    editor.aisle_proposals = [AisleProposal('AUTO-A1', 'AUTO-G1', 'AUTO-G2', 'INTERIOR',
        ((0., 2.), (5., 2.)), (), 1., .8, {})]
    editor.sync_review_proposals()
    layers = editor.review3d.canvas._layers
    assert len(layers['row_centerline']) > 50
    assert len(layers['geometric_centerline']) > 50
    np.testing.assert_allclose(layers['row_centerline'][:, 2], -1.12)


def test_3d_numpy_sample_limit_and_intensity_alignment(editor):
    cloud = np.column_stack((np.arange(100), np.zeros(100), np.arange(100), np.arange(100)*2))
    canvas = editor.review3d.canvas
    canvas.set_cloud(cloud, sample_limit=10)
    assert canvas.sample_count() == 10
    canvas.set_height_filter((20., 80.))
    points, intensity = canvas.visible_cloud()
    np.testing.assert_array_equal(intensity, points[:, 2]*2)


def test_height_colors_legend_and_filter(editor):
    cloud = np.array([[0., 0., -1.], [1., 0., .5], [2., 0., 3.]])
    editor.cloud_loaded(cloud, 3)
    scatter = editor.axes.collections[0]
    np.testing.assert_array_equal(scatter.get_array(), cloud[:, 2])
    np.testing.assert_allclose(scatter.get_clim(), np.quantile(cloud[:, 2], [.02, .98]))
    assert len(editor.axes.child_axes) == 1
    editor.height_min.setValue(0.)
    editor.height_max.setValue(1.)
    editor.height_filter.setChecked(True)
    np.testing.assert_array_equal(editor.axes.collections[0].get_array(), [.5])
    assert editor.axes.collections[0].get_clim() == (0., 1.)
    editor.draw()
    assert len(editor.axes.child_axes) == 1
    editor.cloud_color_mode.setCurrentIndex(1)
    assert editor.axes.collections[0].get_array() is None
    assert not editor.axes.child_axes
    np.testing.assert_array_equal(editor.cloud, cloud)


def test_scroll_zoom_anchor_direction_and_toolbar_back(editor):
    editor.axes.set_xlim(0., 10.)
    editor.axes.set_ylim(-5., 5.)
    event = SimpleNamespace(inaxes=editor.axes, xdata=2., ydata=1., step=1.)
    editor.scroll_zoom(event)
    xlim, ylim = editor.axes.get_xlim(), editor.axes.get_ylim()
    assert xlim[1]-xlim[0] == pytest.approx(10./1.2)
    assert (2.-xlim[0])/(xlim[1]-xlim[0]) == pytest.approx(.2)
    assert (1.-ylim[0])/(ylim[1]-ylim[0]) == pytest.approx(.6)
    editor.canvas.toolbar.back()
    np.testing.assert_allclose(editor.axes.get_xlim(), [0., 10.])
    event.step = -1.
    editor.scroll_zoom(event)
    assert np.ptp(editor.axes.get_xlim()) == pytest.approx(12.)


def test_scroll_ignores_outside_axes_and_vertex_drag(editor):
    before = editor.axes.get_xlim()
    event = SimpleNamespace(inaxes=None, xdata=None, ydata=None, step=1.)
    editor.scroll_zoom(event)
    assert editor.axes.get_xlim() == before
    event.inaxes, event.xdata, event.ydata = editor.axes, .5, .5
    editor.drag = ('rows', 0, 0)
    editor.scroll_zoom(event)
    assert editor.axes.get_xlim() == before
    editor.drag = None


def test_local_review_clips_2d_and_3d_and_restores(editor):
    cloud = np.array([[0., 0., 0.], [2., 2., 1.], [4., 4., 2.]])
    editor.cloud_loaded(cloud, 3)
    editor.start_region_mode(6)
    for x, y in ((1., 1.), (3., 3.)):
        editor.click(SimpleNamespace(inaxes=editor.axes, xdata=x, ydata=y, button=1))
    np.testing.assert_array_equal(editor.display_cloud(), cloud[1:2])
    np.testing.assert_array_equal(editor.review3d.canvas.visible_cloud()[0], cloud[1:2])
    assert editor.axes.get_xlim() == (1., 3.)
    editor.clear_review_region()
    np.testing.assert_array_equal(editor.display_cloud(), cloud)
    assert editor.review3d.canvas.sample_count() == 3


def test_wall_regions_saved_with_map_hash_and_undo(editor):
    editor.start_region_mode(7)
    for x, y in ((0., 0.), (5., 0.), (5., .5), (0., .5)):
        editor.click(SimpleNamespace(inaxes=editor.axes, xdata=x, ydata=y, button=1, dblclick=False))
    editor.key(SimpleNamespace(key='enter'))
    assert len(editor.row_exclusion_polygons) == 1
    import yaml
    saved = yaml.safe_load(editor.row_exclusions_path.read_text())
    assert saved['map_package_hash'] == editor.package.map_package_hash
    assert saved['row_exclusion_polygons_xy'] == editor.row_exclusion_polygons
    editor.undo_row_exclusion()
    assert yaml.safe_load(editor.row_exclusions_path.read_text())['row_exclusion_polygons_xy'] == []


def _reference_fixture(editor):
    from agt_greenhouse_annotation.structure_artifacts import AisleProposal
    editor.row_proposals = [GlobalRowProposal('AUTO-G001', (1., 0.), 1.,
        ((0., 1.), (5., 1.)), .2, .7, .8, {})]
    editor.rows_by_source['GLOBAL_PROFILE'] = list(editor.row_proposals)
    editor.aisle_proposals = [AisleProposal('AUTO-A001', 'AUTO-G001', 'AUTO-G002', 'INTERIOR',
        ((0., 2.), (5., 2.)), ((0., 2.1), (5., 2.1)), 1., .8, {})]
    editor._refresh_proposal_panel()
    editor.axes.set_xlim(-1., 6.)
    editor.axes.set_ylim(0., 3.)
    editor.canvas.draw()


def test_reference_selection_and_3d_highlight_are_linked(editor):
    _reference_fixture(editor)
    editor.structure_panel.row_list.setCurrentRow(0)
    assert editor.active_reference()[2].auto_id == 'AUTO-G001'
    assert 'AUTO-G001' in editor.structure_panel.selection_info.text()
    assert len(editor.review3d.canvas._layers['selected_reference']) > 20
    assert 'AUTO-G001' in editor.review3d.canvas.selection_label
    editor.structure_panel.aisle_list.setCurrentRow(0)
    assert editor.active_reference()[0] == 'aisle'
    assert not editor.selected_proposal_indices
    assert 'AUTO-A001' in editor.review3d.canvas.selection_label


def test_reference_row_vertex_drag_and_unified_undo_redo(editor):
    _reference_fixture(editor)
    original = editor.row_proposals[0].centerline_xy
    editor.click(SimpleNamespace(inaxes=editor.axes, xdata=0., ydata=1., button=1, dblclick=False))
    editor.motion(SimpleNamespace(inaxes=editor.axes, xdata=.2, ydata=1.2))
    editor.release(None)
    assert editor.row_proposals[0].centerline_xy[0] == pytest.approx((.2, 1.2))
    editor.undo()
    assert editor.row_proposals[0].centerline_xy == original
    editor.redo()
    assert editor.row_proposals[0].centerline_xy[0] == pytest.approx((.2, 1.2))
    assert editor.reference_draft_path.exists()


def test_aisle_edit_resets_review_and_safe_evidence(editor):
    _reference_fixture(editor)
    editor.mark_reference_reviewed('aisle', 0, True)
    editor.click(SimpleNamespace(inaxes=editor.axes, xdata=2.5, ydata=2., button=1, dblclick=False))
    editor.motion(SimpleNamespace(inaxes=editor.axes, xdata=2.5, ydata=2.3))
    editor.release(None)
    aisle = editor.aisle_proposals[0]
    assert aisle.centerline_xy == ((0., 2.3), (5., 2.3))
    assert not aisle.diagnostics['reference_reviewed']
    assert not aisle.safe_centerline_xy
    editor.undo()
    assert editor.aisle_proposals[0].diagnostics['reference_reviewed']
    assert editor.aisle_proposals[0].safe_centerline_xy


def test_wall_undo_works_through_common_shortcut(editor):
    editor.pending = [[0., 0.], [5., 0.], [5., .5], [0., .5]]
    editor.finish_row_exclusion()
    assert editor.wall_exclusions_enabled
    editor.undo()
    assert editor.row_exclusion_polygons == []
    editor.redo()
    assert len(editor.row_exclusion_polygons) == 1
    editor.clear_wall_exclusions()
    assert not editor.row_exclusion_polygons
    editor.undo()
    assert len(editor.row_exclusion_polygons) == 1


def test_whole_map_wall_exclusion_is_rejected(editor):
    editor.cloud_loaded(np.array([[1., 1., 0.], [2., 2., 0.]]), 2)
    editor.pending = [[0., 0.], [3., 0.], [3., 3.], [0., 3.]]
    editor.finish_row_exclusion()
    assert not editor.row_exclusion_polygons
    assert '覆盖过大' in editor.region_status.text()


def test_reference_draft_roundtrip_and_reviewed_export(editor, tmp_path):
    from agt_greenhouse_annotation.reference_review import load_reference_draft, export_reviewed_reference
    _reference_fixture(editor)
    target = tmp_path/'export.yaml'
    with pytest.raises(ValueError, match='尚未勾选'):
        export_reviewed_reference(target, editor.package, None, editor.row_proposals, editor.aisle_proposals)
    editor.mark_reference_reviewed('row', 0, True)
    editor.mark_reference_reviewed('aisle', 0, True)
    paths = export_reviewed_reference(target, editor.package, None, editor.row_proposals, editor.aisle_proposals)
    assert all(path.exists() for path in paths)
    rows, aisles, _, _ = load_reference_draft(editor.reference_draft_path, editor.package)
    assert rows[0].diagnostics['reference_reviewed']
    assert aisles[0].centerline_xy == editor.aisle_proposals[0].centerline_xy
    import yaml
    data = yaml.safe_load(target.read_text())
    assert data['manual_review_confirmed']
    assert not data['absolute_ground_truth']
    with pytest.raises(FileExistsError):
        export_reviewed_reference(target, editor.package, None, rows, aisles)


def test_one_click_result_uses_detector_centerlines_and_saves(editor):
    from test_structure_artifacts import _synthetic_cloud
    from agt_greenhouse_annotation.structure_artifacts import analyze_greenhouse_structure
    analysis = analyze_greenhouse_structure(_synthetic_cloud(), editor.package.map_package_hash,
        row_direction_xy=(1., 0.))
    editor.structure_analysis_completed(analysis)
    assert editor.row_proposals
    assert editor.aisle_proposals
    assert editor.reference_draft_path.exists()
    assert editor.aisle_proposals[0].safe_centerline_xy == analysis.aisle_proposals[0].safe_centerline_xy
    editor.undo()
    assert not editor.row_proposals
    editor.redo()
    assert editor.row_proposals


def test_one_click_ignores_disabled_legacy_wall_regions(editor, monkeypatch):
    editor.row_exclusion_polygons = [[[0., 0.], [10., 0.], [10., 10.], [0., 10.]]]
    editor.wall_exclusions_enabled = False
    monkeypatch.setattr(gui.StructureAnalyzer, 'start', lambda worker: None)
    editor.structure_panel.run_button.click()
    assert editor.analysis_worker.mode == 'COMPARE'
    assert editor.analysis_worker.row_exclusions == []


def test_first_visible_map_load_schedules_compare_proposals_without_promoting_topology(editor, monkeypatch):
    scheduled = []
    started = []
    monkeypatch.setattr(gui.QtCore.QTimer, 'singleShot',
                        lambda delay, callback: scheduled.append((delay, callback)))
    monkeypatch.setattr(gui.StructureAnalyzer, 'start', lambda worker: started.append(worker.mode))
    cloud = np.array([[0., 0., 0.], [1., 0., .2], [2., 0., .4]])

    editor.cloud_loaded(cloud, len(cloud))
    assert scheduled == []  # hidden unit-test windows do not start background work
    editor.show()
    editor.cloud_loaded(cloud, len(cloud))
    assert scheduled and scheduled[-1][0] == 50
    scheduled[-1][1]()
    assert started == ['COMPARE']
    assert editor.topology['rows'] == []
    assert editor.topology['annotation']['manual_review_confirmed'] is False


def test_keyboard_undo_and_redo_with_canvas_focus_execute_once(editor):
    from PyQt5 import QtCore, QtTest
    _reference_fixture(editor)
    editor.mark_reference_reviewed('row', 0, True)
    editor.mark_reference_reviewed('aisle', 0, True)
    history_length = len(editor.workflow_history)
    editor.show()
    editor.activateWindow()
    editor.canvas.setFocus()
    QtWidgets.QApplication.processEvents()
    QtTest.QTest.keyClick(editor.canvas, QtCore.Qt.Key_Z, QtCore.Qt.ControlModifier)
    QtWidgets.QApplication.processEvents()
    assert len(editor.workflow_history) == history_length-1
    assert editor.row_proposals[0].diagnostics['reference_reviewed']
    assert not editor.aisle_proposals[0].diagnostics.get('reference_reviewed', False)
    QtTest.QTest.keyClick(editor.canvas, QtCore.Qt.Key_Z, QtCore.Qt.ControlModifier | QtCore.Qt.ShiftModifier)
    QtWidgets.QApplication.processEvents()
    assert len(editor.workflow_history) == history_length
    assert editor.aisle_proposals[0].diagnostics['reference_reviewed']
