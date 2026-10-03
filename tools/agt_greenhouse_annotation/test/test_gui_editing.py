"""Qt offscreen interaction checks on a small synthetic package."""
import argparse
import copy
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5 import QtWidgets

from agt_greenhouse_annotation import gui
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
