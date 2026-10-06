from types import SimpleNamespace

import numpy as np
from PyQt5 import QtWidgets


def test_workbench_and_3d_review_import_with_separate_centerline_layers():
    from agt_greenhouse_annotation import workbench
    from agt_greenhouse_annotation.review_3d import ThreeDReviewWidget

    assert callable(workbench.run_real_smoke)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = ThreeDReviewWidget()
    shape = (5, 6)
    valid = np.ones(shape, dtype=bool)
    navigation = SimpleNamespace(
        ground_valid=valid,
        origin_x_m=0.0,
        origin_y_m=0.0,
        resolution_m=0.1,
        ground_height_m=np.zeros(shape),
    )
    corridor = SimpleNamespace(
        row_centerline=valid,
        row_structural_band=valid,
        aisle_candidate=valid,
        aisle_centerline=valid,
    )
    geometric = SimpleNamespace(mask=np.eye(5, 6, dtype=bool))
    widget.set_analysis(navigation, corridor, geometric)

    assert widget.canvas._layers['geometric_centerline'].shape[0] == 5
    assert widget.canvas._layers['safe_centerline'].shape[0] == 30
    assert widget.layer_checks['geometric_centerline'].isChecked()
    assert not widget.layer_checks['safe_centerline'].isChecked()
    widget.layer_checks['safe_centerline'].setChecked(True)
    assert widget.canvas._layer_visibility['safe_centerline']
    widget.close()
    app.processEvents()
