import numpy as np

from agt_greenhouse_annotation.navigation_corridor import _boundary_candidate_centers


def test_boundary_candidates_exclude_members_of_interior_merged_rows():
    centers = np.asarray([0.20, 0.60, 1.00, 2.00, 3.00, 4.80], dtype=np.float64)
    candidates = _boundary_candidate_centers(
        centers,
        v_min=0.0,
        v_max=5.0,
        boundary_exclusion_m=0.45,
    )
    np.testing.assert_allclose(candidates, np.asarray([0.20, 4.80]))
