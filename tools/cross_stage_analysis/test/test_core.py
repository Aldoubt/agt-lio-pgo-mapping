import numpy as np
import pytest

from cross_stage_analysis.core import invert_se3, transform_xyz, validate_se3


def test_identity_se3_and_inverse_roundtrip():
    identity = np.eye(4)
    assert validate_se3(identity)["valid"]
    point = np.array([[1.0, 2.0, 3.0]])
    np.testing.assert_allclose(transform_xyz(transform_xyz(point, identity), invert_se3(identity)), point)


@pytest.mark.parametrize("matrix", [
    np.eye(3),
    np.diag([2.0, 1.0, 1.0, 1.0]),
    np.diag([-1.0, 1.0, 1.0, 1.0]),
    np.full((4, 4), np.nan),
])
def test_invalid_se3_is_rejected(matrix):
    with pytest.raises(ValueError):
        validate_se3(matrix)


def test_transform_uses_source_to_target_column_convention():
    transform = np.eye(4)
    transform[:3, 3] = [4.0, -2.0, 1.5]
    point = transform_xyz(np.array([[1.0, 2.0, 3.0]]), transform)
    np.testing.assert_allclose(point, [[5.0, 0.0, 4.5]])
