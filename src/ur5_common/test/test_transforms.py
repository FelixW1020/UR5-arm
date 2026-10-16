import numpy as np

from ur5_common.transforms import (invert, make_transform, matrix_to_quat,
                                   quat_to_matrix, rotation_angle)


def _random_rotation(rng):
    q = rng.normal(size=4)
    return quat_to_matrix(*(q / np.linalg.norm(q)))


def test_quat_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(200):
        R = _random_rotation(rng)
        assert np.allclose(quat_to_matrix(*matrix_to_quat(R)), R, atol=1e-9)


def test_invert():
    rng = np.random.default_rng(1)
    T = make_transform(_random_rotation(rng), rng.normal(size=3))
    assert np.allclose(T @ invert(T), np.eye(4), atol=1e-12)


def test_rotation_angle():
    c, s = np.cos(0.7), np.sin(0.7)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    assert abs(rotation_angle(Rz) - 0.7) < 1e-12
