import numpy as np

from ur5_common.transforms import make_transform, quat_to_matrix
from vr_registration.registration import Registration
from vr_registration.registration_io import load_registration, save_registration


def test_round_trip(tmp_path):
    T = make_transform(quat_to_matrix(0, 0, 0.6, 0.8), [1.0, -0.5, -0.8])
    r = Registration(T, np.array([0.01, 0.02]), 'points_yaw_only')
    path = save_registration(str(tmp_path / 'r.yaml'), r, 'base_link', 'vr_world', {'tags': [1, 2]})
    parent, child, T2, d = load_registration(path)
    assert (parent, child) == ('base_link', 'vr_world')
    assert np.allclose(T, T2)
    assert d['tags'] == [1, 2] and abs(d['max_error_m'] - 0.02) < 1e-12
