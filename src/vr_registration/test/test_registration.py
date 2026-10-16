import numpy as np
import pytest

from ur5_common.transforms import invert, make_transform, quat_to_matrix
from vr_registration.registration import fit_controller_on_flange, fit_points


def _rand_T(rng, t_scale=0.5, max_angle=np.pi):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    a = rng.uniform(-max_angle, max_angle)
    return make_transform(quat_to_matrix(*axis * np.sin(a / 2), np.cos(a / 2)),
                          rng.normal(scale=t_scale, size=3))


def _yaw_T(theta, t):
    c, s = np.cos(theta), np.sin(theta)
    return make_transform([[c, -s, 0], [s, c, 0], [0, 0, 1]], t)


def _tags(rng, n=5):
    # Tags spread over a table in front of the robot.
    return np.column_stack([rng.uniform(0.2, 0.8, n), rng.uniform(-0.5, 0.5, n),
                            rng.uniform(-0.02, 0.02, n)])


def _to_vr(T_base_vr, p_base):
    T = invert(T_base_vr)
    return p_base @ T[:3, :3].T + T[:3, 3]


def test_yaw_only_exact():
    rng = np.random.default_rng(0)
    T_true = _yaw_T(2.1, [1.5, -0.7, -0.8])   # e.g. Quest floor origin 0.8 m below the base
    p_base = _tags(rng)
    r = fit_points(_to_vr(T_true, p_base), p_base, 'yaw_only')
    assert np.allclose(r.T_base_vr, T_true, atol=1e-9)
    assert r.max_error_m < 1e-9


def test_full_exact():
    rng = np.random.default_rng(1)
    T_true = _rand_T(rng)
    p_base = _tags(rng)
    r = fit_points(_to_vr(T_true, p_base), p_base, 'full')
    assert np.allclose(r.T_base_vr, T_true, atol=1e-9)


def test_yaw_only_beats_full_with_noise():
    """With 1.5 cm VR noise and few tags, the gravity constraint gives a better transform."""
    rng = np.random.default_rng(2)
    T_true = _yaw_T(-0.6, [0.9, 0.3, -0.75])
    err = {'yaw_only': [], 'full': []}
    probe = np.array([[0.5, 0.0, 0.3], [0.3, 0.4, 0.5]])    # points where the game cares
    for _ in range(200):
        p_base = _tags(rng, 4)
        p_vr = _to_vr(T_true, p_base) + rng.normal(scale=0.015, size=(4, 3))
        for mode in err:
            T = fit_points(p_vr, p_base, mode).T_base_vr
            pv = _to_vr(T_true, probe)
            err[mode].append(np.mean(np.linalg.norm(pv @ T[:3, :3].T + T[:3, 3] - probe, axis=1)))
    assert np.mean(err['yaw_only']) < np.mean(err['full'])


def test_rejects_degenerate():
    with pytest.raises(ValueError):
        fit_points([[0, 0, 0], [1, 0, 0]], [[0, 0, 0], [1, 0, 0]], 'full')
    with pytest.raises(ValueError):
        fit_points([[0, 0, 0], [1, 0, 0], [2, 0, 0]], [[0, 0, 0], [1, 0, 0], [2, 0, 0]], 'full')
    with pytest.raises(ValueError):
        fit_points([[0, 0, 0], [0, 0, 1]], [[0, 0, 0], [0, 0, 1]], 'yaw_only')


def test_controller_on_flange():
    rng = np.random.default_rng(3)
    T_base_vr = _rand_T(rng, t_scale=1.0)
    T_tool0_ctrl = _rand_T(rng, t_scale=0.08)
    T_bt = []
    for _ in range(15):
        T = _rand_T(rng, max_angle=0.9)
        T[:3, 3] = [0.4, 0.0, 0.4] + rng.uniform(-0.2, 0.2, 3)
        T_bt.append(T)
    T_vc = [invert(T_base_vr) @ T @ T_tool0_ctrl for T in T_bt]
    r, X = fit_controller_on_flange(T_bt, T_vc)
    assert np.allclose(r.T_base_vr, T_base_vr, atol=1e-6)
    assert np.allclose(X, T_tool0_ctrl, atol=1e-6)
    assert r.max_error_m < 1e-6
