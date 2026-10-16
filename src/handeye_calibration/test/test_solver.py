"""Synthetic ground-truth tests for the hand-eye solver, both mounts."""
import numpy as np
import pytest

from handeye_calibration import solver
from ur5_common.transforms import invert, make_transform, quat_to_matrix


def _rand_T(rng, t_scale=0.3, max_angle=np.pi):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = rng.uniform(-max_angle, max_angle)
    q = np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]
    return make_transform(quat_to_matrix(*q), rng.normal(scale=t_scale, size=3))


def _perturb(T, rng, t_noise, r_noise_rad):
    return T @ _rand_T(rng, t_scale=t_noise, max_angle=r_noise_rad) if t_noise else T


def _gripper_poses(rng, n):
    # Flange poses in a plausible UR5 workspace with large wrist rotations.
    poses = []
    for _ in range(n):
        T = _rand_T(rng, max_angle=0.8)
        T[:3, 3] = [0.4, 0.0, 0.4] + rng.uniform(-0.15, 0.15, size=3)
        poses.append(T)
    return poses


def _make_data(mount, rng, n=15, t_noise=0.0, r_noise_deg=0.0):
    X_true = _rand_T(rng, t_scale=0.1)
    T_fixed = _rand_T(rng, t_scale=0.5)    # board pose in base (eye-in-hand) / in gripper (eye-to-hand)
    robot = _gripper_poses(rng, n)
    target = []
    for T_bg in robot:
        if mount == 'eye_in_hand':        # X = T_gripper_cam, T_fixed = T_base_target
            T_ct = invert(T_bg @ X_true) @ T_fixed
        else:                             # X = T_base_cam, T_fixed = T_gripper_target
            T_ct = invert(X_true) @ T_bg @ T_fixed
        target.append(_perturb(T_ct, rng, t_noise, np.radians(r_noise_deg)))
    return robot, target, X_true


@pytest.mark.parametrize('mount', solver.MOUNTS)
@pytest.mark.parametrize('method', sorted(solver.METHODS))
def test_exact_recovery(mount, method):
    robot, target, X_true = _make_data(mount, np.random.default_rng(42))
    r = solver.solve(robot, target, mount, method)
    assert np.allclose(r.X, X_true, atol=1e-5)
    assert r.translation_residual_m < 1e-5
    assert r.rotation_residual_deg < 1e-3


@pytest.mark.parametrize('mount', solver.MOUNTS)
def test_noisy_recovery(mount):
    robot, target, X_true = _make_data(mount, np.random.default_rng(7), n=25,
                                       t_noise=0.001, r_noise_deg=0.2)
    r = solver.solve(robot, target, mount, 'park')
    assert np.linalg.norm(r.X[:3, 3] - X_true[:3, 3]) < 0.005
    assert 0.0 < r.translation_residual_m < 0.01


def test_rejects_too_few_samples():
    robot, target, _ = _make_data('eye_in_hand', np.random.default_rng(0), n=2)
    with pytest.raises(ValueError):
        solver.solve(robot, target)


def test_rotation_spread():
    I = np.eye(4)
    c, s = np.cos(np.radians(40)), np.sin(np.radians(40))
    Rz = make_transform([[c, -s, 0], [s, c, 0], [0, 0, 1]], [0, 0, 0])
    assert abs(solver.rotation_spread_deg([I, Rz]) - 40.0) < 1e-9
