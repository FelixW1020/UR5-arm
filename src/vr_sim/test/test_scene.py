import numpy as np
import pytest

from ur5_common.transforms import make_transform, quat_to_matrix
from vr_sim import scene


def test_rpy_matches_ros_convention():
    # ROS rpy (0, 0, 90 deg) turns x onto y; tf2 quaternion for rpy (0.1, 0.2, 0.3)
    assert np.allclose(scene.rpy_to_matrix(0, 0, np.pi / 2) @ [1, 0, 0], [0, 1, 0])
    q = [0.0342708, 0.1060205, 0.1435722, 0.9833474]
    assert np.allclose(scene.rpy_to_matrix(0.1, 0.2, 0.3), quat_to_matrix(*q), atol=1e-6)
    T = scene.pose_from_xyz_rpy_deg([1, 2, 3], [0, 0, 135])
    assert np.allclose(T[:3, 3], [1, 2, 3]) and np.allclose(T[:3, 0], [-0.5 ** 0.5, 0.5 ** 0.5, 0])


def test_look_at():
    eye, target = np.array([0.45, -0.85, 0.65]), np.array([0.47, -0.07, 0.0])
    T = scene.look_at(eye, target)
    R = T[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3)) and np.isclose(np.linalg.det(R), 1.0)
    assert np.allclose(T[:3, 3], eye)
    assert np.allclose(R[:, 0], (target - eye) / np.linalg.norm(target - eye))
    assert abs(R[2, 1]) < 1e-12 and R[2, 2] > 0          # no roll, z up-ish
    down = scene.look_at([0, 0, 1], [0, 0, 0])              # degenerate up vector
    assert np.allclose(down[:3, 0], [0, 0, -1])


def test_tags_from_params():
    tags = scene.tags_from_params([0, 1], [0.4, 0.3, 0.0, 0.6, -0.1, 0.0], [0.0, 90.0])
    assert sorted(tags) == [0, 1]
    assert np.allclose(tags[1][:3, 3], [0.6, -0.1, 0.0])
    assert np.allclose(tags[1][:3, 0], [0, 1, 0]) and np.allclose(tags[1][:3, 2], [0, 0, 1])
    with pytest.raises(ValueError):
        scene.tags_from_params([0, 1], [0.4, 0.3, 0.0], [0.0, 0.0])
    with pytest.raises(ValueError):
        scene.tags_from_params([0, 0], [0.4, 0.3, 0.0, 0.6, -0.1, 0.0], [0.0, 0.0])


def test_square_offset():
    side, period = 0.2, 16.0
    corners = [scene.square_offset(t, side, period) for t in (0.0, 4.0, 8.0, 12.0, 16.0)]
    assert np.allclose(corners, [[0, 0], [side, 0], [side, side], [0, side], [0, 0]])
    ts = np.linspace(0.0, 2 * period, 4001)
    path = np.array([scene.square_offset(t, side, period) for t in ts])
    assert np.max(np.linalg.norm(np.diff(path, axis=0), axis=1)) < 1e-3     # continuous
    mid = scene.square_offset(2.0, side, period)                             # first leg: +x
    assert np.isclose(mid[1], 0.0) and 0 < mid[0] < side
    assert np.allclose(scene.square_offset(2.0 + period, side, period), mid)


def test_sway_is_small_and_smooth():
    for t in np.linspace(0, 60, 50):
        offset, R = scene.sway(t, 0.02, 2.0)
        assert np.all(np.abs(offset) <= 0.02 + 1e-12)
        assert np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))) < 2.3
    assert np.allclose(scene.sway(3.0, 0.0, 0.0)[1], np.eye(3))


def test_random_pose_noise():
    rng = np.random.default_rng(0)
    assert np.array_equal(scene.random_pose_noise(rng, 0.0, 0.0), np.eye(4))
    Ts = [scene.random_pose_noise(rng, 0.002, 0.2) for _ in range(2000)]
    assert abs(np.std([T[:3, 3] for T in Ts]) - 0.002) < 2e-4
    angles = [np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1))) for T in Ts]
    assert abs(np.sqrt(np.mean(np.square(angles))) - 0.2) < 0.02


def test_registration_error():
    T_true = scene.pose_from_xyz_rpy_deg([1.2, -0.5, -0.76], [0, 0, 135])
    centroid = [0.47, -0.07, 0.0]
    assert np.allclose(scene.registration_error(T_true, T_true, centroid), (0.0, 0.0))
    shifted = make_transform(np.eye(3), [0.003, -0.004, 0.0]) @ T_true
    deg, m = scene.registration_error(shifted, T_true, centroid)
    assert np.isclose(deg, 0.0, atol=1e-6) and np.isclose(m, 0.005)
    # 1 degree about the vertical through the centroid: rotation error 1 deg, no offset there
    pivot = make_transform(scene.rpy_to_matrix(0, 0, np.radians(1.0)), [0, 0, 0])
    about_centroid = (make_transform(np.eye(3), centroid) @ pivot
                      @ make_transform(np.eye(3), -np.asarray(centroid)) @ T_true)
    deg, m = scene.registration_error(about_centroid, T_true, centroid)
    assert np.isclose(deg, 1.0) and m < 1e-9
    deg, m = scene.registration_error(about_centroid, T_true, [1.47, -0.07, 0.0])
    assert np.isclose(m, np.radians(1.0), rtol=1e-3)                 # 1 m lever arm
