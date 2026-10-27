import numpy as np
import pytest

from ur5_common.transforms import invert, make_transform, quat_to_matrix, rotation_angle
from vr_registration.registration import (Registration, average_tag_observations, fit_points,
                                          fit_tags)


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return quat_to_matrix(*axis * np.sin(angle / 2), np.cos(angle / 2))


def _Rz(theta):
    return _axis_angle([0, 0, 1], theta)


def _rand_R(rng, max_angle=np.pi):
    return _axis_angle(rng.normal(size=3), rng.uniform(-max_angle, max_angle))


def _tag_poses(rng, n):
    """Tags lying face-up on the table with random yaw: R_base_tag = Rz(psi)."""
    return [make_transform(_Rz(rng.uniform(-np.pi, np.pi)), [rng.uniform(0.2, 0.8),
                           rng.uniform(-0.5, 0.5), rng.uniform(-0.02, 0.02)])
            for _ in range(n)]


def _observe(T_base_vr, tags):
    """What the Quest would report: tag poses in vr_world."""
    T_vr_base = invert(T_base_vr)
    return [T_vr_base @ T for T in tags]


def _split(Ts):
    return [T[:3, 3] for T in Ts], [T[:3, :3] for T in Ts]


T_YAW = make_transform(_Rz(2.1), [1.5, -0.7, -0.8])    # Quest floor origin 0.8 m below the base


def test_single_tag_yaw_only_exact():
    tag = make_transform(_Rz(0.3), [0.5, 0.2, 0.01])
    (p_vr,), (R_vr,) = _split(_observe(T_YAW, [tag]))
    r = fit_tags([p_vr], [tag[:3, 3]], [R_vr], [tag[:3, :3]], 'yaw_only')
    assert np.allclose(r.T_base_vr, T_YAW, atol=1e-9)
    assert r.method == 'tags_yaw_only'
    assert r.max_error_m < 1e-9
    assert r.rotation_errors_deg.shape == (1,) and r.rotation_errors_deg[0] < 1e-4


def test_single_tag_full_exact():
    rng = np.random.default_rng(10)
    T_true = make_transform(_rand_R(rng), rng.normal(scale=1.0, size=3))
    tag = make_transform(_rand_R(rng), [0.4, -0.3, 0.2])       # any orientation, e.g. on a wall
    (p_vr,), (R_vr,) = _split(_observe(T_true, [tag]))
    for w in (1e-3, 0.1, 10.0):                                # one tag: R = Rb Ra^T for any w
        r = fit_tags([p_vr], [tag[:3, 3]], [R_vr], [tag[:3, :3]], 'full', rotation_weight=w)
        assert np.allclose(r.T_base_vr[:3, :3], tag[:3, :3] @ R_vr.T, atol=1e-9)
        assert np.allclose(r.T_base_vr, T_true, atol=1e-9)
        assert r.method == 'tags_full'


def test_single_tag_with_noise():
    """3 mm / 1 deg on one sighted tag -> about 1 deg and a few mm at the tag."""
    rng = np.random.default_rng(11)
    T_true = make_transform(_Rz(-0.6) @ _axis_angle([1, 0, 0], 0.01), [0.9, 0.3, -0.75])
    rot_err = {'yaw_only': [], 'full': []}
    for _ in range(200):
        tag = _tag_poses(rng, 1)[0]
        T_obs = _observe(T_true, [tag])[0]
        p_vr = T_obs[:3, 3] + rng.normal(scale=0.003, size=3)
        R_vr = _axis_angle(rng.normal(size=3), np.radians(1.0)) @ T_obs[:3, :3]
        for mode in rot_err:
            r = fit_tags([p_vr], [tag[:3, 3]], [R_vr], [tag[:3, :3]], mode)
            T = r.T_base_vr
            rot_err[mode].append(np.degrees(rotation_angle(T[:3, :3].T @ T_true[:3, :3])))
            # The tag centre maps back to within the position noise.
            assert np.linalg.norm((T @ np.append(T_obs[:3, 3], 1))[:3] - tag[:3, 3]) < 0.015
            if mode == 'yaw_only':
                assert np.allclose(T[2, :3], [0, 0, 1])
                # Residual = noise + the 0.57 deg Quest tilt that a yaw cannot absorb.
                assert r.rotation_errors_deg[0] < 1.0 + 0.6 + 1e-6
            else:
                assert r.rotation_errors_deg[0] < 1e-4        # one tag is fitted exactly
    assert np.max(rot_err['full']) < 1.0 + 1e-6               # exactly the injected noise
    assert np.mean(rot_err['yaw_only']) < 1.2                 # yaw part of noise + tilt
    assert np.max(rot_err['yaw_only']) < 1.6


def test_several_tags_positions_and_orientations_exact():
    rng = np.random.default_rng(12)
    for mode, T_true in (('yaw_only', T_YAW),
                         ('full', make_transform(_rand_R(rng), rng.normal(size=3)))):
        tags = _tag_poses(rng, 4)
        p_vr, R_vr = _split(_observe(T_true, tags))
        p_base, R_base = _split(tags)
        # Positions of all four, orientations of two of them.
        r = fit_tags(p_vr, p_base, R_vr[1:3], R_base[1:3], mode)
        assert np.allclose(r.T_base_vr, T_true, atol=1e-9)
        assert r.method == f'tags_{mode}'
        assert r.errors_m.shape == (4,) and r.max_error_m < 1e-9
        assert r.rotation_errors_deg.shape == (2,) and np.all(r.rotation_errors_deg < 1e-4)


def test_rotation_weight_trades_positions_against_orientations():
    """Positions say yaw 0.5, the tag orientation says 0.6: w moves the answer between them."""
    rng = np.random.default_rng(13)
    tags = _tag_poses(rng, 3)
    p_vr, _ = _split(_observe(make_transform(_Rz(0.5), [1, 0, 0]), tags))
    (R_vr,) = _split(_observe(make_transform(_Rz(0.6), [1, 0, 0]), tags[:1]))[1]
    p_base, R_base = _split(tags)

    def yaw(w):
        T = fit_tags(p_vr, p_base, [R_vr], R_base[:1], 'yaw_only', w).T_base_vr
        return np.arctan2(T[1, 0], T[0, 0])

    assert abs(yaw(1e-6) - 0.5) < 1e-3
    assert abs(yaw(1e3) - 0.6) < 1e-3
    assert 0.5 < yaw(0.1) < 0.6
    assert yaw(0.01) < yaw(0.1) < yaw(1.0)


def test_rotation_errors_reported():
    rng = np.random.default_rng(14)
    tags = _tag_poses(rng, 3)
    p_vr, R_vr = _split(_observe(T_YAW, tags))
    p_base, R_base = _split(tags)
    R_vr[1] = _Rz(np.radians(5.0)) @ R_vr[1]                  # tag 1's orientation is 5 deg off
    r = fit_tags(p_vr, p_base, R_vr, R_base, 'yaw_only', rotation_weight=1e-6)
    assert np.allclose(r.rotation_errors_deg, [0.0, 5.0, 0.0], atol=1e-3)
    assert r.max_error_m < 1e-5                               # positions still fit
    # rotation_weight 0: orientations ignored by the fit but still checked against it.
    r0 = fit_tags(p_vr, p_base, R_vr, R_base, 'yaw_only', rotation_weight=0.0)
    assert r0.method == 'points_yaw_only'
    assert np.allclose(r0.T_base_vr, T_YAW, atol=1e-9)
    assert np.allclose(r0.rotation_errors_deg, [0.0, 5.0, 0.0], atol=1e-6)


def test_without_rotations_equals_fit_points():
    rng = np.random.default_rng(15)
    for mode in ('yaw_only', 'full'):
        p_base = np.array([T[:3, 3] for T in _tag_poses(rng, 5)])
        T_vr_base = invert(T_YAW)
        p_vr = p_base @ T_vr_base[:3, :3].T + T_vr_base[:3, 3] + rng.normal(scale=0.01, size=(5, 3))
        a, b = fit_tags(p_vr, p_base, mode=mode), fit_points(p_vr, p_base, mode)
        assert np.allclose(a.T_base_vr, b.T_base_vr, atol=1e-12)
        assert np.allclose(a.errors_m, b.errors_m, atol=1e-12)
        assert a.method == b.method == f'points_{mode}'
        assert a.rotation_errors_deg.shape == b.rotation_errors_deg.shape == (0,)


def test_fit_points_messages_unchanged():
    with pytest.raises(ValueError, match=r'^yaw_only needs at least 2 points, got 1$'):
        fit_points([[0, 0, 0]], [[0, 0, 0]], 'yaw_only')
    with pytest.raises(ValueError, match=r'^full needs at least 3 points, got 2$'):
        fit_points([[0, 0, 0], [1, 0, 0]], [[0, 0, 0], [1, 0, 0]], 'full')
    with pytest.raises(ValueError, match=r'^points are \(nearly\) collinear; use at least 3 tags '
                                         r'not in a line$'):
        fit_points([[0, 0, 0], [1, 0, 0], [2, 0, 0]], [[0, 0, 0], [1, 0, 0], [2, 0, 0]], 'full')
    with pytest.raises(ValueError, match=r'^points are \(nearly\) vertically stacked; spread them '
                                         r'out horizontally$'):
        fit_points([[0, 0, 0], [0, 0, 1]], [[0, 0, 0], [0, 0, 1]], 'yaw_only')
    with pytest.raises(ValueError, match=r'^point lists differ in length$'):
        fit_points([[0, 0, 0]], [[0, 0, 0], [1, 0, 0]])
    with pytest.raises(ValueError, match='mode must be one of'):
        fit_points([[0, 0, 0], [1, 0, 0]], [[0, 0, 0], [1, 0, 0]], 'yaw')


def test_fit_tags_rejects_degenerate():
    tag = make_transform(_Rz(0.3), [0.5, 0.2, 0.01])
    p, R = tag[:3, 3], tag[:3, :3]
    for mode in ('yaw_only', 'full'):
        # One tag without its orientation: the error says what would make it work.
        with pytest.raises(ValueError, match='orientation'):
            fit_tags([p], [p], mode=mode)
        with pytest.raises(ValueError, match='at least one tag position'):
            fit_tags([], [], [R], [R], mode)
        with pytest.raises(ValueError, match='rotation_weight is 0'):
            fit_tags([p], [p], [R], [R], mode, rotation_weight=0.0)
    with pytest.raises(ValueError, match='orientation lists differ'):
        fit_tags([p], [p], [R, R], [R])
    with pytest.raises(ValueError, match='point lists differ'):
        fit_tags([p, p], [p], [R], [R])
    for w in (-0.1, np.nan, np.inf):
        with pytest.raises(ValueError, match='rotation_weight'):
            fit_tags([p], [p], [R], [R], rotation_weight=w)
    with pytest.raises(ValueError, match='mode must be one of'):
        fit_tags([p], [p], [R], [R], 'yaw')
    # Tag reported upside down: no heading explains it.
    with pytest.raises(ValueError, match='heading'):
        fit_tags([p], [p], [_axis_angle([1, 0, 0], np.pi) @ R], [R], 'yaw_only')
    # Collinear tags whose single orientation barely counts.
    line = [[0, 0, 0], [0.5, 0, 0], [1.0, 0, 0]]
    with pytest.raises(ValueError, match='ill-determined'):
        fit_tags(line, line, [R], [R], 'full', rotation_weight=1e-9)
    assert np.allclose(fit_tags(line, line, [R], [R], 'full').T_base_vr, np.eye(4), atol=1e-9)


def test_registration_backward_compatible():
    r = Registration(np.eye(4), np.array([0.01, 0.02]), 'points_yaw_only')
    assert r.rotation_errors_deg.shape == (0,)
    assert abs(r.max_error_m - 0.02) < 1e-12
    assert abs(r.rms_error_m - np.sqrt((0.01 ** 2 + 0.02 ** 2) / 2)) < 1e-12


def test_average_tag_observations():
    R0 = _Rz(0.4)
    tilt = np.radians(3.0)
    obs = [
        (3, [1.0, 0.0, 0.0], _axis_angle([1, 0, 0], tilt) @ R0),
        (3, [1.2, 0.2, 0.0], _axis_angle([1, 0, 0], -tilt) @ R0),
        (3, [1.1, 0.1, 0.3], None),                    # controller touch: position only
        (7, [0.0, 0.5, 0.0], None),
        (7, [0.0, 0.7, 0.2], None),
        (9, np.array([0.3, 0.3, 0.3]), make_transform(R0, [9, 9, 9])),   # 4x4 is fine too
    ]
    avg = average_tag_observations(obs)
    assert sorted(avg) == [3, 7, 9]
    p3, R3 = avg[3]
    assert np.allclose(p3, [1.1, 0.1, 0.1])            # mean over all three, touch included
    assert np.allclose(R3, R0, atol=1e-12)             # symmetric tilts cancel
    assert np.allclose(R3 @ R3.T, np.eye(3)) and np.linalg.det(R3) > 0
    p7, R7 = avg[7]
    assert np.allclose(p7, [0.0, 0.6, 0.1]) and R7 is None
    p9, R9 = avg[9]
    assert np.allclose(p9, [0.3, 0.3, 0.3]) and np.allclose(R9, R0)
    assert average_tag_observations([]) == {}


def test_average_then_fit_single_tag():
    """Several noisy sightings of one tag, averaged, register better than one sighting."""
    rng = np.random.default_rng(16)
    tag = make_transform(_Rz(1.0), [0.6, -0.1, 0.0])
    T_obs = _observe(T_YAW, [tag])[0]
    one, many = [], []
    for _ in range(100):
        obs = [(0, T_obs[:3, 3] + rng.normal(scale=0.003, size=3),
                _axis_angle(rng.normal(size=3), np.radians(2.0)) @ T_obs[:3, :3])
               for _ in range(10)]
        for sightings, out in ((obs[:1], one), (obs, many)):
            p, R = average_tag_observations(sightings)[0]
            T = fit_tags([p], [tag[:3, 3]], [R], [tag[:3, :3]], 'full').T_base_vr
            out.append(np.degrees(rotation_angle(T[:3, :3].T @ T_YAW[:3, :3])))
    assert np.mean(many) < 0.5 * np.mean(one)
