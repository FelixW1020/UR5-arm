import xml.etree.ElementTree as ET

import numpy as np
import pytest

from vr_sim import ur_kinematics as uk


def _rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def _rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1.0, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def _ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s, 0], [0, 1.0, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


def _trans(x, y, z):
    T = np.eye(4)
    T[:3, 3] = x, y, z
    return T


def _origin(xyz, rpy):
    """URDF <origin>: translate, then fixed-axis rpy."""
    return _trans(*xyz) @ _rz(rpy[2]) @ _ry(rpy[1]) @ _rx(rpy[0])


def _random_q(rng):
    q = rng.uniform(-np.pi, np.pi, 6)
    q[2] = rng.choice([-1, 1]) * rng.uniform(0.4, 2.7)      # away from the elbow singularity
    q[4] = rng.choice([-1, 1]) * rng.uniform(0.4, 2.7)      # and the wrist singularity
    return q


def test_fk_at_zero_ur5():
    T = uk.fk(np.zeros(6), 'ur5')
    assert np.allclose(T[:3, 3], [0.81725, 0.19145, -0.005491], atol=1e-9)
    assert np.allclose(T[:3, 2], [0.0, 1.0, 0.0], atol=1e-12)    # flange faces +y in base_link


@pytest.mark.parametrize('ur_type', sorted(uk.DH_PARAMS))
def test_fk_matches_independent_dh_chain(ur_type):
    d1, a2, a3, d4, d5, d6 = uk.DH_PARAMS[ur_type]
    d, a = [d1, 0, 0, d4, d5, d6], [0, a2, a3, 0, 0, 0]
    alpha = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 6)
        T = _rz(np.pi)
        for i in range(6):
            T = T @ _rz(q[i]) @ _trans(0, 0, d[i]) @ _trans(a[i], 0, 0) @ _rx(alpha[i])
        assert np.allclose(uk.fk(q, ur_type), T, atol=1e-12)


def test_fk_matches_ur_description_ur5_chain():
    """Chain built like ur_description's URDF (config/ur5/default_kinematics.yaml origins)."""
    origins = [((0, 0, 0.089159), (0, 0, 0)),
               ((0, 0, 0), (1.570796327, 0, 0)),
               ((-0.425, 0, 0), (0, 0, 0)),
               ((-0.39225, 0, 0.10915), (0, 0, 0)),
               ((0, -0.09465, -1.941303950897609e-11), (1.570796327, 0, 0)),
               ((0, 0.0823, -1.688001216681175e-11), (1.570796326589793, np.pi, np.pi))]
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 6)
        T = _rz(np.pi)                                      # base_link -> base_link_inertia
        for (xyz, rpy), qi in zip(origins, q):
            T = T @ _origin(xyz, rpy) @ _rz(qi)
        T = T @ _origin((0, 0, 0), (0, -np.pi / 2, -np.pi / 2))   # wrist_3_link -> flange
        T = T @ _origin((0, 0, 0), (np.pi / 2, 0, np.pi / 2))     # flange -> tool0
        Tf = uk.fk(q, 'ur5')
        assert np.linalg.norm(Tf[:3, 3] - T[:3, 3]) < 1e-8
        assert np.allclose(Tf[:3, :3], T[:3, :3], atol=1e-8)


def _urdf_chain(urdf, base='base_link', tip='tool0'):
    joints = {j.find('child').get('link'): j for j in ET.fromstring(urdf).findall('joint')}
    chain, link = [], tip
    while link != base:
        j = joints[link]
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
        chain.append((_origin(xyz, rpy), j.get('type') != 'fixed'))
        link = j.find('parent').get('link')
    return chain[::-1]


@pytest.mark.parametrize('ur_type', ['ur5', 'ur5e', 'ur10e'])
def test_fk_matches_installed_urdf(ur_type):
    """Ground truth: the URDF that robot_state_publisher uses (runs where ur_description is)."""
    xacro = pytest.importorskip('xacro')
    packages = pytest.importorskip('ament_index_python.packages')
    try:
        share = packages.get_package_share_directory('ur_description')
    except packages.PackageNotFoundError:
        pytest.skip('ur_description not installed')
    doc = xacro.process_file(f'{share}/urdf/ur.urdf.xacro',
                             mappings={'name': ur_type, 'ur_type': ur_type})
    chain = _urdf_chain(doc.toxml())
    assert sum(movable for _, movable in chain) == 6
    rng = np.random.default_rng(2)
    for _ in range(20):
        q = iter(rng.uniform(-np.pi, np.pi, 6))
        qs, T = [], np.eye(4)
        for origin, movable in chain:
            T = T @ origin
            if movable:
                qs.append(next(q))
                T = T @ _rz(qs[-1])
        Tf = uk.fk(qs, ur_type)
        assert np.linalg.norm(Tf[:3, 3] - T[:3, 3]) < 1e-6
        assert np.allclose(Tf[:3, :3], T[:3, :3], atol=1e-6)


def test_jacobian_matches_finite_differences():
    rng = np.random.default_rng(3)
    q = _random_q(rng)
    J = uk.jacobian(q)
    T0, h = uk.fk(q), 1e-7
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        T1 = uk.fk(q + dq)
        assert np.allclose((T1[:3, 3] - T0[:3, 3]) / h, J[:3, i], atol=1e-5)
        assert np.allclose(uk.rotation_log(T1[:3, :3] @ T0[:3, :3].T) / h, J[3:, i], atol=1e-5)


def test_ik_converges_from_nearby_start():
    rng = np.random.default_rng(4)
    for ur_type in ('ur5', 'ur5e'):
        for _ in range(40):
            q_true = _random_q(rng)
            T = uk.fk(q_true, ur_type)
            q0 = q_true + rng.normal(scale=0.25, size=6)
            q, pos_err, rot_err = uk.solve_ik(q0, T, ur_type)
            assert pos_err < 1e-4 and rot_err < 1e-3
            Tq = uk.fk(q, ur_type)
            assert np.linalg.norm(Tq[:3, 3] - T[:3, 3]) < 1e-4


def test_ik_respects_limits_and_returns_closest_for_unreachable():
    rng = np.random.default_rng(5)
    q0 = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
    far = uk.fk(q0)
    far[:3, 3] = [2.0, 0.5, 0.3]                            # outside the 0.85 m reach
    start_err = np.linalg.norm(far[:3, 3] - uk.fk(q0)[:3, 3])
    q, pos_err, _ = uk.solve_ik(q0, far, max_iterations=200)
    assert np.all(q >= uk.JOINT_LOWER) and np.all(q <= uk.JOINT_UPPER)
    assert 0.9 < pos_err < start_err - 0.3                 # stretched toward it, not reached
    # Out-of-range start is clamped; a target behind the elbow limit stays within limits.
    q_start = np.array([7.0, 0.0, 4.0, 0.0, 0.0, -7.0])
    for _ in range(10):
        q, _, _ = uk.solve_ik(q_start, uk.fk(_random_q(rng)), max_iterations=50)
        assert np.all(q >= uk.JOINT_LOWER - 1e-12) and np.all(q <= uk.JOINT_UPPER + 1e-12)
    assert np.allclose(uk.clamp_to_limits(q_start), [2 * np.pi, 0, np.pi, 0, 0, -2 * np.pi])


def test_ik_step_limit():
    q0 = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
    T = uk.fk(q0 + [0.8, 0.3, -0.4, 0.2, 0.5, 0.6])
    q, _, _ = uk.solve_ik(q0, T, max_iterations=1, max_step=0.05)
    assert np.max(np.abs(q - q0)) <= 0.05 + 1e-12


def test_limit_joint_step_keeps_direction():
    dq = np.array([0.5, -0.1, 0.02, 0.0, 0.3, -0.04])
    out = uk.limit_joint_step(dq, 0.02)
    assert abs(np.max(np.abs(out)) - 0.02) < 1e-12
    assert abs(np.dot(out, dq) / (np.linalg.norm(out) * np.linalg.norm(dq)) - 1.0) < 1e-12
    small = dq * 0.01
    assert np.array_equal(uk.limit_joint_step(small, 0.02), small)


def test_velocity_limited_arm_follows_a_straight_line():
    """ur5_kinematic_sim's tick loop: a fast (1 m/s) straight hand move keeps tool0 near the line."""
    rate, max_step = 50.0, 1.0 / 50.0
    q = q_goal = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
    T0 = uk.fk(q)
    p0, direction = T0[:3, 3].copy(), np.array([1.0, 0.0, 0.0])
    worst = 0.0
    for k in range(int(3.2 * rate)):
        T = T0.copy()
        T[:3, 3] = p0 + direction * min(0.2, k / rate)
        q_goal, _, _ = uk.solve_ik(q_goal, T, max_iterations=10)
        q = q + uk.limit_joint_step(q_goal - q, max_step)
        off = uk.fk(q)[:3, 3] - p0
        worst = max(worst, np.linalg.norm(off - np.dot(off, direction) * direction))
    assert np.linalg.norm(uk.fk(q)[:3, 3] - (p0 + 0.2 * direction)) < 1e-3
    assert worst < 0.015            # clipping each joint on its own strays ~6 cm


def _axis(rng):
    a = rng.normal(size=3)
    return a / np.linalg.norm(a)


@pytest.mark.parametrize('angle', [0.0, 1e-12, 1e-7, 1e-3, 0.5, 1.5, np.pi / 2, 2.5,
                                   np.pi - 1e-3, np.pi - 1e-6, np.pi - 1e-9])
def test_rotation_log_round_trip(angle):
    rng = np.random.default_rng(6)
    for _ in range(20):
        w = _axis(rng) * angle
        R = uk.rotation_exp(w)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert np.allclose(uk.rotation_log(R), w, atol=1e-8)


def test_rotation_log_at_pi():
    rng = np.random.default_rng(7)
    for _ in range(20):
        n = _axis(rng)
        R = 2.0 * np.outer(n, n) - np.eye(3)                # exactly pi about n
        w = uk.rotation_log(R)
        assert abs(np.linalg.norm(w) - np.pi) < 1e-9
        assert np.allclose(np.abs(w), np.pi * np.abs(n), atol=1e-8)   # +-n are both correct
        assert np.allclose(uk.rotation_exp(w), R, atol=1e-9)


def test_unknown_ur_type():
    with pytest.raises(ValueError):
        uk.fk(np.zeros(6), 'ur99')
