import numpy as np

from ur5_common.transforms import make_transform, quat_to_matrix
from vr_teleop.alignment import AlignmentTracker, angle_between_deg, rotation_error_deg


def _rot(axis, deg):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    a = np.radians(deg)
    return quat_to_matrix(*axis * np.sin(a / 2), np.cos(a / 2))


def _T(p, R=None):
    return make_transform(np.eye(3) if R is None else R, p)


def test_angle_helpers():
    assert abs(angle_between_deg([1, 0, 0], [0, 2, 0]) - 90.0) < 1e-9
    assert abs(angle_between_deg([1, 0, 0], [-1, 1e-9, 0]) - 180.0) < 1e-6
    assert np.isnan(angle_between_deg([0, 0, 0], [1, 0, 0]))
    assert abs(rotation_error_deg(_rot([0, 0, 1], 30), _rot([0, 0, 1], 25)) - 5.0) < 1e-9


def test_yaw_offset_between_motions():
    """Reference moves along +x, the tool (lagging, half speed) along x rotated by 10 deg."""
    tr = AlignmentTracker(min_motion_m=0.03)
    d_tool = _rot([0, 0, 1], 10.0) @ [1.0, 0.0, 0.0]
    p_ref0, p_tool0 = np.array([1.2, -0.4, 0.9]), np.array([0.5, 0.1, 0.3])
    results = []
    for k in range(1, 40):
        m = tr.update(_T(p_ref0 + [0.01 * k, 0, 0]), _T(p_tool0 + 0.005 * k * d_tool))
        if m is not None:
            results.append(m)
    assert len(results) >= 2
    for m in results:
        assert abs(m.direction_error_deg - 10.0) < 1e-6
        assert np.linalg.norm(m.tool_motion) > 0.03 and np.linalg.norm(m.reference_motion) > 0.03
        assert m.rotation_error_deg is None
    assert tr.direction.count == len(results) and abs(tr.direction.mean - 10.0) < 1e-6


def test_registration_heading_error_with_clamp_offset():
    """Controller clamped to the tool, registration off by 7 deg of yaw: translations show 7 deg."""
    E = make_transform(_rot([0, 0, 1], 7.0), [0.02, -0.01, 0.0])     # T_base_vr error
    X = make_transform(_rot([0, 1, 0], 90.0), [0.0, 0.05, 0.12])      # tool0 -> controller
    R_tool = _rot([1, 0, 0], 180.0)
    tr = AlignmentTracker()
    m = None
    for k in range(10):
        T_tool = make_transform(R_tool, [0.4 + 0.01 * k, 0.2 - 0.005 * k, 0.3])
        m = tr.update(E @ T_tool @ X, T_tool) or m
    assert m is not None and abs(m.direction_error_deg - 7.0) < 1e-6


def test_below_threshold_gives_nothing():
    tr = AlignmentTracker(min_motion_m=0.03, min_rotation_deg=10.0)
    assert tr.update(_T([0, 0, 0]), _T([1, 1, 1])) is None
    assert tr.update(_T([0.02, 0, 0]), _T([1.02, 1, 1])) is None           # both too short
    assert tr.update(_T([0.20, 0, 0]), _T([1.01, 1, 1])) is None           # tool didn't follow
    assert tr.update(_T([0, 0, 0], _rot([0, 0, 1], 5)), _T([1, 1, 1])) is None
    assert tr.direction.count == 0 and tr.rotation.count == 0
    assert np.isnan(tr.direction.mean)


def test_rotation_error():
    tr = AlignmentTracker(min_rotation_deg=10.0)
    R0 = _rot([1, 2, 3], 40.0)
    tr.update(_T([0, 0, 0], R0), _T([1, 0, 0], np.eye(3)))
    # Reference turns 11 deg about base z, the tool only 8 deg about the same axis.
    m = tr.update(_T([0, 0, 0], _rot([0, 0, 1], 11) @ R0), _T([1, 0, 0], _rot([0, 0, 1], 8)))
    assert m is not None and m.direction_error_deg is None
    assert abs(m.rotation_error_deg - 3.0) < 1e-6
    # Re-anchored: a further 6 deg is below the threshold; tool following exactly gives 0.
    assert tr.update(_T([0, 0, 0], _rot([0, 0, 1], 17) @ R0), _T([1, 0, 0], np.eye(3))) is None
    R_ref = _rot([0, 1, 0], 12) @ _rot([0, 0, 1], 11) @ R0
    R_tool = _rot([0, 1, 0], 12) @ _rot([0, 0, 1], 8)
    m = tr.update(_T([0, 0, 0], R_ref), _T([1, 0, 0], R_tool))
    assert abs(m.rotation_error_deg) < 1e-6
    assert tr.rotation.count == 2 and abs(tr.rotation.max - 3.0) < 1e-6


def test_stats_and_reset():
    tr = AlignmentTracker(min_motion_m=0.03)
    tr.update(_T([0, 0, 0]), _T([0, 0, 0]))
    m1 = tr.update(_T([0.1, 0, 0]), _T(0.1 * (_rot([0, 0, 1], 10) @ [1, 0, 0])))
    p_ref, p_tool = np.array([0.1, 0, 0]), m1.tool_motion
    m2 = tr.update(_T(p_ref + [0, 0.1, 0]), _T(p_tool + 0.1 * (_rot([1, 0, 0], 20) @ [0, 1, 0])))
    assert abs(m1.direction_error_deg - 10) < 1e-6 and abs(m2.direction_error_deg - 20) < 1e-6
    s = tr.direction
    assert s.count == 2 and abs(s.mean - 15) < 1e-6 and abs(s.max - 20) < 1e-6
    assert str(s) == '20.0 deg (mean 15.0, max 20.0, n=2)'

    tr.reset_anchor()      # keeps stats, next sample is the new anchor
    assert tr.update(_T([5, 5, 5]), _T([-5, -5, -5])) is None and tr.direction.count == 2
    tr.reset()
    assert tr.direction.count == 0 and tr.rotation.count == 0
    assert tr.update(_T([0, 0, 0]), _T([0, 0, 0])) is None   # far from the old anchor: no stroke
