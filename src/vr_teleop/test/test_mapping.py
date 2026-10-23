import numpy as np
import pytest

from ur5_common.transforms import make_transform, quat_to_matrix
from vr_teleop.mapping import ClutchMapper


def _rot(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return quat_to_matrix(*axis * np.sin(angle / 2), np.cos(angle / 2))


CTRL0 = make_transform(_rot([0.3, -1, 0.5], 1.2), [1.4, -0.3, 0.9])   # hand anywhere, any angle
TOOL0 = make_transform(_rot([1, 0, 0], np.pi), [0.5, 0.1, 0.4])       # tool pointing down


def test_pure_translation():
    m = ClutchMapper()
    m.engage(CTRL0, TOOL0)
    ctrl = CTRL0.copy()
    ctrl[:3, 3] += [0.1, -0.05, 0.02]
    T = m.target(ctrl)
    assert np.allclose(T[:3, 3], TOOL0[:3, 3] + [0.1, -0.05, 0.02])
    assert np.allclose(T[:3, :3], TOOL0[:3, :3])


def test_target_at_engage_is_tool():
    m = ClutchMapper()
    m.engage(CTRL0, TOOL0)
    assert np.allclose(m.target(CTRL0), TOOL0)


def test_rotation_delta_in_base_frame():
    m = ClutchMapper()
    m.engage(CTRL0, TOOL0)
    dR = _rot([0, 0, 1], 0.4)                       # turn the hand 0.4 rad about base z
    ctrl = make_transform(dR @ CTRL0[:3, :3], CTRL0[:3, 3])
    T = m.target(ctrl)
    assert np.allclose(T[:3, :3], dR @ TOOL0[:3, :3])
    assert np.allclose(T[:3, 3], TOOL0[:3, 3])      # turning in place doesn't move the tool


def test_scale():
    m = ClutchMapper(scale=0.5)
    m.engage(CTRL0, TOOL0)
    ctrl = CTRL0.copy()
    ctrl[:3, 3] += [0.2, 0.0, -0.1]
    assert np.allclose(m.target(ctrl)[:3, 3], TOOL0[:3, 3] + [0.1, 0.0, -0.05])


def test_workspace_clamp():
    m = ClutchMapper(workspace_min=[-1.0, -1.0, 0.1], workspace_max=[0.6, 1.0, 1.2])
    m.engage(CTRL0, TOOL0)
    ctrl = CTRL0.copy()
    ctrl[:3, 3] += [0.5, 0.2, -1.0]
    assert np.allclose(m.target(ctrl)[:3, 3], [0.6, 0.3, 0.1])


def test_workspace_one_sided_and_invalid():
    m = ClutchMapper(workspace_min=[0.0, 0.0, 0.0])
    assert np.allclose(m.clamp([5.0, -1.0, 2.0]), [5.0, 0.0, 2.0])
    with pytest.raises(ValueError):
        ClutchMapper(workspace_min=[0, 0, 1], workspace_max=[1, 1, 0])
    with pytest.raises(ValueError):
        ClutchMapper(workspace_min=[0, 0])


def test_engage_release():
    m = ClutchMapper()
    assert not m.engaged
    with pytest.raises(RuntimeError):
        m.target(CTRL0)
    m.engage(CTRL0, TOOL0)
    assert m.engaged
    m.release()
    assert not m.engaged
    # Re-engaging somewhere else re-centres: the old anchor no longer matters.
    ctrl1 = make_transform(np.eye(3), [0.0, 0.0, 0.0])
    tool1 = make_transform(np.eye(3), [0.3, 0.3, 0.3])
    m.engage(ctrl1, tool1)
    assert np.allclose(m.target(ctrl1), tool1)


def test_follow_orientation_false():
    m = ClutchMapper(follow_orientation=False)
    m.engage(CTRL0, TOOL0)
    ctrl = make_transform(_rot([1, 1, 0], 0.7) @ CTRL0[:3, :3], CTRL0[:3, 3] + [0.0, 0.1, 0.0])
    T = m.target(ctrl)
    assert np.allclose(T[:3, :3], TOOL0[:3, :3])
    assert np.allclose(T[:3, 3], TOOL0[:3, 3] + [0.0, 0.1, 0.0])
