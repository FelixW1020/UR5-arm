"""Controller -> tool target mapping for teleoperation (pure numpy, no ROS).

Relative ("clutch") mapping: while the grip is held, the tool target moves by the controller's
displacement since the grip was pressed and turns by the controller's rotation since then, both
measured in base_link. The controller never has to be aligned with the tool, and releasing and
re-gripping re-centres the hand without moving the arm.

Because only displacements are used, the mapping is a direct test of the registration
(base_link -> vr_world): with a correct registration, pushing the controller forward pushes the
arm forward; a heading error in the registration makes the arm move off at that angle.
"""
import numpy as np

from ur5_common.transforms import make_transform, rotation_angle


def _vec3(v, name):
    if v is None:
        return None
    v = np.asarray(v, dtype=float).reshape(-1)
    if v.shape != (3,):
        raise ValueError(f'{name} must have 3 elements, got {v.size}')
    return v


# A registration (base_link -> vr_world) that moved by more than this was replaced.
REGISTRATION_CHANGE_M, REGISTRATION_CHANGE_DEG = 0.001, 0.05


def pose_change(T_a, T_b):
    """(distance m, angle deg) from pose T_a to pose T_b."""
    T_a, T_b = np.asarray(T_a, dtype=float), np.asarray(T_b, dtype=float)
    return (float(np.linalg.norm(T_b[:3, 3] - T_a[:3, 3])),
            float(np.degrees(rotation_angle(T_b[:3, :3] @ T_a[:3, :3].T))))


class ClutchMapper:

    def __init__(self, scale=1.0, follow_orientation=True, workspace_min=None, workspace_max=None):
        self.scale = float(scale)
        self.follow_orientation = bool(follow_orientation)
        self.workspace_min = _vec3(workspace_min, 'workspace_min')
        self.workspace_max = _vec3(workspace_max, 'workspace_max')
        if (self.workspace_min is not None and self.workspace_max is not None
                and np.any(self.workspace_min > self.workspace_max)):
            raise ValueError(f'workspace_min {self.workspace_min} > workspace_max '
                             f'{self.workspace_max}')
        self._T_ctrl0 = None
        self._T_tool0 = None

    @property
    def engaged(self):
        return self._T_ctrl0 is not None

    def engage(self, T_base_ctrl, T_base_tool):
        """Grip pressed: remember where the controller and the tool are now."""
        self._T_ctrl0 = np.array(T_base_ctrl, dtype=float)
        self._T_tool0 = np.array(T_base_tool, dtype=float)

    def release(self):
        self._T_ctrl0 = self._T_tool0 = None

    def clamp(self, p):
        """Clamp a position into the workspace box (either bound may be None)."""
        p = np.asarray(p, dtype=float)
        if self.workspace_min is not None:
            p = np.maximum(p, self.workspace_min)
        if self.workspace_max is not None:
            p = np.minimum(p, self.workspace_max)
        return p

    def in_workspace(self, p):
        return bool(np.allclose(self.clamp(p), p))

    def target(self, T_base_ctrl):
        """T_base_target for the current controller pose (call only while engaged)."""
        if not self.engaged:
            raise RuntimeError('ClutchMapper.target() called while not engaged')
        T_base_ctrl = np.asarray(T_base_ctrl, dtype=float)
        p = self._T_tool0[:3, 3] + self.scale * (T_base_ctrl[:3, 3] - self._T_ctrl0[:3, 3])
        R = self._T_tool0[:3, :3]
        if self.follow_orientation:
            R = (T_base_ctrl[:3, :3] @ self._T_ctrl0[:3, :3].T) @ R
        return make_transform(R, self.clamp(p))
