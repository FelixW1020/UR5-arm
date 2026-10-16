"""Robot <-> VR registration math (pure numpy/OpenCV, no ROS).

All results are T_base_vr: the pose of the VR world frame in the robot base frame,
so a point measured in VR maps into the robot frame as  p_base = T_base_vr @ p_vr.

vr_world uses the ROS convention (x forward, y left, z up). Unity is converted on the
Quest side with ROS-TCP-Connector's FLU helpers before anything reaches ROS.
"""
from dataclasses import dataclass

import cv2
import numpy as np

from ur5_common.transforms import invert, make_transform

MODES = ('yaw_only', 'full')


@dataclass
class Registration:
    T_base_vr: np.ndarray
    errors_m: np.ndarray           # per point / per sample
    method: str

    @property
    def rms_error_m(self):
        return float(np.sqrt(np.mean(self.errors_m ** 2)))

    @property
    def max_error_m(self):
        return float(np.max(self.errors_m))


def fit_points(p_vr, p_base, mode='yaw_only'):
    """Rigid fit of corresponding points, p_base ~= R p_vr + t (no scale: both are metric).

    yaw_only: rotation about z only (4 DOF). Both frames are gravity-aligned (UR base on a
        level table; the Quest's floor frame is levelled by its IMU), so this is much more
        robust to the cm-level noise of VR measurements. Needs >= 2 points.
    full: unconstrained 6-DOF fit (Kabsch). Needs >= 3 non-collinear points.
    """
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}')
    P = np.asarray(p_vr, dtype=float).reshape(-1, 3)
    Q = np.asarray(p_base, dtype=float).reshape(-1, 3)
    if len(P) != len(Q):
        raise ValueError('point lists differ in length')
    need = 2 if mode == 'yaw_only' else 3
    if len(P) < need:
        raise ValueError(f'{mode} needs at least {need} points, got {len(P)}')

    cp, cq = P.mean(axis=0), Q.mean(axis=0)
    A, B = P - cp, Q - cq
    if mode == 'yaw_only':
        if np.max(np.linalg.norm(A[:, :2], axis=1)) < 0.05:
            raise ValueError('points are (nearly) vertically stacked; spread them out horizontally')
        theta = np.arctan2(np.sum(A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]),
                           np.sum(A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]))
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    else:
        U, S, Vt = np.linalg.svd(A.T @ B)
        if S[1] < 1e-3 * max(S[0], 1e-12):
            raise ValueError('points are (nearly) collinear; use at least 3 tags not in a line')
        D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
    T = make_transform(R, cq - R @ cp)
    errors = np.linalg.norm((P @ R.T + T[:3, 3]) - Q, axis=1)
    return Registration(T, errors, f'points_{mode}')


if not hasattr(cv2, 'calibrateRobotWorldHandEye'):
    raise ImportError(f'OpenCV {cv2.__version__} lacks cv2.calibrateRobotWorldHandEye; '
                      'install OpenCV 4.x (pip install "opencv-python<5").')


def fit_controller_on_flange(T_base_tool0, T_vr_controller):
    """Quest controller rigidly clamped to the tool flange, arm moved through N poses.

    Solves the robot-world/hand-eye problem AX = ZB for both unknowns:
      T_base_vr (wanted) and T_tool0_controller (the clamp offset, a by-product).
    Mapping onto OpenCV's naming: its moving "camera" is our controller, its fixed
    "world" is vr_world, its "gripper" is tool0.
    Returns (Registration, T_tool0_controller).
    """
    if len(T_base_tool0) != len(T_vr_controller):
        raise ValueError('sample lists differ in length')
    if len(T_base_tool0) < 3:
        raise ValueError(f'need at least 3 samples, got {len(T_base_tool0)}')
    world2cam = [invert(T) for T in T_vr_controller]        # T_controller_vr
    base2gripper = [invert(T) for T in T_base_tool0]         # T_tool0_base
    R_b2w, t_b2w, R_g2c, t_g2c = cv2.calibrateRobotWorldHandEye(
        [T[:3, :3] for T in world2cam], [T[:3, 3].reshape(3, 1) for T in world2cam],
        [T[:3, :3] for T in base2gripper], [T[:3, 3].reshape(3, 1) for T in base2gripper])
    T_vr_base = make_transform(R_b2w, t_b2w)                 # base -> world, i.e. pose of base in vr
    T_controller_tool0 = make_transform(R_g2c, t_g2c)
    T_base_vr = invert(T_vr_base)
    T_tool0_controller = invert(T_controller_tool0)
    # Error: where the controller should be (via robot kinematics) vs where VR saw it.
    errors = np.array([
        np.linalg.norm((T_base_vr @ Tvc)[:3, 3] - (Tbt @ T_tool0_controller)[:3, 3])
        for Tbt, Tvc in zip(T_base_tool0, T_vr_controller)])
    return Registration(T_base_vr, errors, 'controller_on_flange'), T_tool0_controller
