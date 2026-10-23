"""Robot <-> VR registration math (pure numpy/OpenCV, no ROS).

All results are T_base_vr: the pose of the VR world frame in the robot base frame,
so a point measured in VR maps into the robot frame as  p_base = T_base_vr @ p_vr.

vr_world uses the ROS convention (x forward, y left, z up). Unity is converted on the
Quest side with ROS-TCP-Connector's FLU helpers before anything reaches ROS.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

from ur5_common.transforms import (invert, make_transform, project_to_rotation,
                                   rotation_angle)

MODES = ('yaw_only', 'full')


@dataclass
class Registration:
    T_base_vr: np.ndarray
    errors_m: np.ndarray           # per point / per sample
    method: str
    rotation_errors_deg: np.ndarray = field(default_factory=lambda: np.zeros(0))  # per tag pair

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
    return _fit(p_vr, p_base, np.zeros((0, 3, 3)), np.zeros((0, 3, 3)), mode, 0.0)


def fit_tags(p_vr, p_base, R_vr=(), R_base=(), mode='yaw_only', rotation_weight=0.1):
    """fit_points plus tag orientations (R_vr[j] = R_vr_tag, R_base[j] = R_base_tag of a tag).

    A tag the headset *sees* (rather than touches) comes with its orientation, which pins the
    rotation on its own: one such tag registers vr_world. Minimises (weighted Procrustes)
        sum_i |R p_vr_i + t - p_base_i|^2 + w sum_j |R R_vr_j - R_base_j|_F^2
    The orientation term acts like three extra points sqrt(w) metres out along the tag's axes, so
    w = rotation_weight (m^2) = 0.1 weighs one tag's orientation like 3 points 32 cm from its
    centre; 1 deg of orientation error then costs as much as 7.8 mm of position error.
    With orientations either mode works from a single tag; without them (or with w = 0) the
    requirements of fit_points apply. rotation_errors_deg: angle(R R_vr_j, R_base_j) per pair.
    """
    Ra = np.asarray(R_vr, dtype=float).reshape(-1, 3, 3)
    Rb = np.asarray(R_base, dtype=float).reshape(-1, 3, 3)
    if len(Ra) != len(Rb):
        raise ValueError('orientation lists differ in length')
    w = float(rotation_weight)
    if not (np.isfinite(w) and w >= 0.0):
        raise ValueError(f'rotation_weight must be a finite number >= 0, got {rotation_weight}')
    hint = ('; tag orientations are ignored because rotation_weight is 0' if len(Ra)
            else '; or give a tag orientation (one oriented tag is enough)')
    return _fit(p_vr, p_base, Ra, Rb, mode, w, hint)


def _fit(p_vr, p_base, Ra, Rb, mode, w, hint=''):
    """Shared solver: R maximises tr(R H), H = A^T B + w sum_j Ra_j Rb_j^T; t from centroids."""
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}')
    P = np.asarray(p_vr, dtype=float).reshape(-1, 3)
    Q = np.asarray(p_base, dtype=float).reshape(-1, 3)
    if len(P) != len(Q):
        raise ValueError('point lists differ in length')
    oriented = w > 0.0 and len(Ra) > 0
    if oriented:
        if len(P) == 0:
            raise ValueError('need at least one tag position (orientations do not fix the '
                             'translation)')
    else:
        need = 2 if mode == 'yaw_only' else 3
        if len(P) < need:
            raise ValueError(f'{mode} needs at least {need} points, got {len(P)}{hint}')

    cp, cq = P.mean(axis=0), Q.mean(axis=0)
    A, B = P - cp, Q - cq
    H = A.T @ B
    if oriented:
        H = H + w * np.sum(Ra @ Rb.transpose(0, 2, 1), axis=0)
    if mode == 'yaw_only':
        if not oriented and np.max(np.linalg.norm(A[:, :2], axis=1)) < 0.05:
            raise ValueError('points are (nearly) vertically stacked; spread them out '
                             f'horizontally{hint}')
        s, c = H[0, 1] - H[1, 0], H[0, 0] + H[1, 1]
        if oriented:
            # |(c, s)| <= scale; it collapses when orientations contradict a pure yaw (flipped tag).
            scale = (np.sum(np.linalg.norm(A[:, :2], axis=1) * np.linalg.norm(B[:, :2], axis=1))
                     + 2.0 * w * len(Ra))
            if np.hypot(s, c) < 1e-3 * scale:
                raise ValueError('cannot determine the heading: the tag orientations contradict a '
                                 'level registration (tag seen upside down?); check the tag ids '
                                 'and the tag map')
        theta = np.arctan2(s, c)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    else:
        U, S, Vt = np.linalg.svd(H)
        if S[1] < 1e-3 * max(S[0], 1e-12):
            if not oriented:
                raise ValueError('points are (nearly) collinear; use at least 3 tags not in a '
                                 f'line{hint}')
            raise ValueError('rotation is ill-determined (tags nearly collinear and their '
                             'orientations weigh too little or contradict each other); add a tag '
                             'off the line or check the tag ids')
        D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
    T = make_transform(R, cq - R @ cp)
    errors = np.linalg.norm((P @ R.T + T[:3, 3]) - Q, axis=1)
    rot_errors = np.array([np.degrees(rotation_angle((R @ a).T @ b)) for a, b in zip(Ra, Rb)],
                          dtype=float)
    return Registration(T, errors, f'{"tags" if oriented else "points"}_{mode}', rot_errors)


def average_tag_observations(observations):
    """Merge repeated measurements of the same tag (several sightings, or touch + sighting).

    observations: iterable of (tag_id, xyz in vr_world, R_vr_tag 3x3 or None).
    Returns {tag_id: (mean xyz, R or None)}. Positions are averaged over every observation,
    orientations (chordal mean: closest rotation to the summed matrices) only over those that
    have one, so a position-only controller touch still counts towards the tag's centre.
    """
    positions, rotations = {}, {}
    for tag_id, p, R in observations:
        positions.setdefault(int(tag_id), []).append(np.asarray(p, dtype=float).reshape(3))
        if R is not None:
            rotations.setdefault(int(tag_id), []).append(np.asarray(R, dtype=float)[:3, :3])
    return {i: (np.mean(ps, axis=0),
                project_to_rotation(np.sum(rotations[i], axis=0)) if i in rotations else None)
            for i, ps in positions.items()}


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
