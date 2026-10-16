"""Hand-eye calibration math (pure numpy/OpenCV, no ROS).

Inputs per sample i:
  T_base_gripper[i]  pose of the robot flange (tool0) in the robot base frame (from TF)
  T_cam_target[i]    pose of the calibration board in the camera optical frame

eye_in_hand  (camera on the wrist, board fixed in the cell): solves X = T_gripper_cam
eye_to_hand  (camera fixed in the cell, board on the wrist): solves X = T_base_cam
"""
from dataclasses import dataclass

import cv2
import numpy as np

from ur5_common.transforms import (invert, make_transform, project_to_rotation,
                                   rotation_angle)

METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}
MOUNTS = ('eye_in_hand', 'eye_to_hand')
MIN_SAMPLES = 3

if not hasattr(cv2, 'calibrateHandEye'):
    raise ImportError(
        f'OpenCV {cv2.__version__} does not expose cv2.calibrateHandEye '
        '(the OpenCV 5.0 Python bindings dropped it). Install OpenCV 4.x, '
        'e.g. pip install "opencv-python<5".')


@dataclass
class HandEyeResult:
    X: np.ndarray                 # 4x4, see module docstring
    translation_residual_m: float
    rotation_residual_deg: float
    method: str
    num_samples: int


def solve(T_base_gripper, T_cam_target, mount='eye_in_hand', method='park') -> HandEyeResult:
    if mount not in MOUNTS:
        raise ValueError(f'mount must be one of {MOUNTS}, got {mount!r}')
    if method not in METHODS:
        raise ValueError(f'method must be one of {sorted(METHODS)}, got {method!r}')
    if len(T_base_gripper) != len(T_cam_target):
        raise ValueError('robot and target pose lists differ in length')
    if len(T_base_gripper) < MIN_SAMPLES:
        raise ValueError(f'need at least {MIN_SAMPLES} samples, got {len(T_base_gripper)}')

    # Eye-to-hand is the same AX=XB problem with the robot poses inverted:
    # feeding T_gripper_base makes OpenCV's "cam2gripper" output come out as T_base_cam.
    robot = list(T_base_gripper) if mount == 'eye_in_hand' else [invert(T) for T in T_base_gripper]
    R, t = cv2.calibrateHandEye(
        [T[:3, :3] for T in robot], [T[:3, 3].reshape(3, 1) for T in robot],
        [T[:3, :3] for T in T_cam_target], [T[:3, 3].reshape(3, 1) for T in T_cam_target],
        method=METHODS[method])
    X = make_transform(R, t)
    t_res, r_res = residuals(T_base_gripper, T_cam_target, X, mount)
    return HandEyeResult(X, t_res, r_res, method, len(T_base_gripper))


def chained_target_poses(T_base_gripper, T_cam_target, X, mount):
    """Pose of the board in the frame where it is physically fixed, one per sample.

    With a perfect calibration these are all identical.
    eye_in_hand: board fixed in base   -> T_base_target = T_base_gripper @ X @ T_cam_target
    eye_to_hand: board fixed on gripper -> T_gripper_target = inv(T_base_gripper) @ X @ T_cam_target
    """
    if mount == 'eye_in_hand':
        return [Tbg @ X @ Tct for Tbg, Tct in zip(T_base_gripper, T_cam_target)]
    return [invert(Tbg) @ X @ Tct for Tbg, Tct in zip(T_base_gripper, T_cam_target)]


def residuals(T_base_gripper, T_cam_target, X, mount):
    """(mean translation deviation [m], mean rotation deviation [deg]) of the chained board pose."""
    chain = chained_target_poses(T_base_gripper, T_cam_target, X, mount)
    ts = np.array([T[:3, 3] for T in chain])
    t_res = float(np.mean(np.linalg.norm(ts - ts.mean(axis=0), axis=1)))
    R_mean = project_to_rotation(sum(T[:3, :3] for T in chain))
    r_res = float(np.degrees(np.mean([rotation_angle(R_mean.T @ T[:3, :3]) for T in chain])))
    return t_res, r_res


def rotation_spread_deg(T_base_gripper):
    """Largest relative rotation between any two gripper poses.

    AX=XB is ill-conditioned when the wrist barely rotates between samples;
    aim for > 30 deg spread about at least two different axes.
    """
    Rs = [T[:3, :3] for T in T_base_gripper]
    best = 0.0
    for i in range(len(Rs)):
        for j in range(i + 1, len(Rs)):
            best = max(best, rotation_angle(Rs[i].T @ Rs[j]))
    return float(np.degrees(best))
