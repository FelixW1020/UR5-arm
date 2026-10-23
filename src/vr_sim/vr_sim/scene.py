"""Geometry of the simulated Quest scene (pure numpy, no ROS): poses, motion, noise, errors.

Everything the quest_sim node computes beyond message plumbing lives here, so the "truth"
it publishes and the error it reports against the registration are unit tested.
"""
import numpy as np

from ur5_common.transforms import invert, make_transform, rotation_angle
from vr_sim.ur_kinematics import rotation_exp


def rpy_to_matrix(roll, pitch, yaw):
    """ROS fixed-axis roll/pitch/yaw (rad): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pose_from_xyz_rpy_deg(xyz, rpy_deg):
    return make_transform(rpy_to_matrix(*np.radians(np.asarray(rpy_deg, dtype=float))), xyz)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Pose at eye in the ROS body convention (x toward target, y left, z up-ish)."""
    eye = np.asarray(eye, dtype=float)
    x = np.asarray(target, dtype=float) - eye
    x /= np.linalg.norm(x)
    y = np.cross(up, x)
    if np.linalg.norm(y) < 1e-6:                    # looking straight up or down
        y = np.cross((1.0, 0.0, 0.0), x)
    y /= np.linalg.norm(y)
    return make_transform(np.column_stack([x, y, np.cross(x, y)]), eye)


def tags_from_params(ids, xyz_flat, yaw_deg):
    """{id: T_base_tag} for tags lying face-up on the table (R_base_tag = Rz(yaw))."""
    ids = [int(i) for i in ids]
    xyz = np.asarray(xyz_flat, dtype=float)
    if len(ids) == 0 or xyz.size != 3 * len(ids) or len(yaw_deg) != len(ids):
        raise ValueError(f'tag_xyz must hold 3 numbers and tag_yaw_deg 1 per tag id '
                         f'({len(ids)} ids, {xyz.size} xyz, {len(yaw_deg)} yaw)')
    if len(set(ids)) != len(ids):
        raise ValueError(f'duplicate tag ids {ids}')
    return {i: pose_from_xyz_rpy_deg(p, (0.0, 0.0, y))
            for i, p, y in zip(ids, xyz.reshape(-1, 3), yaw_deg)}


def sway(t, amplitude_m, amplitude_deg):
    """(offset_xyz, R_local): slow, smooth head motion of a person standing still.

    Incommensurate periods so the pattern doesn't repeat visibly; offset is in the parent frame,
    R_local is applied in the head frame (small yaw and pitch).
    """
    w = 2.0 * np.pi * t
    offset = amplitude_m * np.array([np.sin(w / 7.0), np.sin(w / 9.0 + 1.0),
                                     0.5 * np.sin(w / 11.0 + 2.0)])
    a = np.radians(amplitude_deg)
    return offset, rpy_to_matrix(0.0, 0.5 * a * np.sin(w / 10.0 + 0.5), a * np.sin(w / 8.0))


def square_offset(t, side, period):
    """Planar offset (dx, dy) along a square: +x, +y, -x, -y legs, period/4 s each.

    Each leg eases in and out (zero velocity at the corners), so a velocity-limited arm can
    follow it and the direction of motion within a leg is constant.
    """
    leg = period / 4.0
    k = int(np.floor(t / leg)) % 4
    s = 0.5 - 0.5 * np.cos(np.pi * ((t % leg) / leg))
    corners = np.array([[0.0, 0.0], [side, 0.0], [side, side], [0.0, side]])
    return corners[k] + s * (corners[(k + 1) % 4] - corners[k])


def random_pose_noise(rng, sigma_m, sigma_deg):
    """Small random rigid transform: N(0, sigma_m) per axis, rotation N(0, sigma_deg) about a
    random axis."""
    if sigma_m <= 0.0 and sigma_deg <= 0.0:
        return np.eye(4)
    axis = rng.normal(size=3)
    angle = np.radians(max(sigma_deg, 0.0)) * rng.normal()
    return make_transform(rotation_exp(axis / np.linalg.norm(axis) * angle),
                          rng.normal(scale=max(sigma_m, 0.0), size=3))


def registration_error(T_base_vr_est, T_base_vr_true, point_base):
    """(rotation error deg, position error m at point_base) of an estimated base<-vr transform.

    The position error is where a vr_world point that truly sits at point_base (e.g. the tag
    centroid, where the game draws things) lands when mapped with the estimate.
    """
    p = np.append(np.asarray(point_base, dtype=float), 1.0)
    p_est = T_base_vr_est @ (invert(T_base_vr_true) @ p)
    deg = np.degrees(rotation_angle(T_base_vr_est[:3, :3].T @ T_base_vr_true[:3, :3]))
    return float(deg), float(np.linalg.norm(p_est[:3] - p[:3]))
