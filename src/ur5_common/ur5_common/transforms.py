"""Pure-numpy rigid transform helpers (no ROS imports, so they are unit-testable anywhere).

Convention: T_a_b is a 4x4 homogeneous matrix that maps points expressed in frame b
into frame a, i.e. it is the pose of frame b in frame a. This matches a TF lookup
``lookup_transform(target_frame=a, source_frame=b)``.
"""
import numpy as np


def quat_to_matrix(x, y, z, w):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    q = np.array([x, y, z, w], dtype=float)
    n = np.dot(q, q)
    if n < 1e-12:
        return np.eye(3)
    q *= np.sqrt(2.0 / n)
    x, y, z, w = q
    return np.array([
        [1.0 - (y * y + z * z), x * y - z * w, x * z + y * w],
        [x * y + z * w, 1.0 - (x * x + z * z), y * z - x * w],
        [x * z - y * w, y * z + x * w, 1.0 - (x * x + y * y)],
    ])


def matrix_to_quat(R):
    """3x3 (or 4x4) rotation -> unit quaternion (x, y, z, w) with w >= 0."""
    R = np.asarray(R, dtype=float)[:3, :3]
    tr = np.trace(R)
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return -q if q[3] < 0.0 else q


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def invert(T):
    R = T[:3, :3]
    t = T[:3, 3]
    return make_transform(R.T, -R.T @ t)


def rotation_angle(R):
    """Angle (rad) of a rotation matrix."""
    c = (np.trace(np.asarray(R)[:3, :3]) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def project_to_rotation(M):
    """Closest rotation matrix (Frobenius norm) to a 3x3 matrix."""
    U, _, Vt = np.linalg.svd(M)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    return U @ D @ Vt
