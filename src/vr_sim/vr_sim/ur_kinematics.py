"""UR forward / inverse kinematics from Universal Robots' DH parameters (pure numpy, no ROS).

Only for the kinematic sim: the real arm is driven by its own controller. The DH chain gives
tool0 in UR's internal "base" frame (the pendant's, x pointing backwards). ur_description's
base_link is that frame turned by pi about z (REP-103, x forward), and its tool0 coincides
with the DH flange frame, so T_base_link_tool0 = Rz(pi) @ FK_DH(q). ur5_kinematic_sim checks
this against robot_state_publisher's TF at start-up, so a DH/URDF mismatch cannot go unseen.

IK is damped least squares on the geometric Jacobian, warm-started from the current joints.
There is no closed-form branch choice: it follows a streamed target from where the arm is,
which is what teleoperation needs, and degrades gracefully near singularities.
"""
import numpy as np

# d1, a2, a3, d4, d5, d6 [m]: Universal Robots, "DH parameters for calculations of kinematics
# and dynamics" (same numbers as ur_description's config/<type>/default_kinematics.yaml).
DH_PARAMS = {
    'ur3': (0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819),
    'ur5': (0.089159, -0.425, -0.39225, 0.10915, 0.09465, 0.0823),
    'ur10': (0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922),
    'ur3e': (0.15185, -0.24355, -0.2132, 0.13105, 0.08535, 0.0921),
    'ur5e': (0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996),
    'ur10e': (0.1807, -0.6127, -0.57155, 0.17415, 0.11985, 0.11655),
}
ALPHA = (np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0)
JOINT_NAMES = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
# ur_description joint_limits.yaml: +-2pi, elbow +-pi (UR3's endless wrist 3 is clamped too).
JOINT_UPPER = np.array([2 * np.pi, 2 * np.pi, np.pi, 2 * np.pi, 2 * np.pi, 2 * np.pi])
JOINT_LOWER = -JOINT_UPPER
T_BASE_LINK_BASE = np.diag([-1.0, -1.0, 1.0, 1.0])       # Rz(pi)


def dh_table(ur_type='ur5'):
    """(d, a, alpha), each of length 6, for the standard DH chain of ur_type."""
    try:
        d1, a2, a3, d4, d5, d6 = DH_PARAMS[ur_type]
    except KeyError:
        raise ValueError(f'Unknown ur_type {ur_type!r}; known: {sorted(DH_PARAMS)}') from None
    return (np.array([d1, 0.0, 0.0, d4, d5, d6]), np.array([0.0, a2, a3, 0.0, 0.0, 0.0]),
            np.array(ALPHA))


def dh_matrix(theta, d, a, alpha):
    """Standard DH link transform Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def joint_frames(q, ur_type='ur5'):
    """[T_base_link_frame_i for i = 0..6]: frame i-1's z axis is joint i's axis, frame 6 = tool0."""
    d, a, alpha = dh_table(ur_type)
    frames = [T_BASE_LINK_BASE]
    for i in range(6):
        frames.append(frames[-1] @ dh_matrix(q[i], d[i], a[i], alpha[i]))
    return frames


def fk(q, ur_type='ur5'):
    """T_base_link_tool0 at joint angles q (rad, JOINT_NAMES order)."""
    return joint_frames(np.asarray(q, dtype=float), ur_type)[-1]


def _jacobian(frames):
    p = frames[-1][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z, o = frames[i][:3, 2], frames[i][:3, 3]
        J[:3, i] = np.cross(z, p - o)
        J[3:, i] = z
    return J


def jacobian(q, ur_type='ur5'):
    """6x6 geometric Jacobian in base_link: [v; w] of tool0 = J @ dq."""
    return _jacobian(joint_frames(np.asarray(q, dtype=float), ur_type))


def clamp_to_limits(q):
    return np.clip(np.asarray(q, dtype=float), JOINT_LOWER, JOINT_UPPER)


def limit_joint_step(dq, max_step):
    """dq scaled down as a whole so no joint moves more than max_step.

    Scaling every joint by the same factor keeps the step's direction in joint space, so the
    joints arrive together and the tool stays close to the commanded path. Clipping each joint
    on its own would bend that path by centimetres once one joint saturates.
    """
    dq = np.asarray(dq, dtype=float)
    largest = float(np.max(np.abs(dq)))
    return dq * (max_step / largest) if largest > max_step else dq


def rotation_log(R):
    """Rotation vector (axis * angle, angle in [0, pi]) of a rotation matrix.

    The textbook theta / (2 sin theta) * vee(R - R^T) is ill-conditioned near pi, where the
    axis comes from the symmetric part instead; the angle uses atan2, accurate everywhere.
    """
    R = np.asarray(R, dtype=float)[:3, :3]
    v = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(v)                   # sin(theta)
    c = 0.5 * (np.trace(R) - 1.0)           # cos(theta)
    theta = np.arctan2(s, c)
    if c > 0.0:
        return v * (theta / s) if s > 1e-12 else v
    S = 0.5 * (R + R.T) - c * np.eye(3)     # (1 - cos) n n^T
    i = int(np.argmax(np.diag(S)))
    n = S[:, i] / np.sqrt(max(S[i, i] * (1.0 - c), 1e-300))
    if np.dot(n, v) < 0.0:
        n = -n
    return theta * n / np.linalg.norm(n)


def rotation_exp(w):
    """Rotation matrix of a rotation vector (Rodrigues)."""
    w = np.asarray(w, dtype=float).reshape(3)
    theta = np.linalg.norm(w)
    K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    if theta < 1e-8:
        return np.eye(3) + K + 0.5 * K @ K
    return (np.eye(3) + np.sin(theta) / theta * K
            + (1.0 - np.cos(theta)) / theta ** 2 * K @ K)


def solve_ik(q0, T_target, ur_type='ur5', max_iterations=100, position_tolerance=1e-4,
             rotation_tolerance=1e-3, damping=0.02, max_step=0.2):
    """Damped-least-squares IK for T_base_link_tool0 = T_target, starting from q0.

    Each iteration moves at most max_step rad per joint and is clamped to the joint limits.
    Returns (q, position_error_m, rotation_error_rad) of the best iterate; for an unreachable
    target that is the closest pose found, so the caller decides what error is acceptable.
    """
    T_target = np.asarray(T_target, dtype=float)
    q = clamp_to_limits(q0)
    best = None
    for it in range(max_iterations + 1):
        frames = joint_frames(q, ur_type)
        T = frames[-1]
        e = np.concatenate([T_target[:3, 3] - T[:3, 3],
                            rotation_log(T_target[:3, :3] @ T[:3, :3].T)])
        pos_err, rot_err = float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
        if best is None or pos_err + 0.1 * rot_err < best[1] + 0.1 * best[2]:
            best = (q.copy(), pos_err, rot_err)
        if (pos_err < position_tolerance and rot_err < rotation_tolerance) \
                or it == max_iterations:
            break
        J = _jacobian(frames)
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
        largest = np.max(np.abs(dq))
        if largest > max_step:
            dq *= max_step / largest
        q = clamp_to_limits(q + dq)
    return best
