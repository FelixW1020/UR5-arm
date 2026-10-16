"""Conversions between ROS geometry messages and 4x4 numpy matrices."""
from geometry_msgs.msg import Pose, Transform

from ur5_common.transforms import make_transform, matrix_to_quat, quat_to_matrix


def transform_to_matrix(tf: Transform):
    q, t = tf.rotation, tf.translation
    return make_transform(quat_to_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])


def pose_to_matrix(pose: Pose):
    q, p = pose.orientation, pose.position
    return make_transform(quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def matrix_to_transform(T) -> Transform:
    tf = Transform()
    tf.translation.x, tf.translation.y, tf.translation.z = (float(v) for v in T[:3, 3])
    q = matrix_to_quat(T)
    tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w = (float(v) for v in q)
    return tf


def matrix_to_pose(T) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    q = matrix_to_quat(T)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(v) for v in q)
    return pose
