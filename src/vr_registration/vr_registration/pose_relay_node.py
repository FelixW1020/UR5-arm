"""ROS 2 node: re-express robot-side poses in vr_world for the game.

Publishes
  /vr/robot_base_pose       PoseStamped of base_link in vr_world (place the robot model here
                            and animate it from /joint_states)
  /vr<input topic>          for each PoseStamped topic in `topics`, e.g.
                            /object_localization/target_pose -> /vr/object_localization/target_pose
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from ur5_common.ros_conversions import matrix_to_pose, pose_to_matrix, transform_to_matrix


class PoseRelayNode(Node):

    def __init__(self):
        super().__init__('vr_pose_relay')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('vr_frame', 'vr_world')
        self.declare_parameter('topics', ['/object_localization/target_pose'])
        self.declare_parameter('robot_pose_rate_hz', 2.0)

        self._base = self.get_parameter('base_frame').value
        self._vr = self.get_parameter('vr_frame').value
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._robot_pub = self.create_publisher(PoseStamped, '/vr/robot_base_pose', latched)
        self._relays = []
        for topic in self.get_parameter('topics').value:
            pub = self.create_publisher(PoseStamped, '/vr' + topic, 10)
            self.create_subscription(PoseStamped, topic,
                                     lambda msg, p=pub: self._relay(msg, p), 10)
            self._relays.append(pub)
        self.create_timer(1.0 / self.get_parameter('robot_pose_rate_hz').value, self._robot_pose)

    def _T_vr(self, frame):
        tf = self._tf_buffer.lookup_transform(self._vr, frame, Time())
        return transform_to_matrix(tf.transform)

    def _robot_pose(self):
        try:
            T = self._T_vr(self._base)
        except TransformException:
            self.get_logger().info('Waiting for vr_world registration...', throttle_duration_sec=10.0)
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._vr
        msg.pose = matrix_to_pose(T)
        self._robot_pub.publish(msg)

    def _relay(self, msg: PoseStamped, pub):
        try:
            T = self._T_vr(msg.header.frame_id)
        except TransformException:
            return
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._vr
        out.pose = matrix_to_pose(T @ pose_to_matrix(msg.pose))
        pub.publish(out)


def main():
    rclpy.init()
    node = PoseRelayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise       # a Ctrl-C landing inside a message callback surfaces as another error
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
