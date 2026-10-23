"""ROS 2 node: does the arm move in the direction of the controller?

At rate_hz it looks up base_frame <- reference_frame and base_frame <- tool_frame and feeds them
to vr_teleop.alignment.AlignmentTracker. Each time both have moved min_motion_m it publishes the
angle between the two motions; each time the reference has turned min_rotation_deg, the angle
between the two rotations. RViz arrows at the tool show both motion directions, with a summary.

Two uses:
 (a) Simulation / teleop. Run vr_sim (fake Quest + kinematic UR5) with vr_teleop and set
     reference_frame:=quest_right_controller_truth. The arm follows the controller as seen
     through the registration; the truth frame is where the hand really is. The angle is how far
     the arm's motion is off the hand's real motion, i.e. the registration's rotation error as the
     user would see it while playing.
 (b) Real robot, no game. Hold or clamp the right controller to the tool, freedrive the arm and
     use reference_frame:=quest_right_controller with engaged_topic:='' (always measure). Both
     frames are the same rigid body, so the angle is the registration's rotation error. Translate
     in straight strokes without turning the wrist: with a clamp offset, turning the wrist swings
     the controller on an arc the tool flange doesn't follow.

With engaged_topic set (default /vr_teleop/engaged) it measures only while teleop is engaged and
starts a new stroke at each grip, so hand motion with the clutch open is ignored.

Publishes  ~/direction_error_deg, ~/rotation_error_deg  std_msgs/Float64, one per measurement
           ~/markers  MarkerArray (base_frame): reference / tool motion arrows at the tool + text
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from ur5_common.ros_conversions import transform_to_matrix
from vr_teleop import markers
from vr_teleop.alignment import AlignmentTracker

ARROW_LENGTH_M = 0.15


class AlignmentMonitorNode(Node):

    def __init__(self):
        super().__init__('vr_alignment_monitor')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('reference_frame', 'quest_right_controller')
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('min_motion_m', 0.03)
        self.declare_parameter('min_rotation_deg', 10.0)
        self.declare_parameter('engaged_topic', '/vr_teleop/engaged')   # '' = always measure
        self.declare_parameter('summary_every', 10)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._base, self._ref, self._tool = g('base_frame'), g('reference_frame'), g('tool_frame')
        self._tracker = AlignmentTracker(g('min_motion_m'), g('min_rotation_deg'))
        self._engaged = not g('engaged_topic')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._dir_pub = self.create_publisher(Float64, '~/direction_error_deg', 10)
        self._rot_pub = self.create_publisher(Float64, '~/rotation_error_deg', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        if g('engaged_topic'):
            # Transient local to get vr_teleop's latched state even if it started first.
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, g('engaged_topic'), self._on_engaged, latched)
        self.create_timer(1.0 / g('rate_hz'), self._step)
        self.get_logger().info(
            f'Comparing the motion of {self._tool} with {self._ref} in {self._base} '
            + (f'while {g("engaged_topic")} is true.' if g('engaged_topic') else '(always).'))

    def _on_engaged(self, msg: Bool):
        if msg.data and not self._engaged:
            self._tracker.reset_anchor()
        self._engaged = msg.data

    def _T(self, frame):
        return transform_to_matrix(self._tf_buffer.lookup_transform(self._base, frame,
                                                                    Time()).transform)

    def _step(self):
        if not self._engaged:
            return
        try:
            T_ref, T_tool = self._T(self._ref), self._T(self._tool)
        except TransformException as e:
            self.get_logger().warn(f'Waiting for TF {self._base} <- {self._ref} / {self._tool} '
                                   f'({e}). For the controller this needs the registration '
                                   'base_link -> vr_world.', throttle_duration_sec=5.0)
            return
        m = self._tracker.update(T_ref, T_tool)
        if m is None:
            return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._base)
        p = T_tool[:3, 3]
        out = MarkerArray()
        if m.direction_error_deg is not None:
            self._dir_pub.publish(Float64(data=m.direction_error_deg))
            for i, (d, color) in enumerate(((m.reference_motion, markers.GREEN),
                                            (m.tool_motion, markers.ORANGE))):
                end = p + ARROW_LENGTH_M * d / np.linalg.norm(d)
                out.markers.append(markers.arrow(header, 'motion', i, p, end, color))
        if m.rotation_error_deg is not None:
            self._rot_pub.publish(Float64(data=m.rotation_error_deg))
        summary = self._summary()
        out.markers.append(markers.text(header, 'summary', 0, p + [0.0, 0.0, 0.2],
                                        summary.replace('; ', '\n')))
        self._marker_pub.publish(out)

        every = self.get_parameter('summary_every').value
        if every > 0 and any(v is not None and s.count % every == 0 for v, s in (
                (m.direction_error_deg, self._tracker.direction),
                (m.rotation_error_deg, self._tracker.rotation))):
            self.get_logger().info(f'{self._tool} vs {self._ref}: {summary}')

    def _summary(self):
        stats = (('direction', self._tracker.direction), ('rotation', self._tracker.rotation))
        return '; '.join(f'{name} error {s}' for name, s in stats if s.count)


def main():
    rclpy.init()
    node = AlignmentMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
