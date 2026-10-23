"""ROS 2 node: stream the Quest 3 headset and controllers into ROS, in the robot's frame.

The Quest app publishes vr_interfaces/QuestState on /quest/state (poses in vr_world, headset
clock) and vr_registration publishes base_link -> vr_world. This node joins the two, so the
UR5's own code never deals with the headset clock, Unity axes or the registration itself.

Publishes
  TF vr_world -> quest_head, quest_left_controller, quest_right_controller
                 x = gaze / pointing direction, y left, z up; stamped with ROS time on receipt
  /quest/{head,left_controller,right_controller}/pose   PoseStamped in output_frame (base_link),
                 only once output_frame <- vr_world is known, i.e. after registration
  /quest/{left,right}_controller/joy   sensor_msgs/Joy, always (no registration needed)
                 axes    = [thumbstick_x, thumbstick_y, index_trigger, hand_trigger]
                 buttons = [button_primary, button_secondary, thumbstick_pressed, menu]
  ~/markers      MarkerArray in vr_world: gaze / pointing arrows for RViz

An untracked device gets no TF and no pose, so its last TF ages instead of being overwritten
with a stale pose: check the stamp if that matters.

Using it from your own UR5 node (either way; TF also relates the controller to tool0 etc.):

    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    tf = self.tf_buffer.lookup_transform('base_link', 'quest_right_controller', Time())
    # tf.transform: controller pose in base_link; tf.header.stamp: when it was last tracked

    self.create_subscription(PoseStamped, '/quest/right_controller/pose', self.on_pose, 10)
    self.create_subscription(Joy, '/quest/right_controller/joy', self.on_joy, 10)
    # on_joy: msg.axes[2] index trigger, msg.axes[3] grip, msg.buttons[0] A, msg.buttons[1] B
    # (or index with vr_tracking.quest_state.JOY_AXES / JOY_BUTTONS)
"""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from ur5_common.ros_conversions import (matrix_to_pose, matrix_to_transform, pose_to_matrix,
                                        transform_to_matrix)
from ur5_common.transforms import invert, rotation_angle
from vr_interfaces.msg import QuestState
from vr_tracking.quest_state import (CONTROLLERS, DEVICES, controller_state, device_states,
                                     joy_arrays)

# RViz arrows: (rgb, length m). Left blue / right red, as in most VR tooling.
_STYLE = {'head': ((1.0, 0.8, 0.1), 0.30),
          'left_controller': ((0.2, 0.5, 1.0), 0.15),
          'right_controller': ((1.0, 0.25, 0.2), 0.15)}


class QuestBridgeNode(Node):

    def __init__(self):
        super().__init__('quest_bridge')
        self.declare_parameter('state_topic', '/quest/state')
        self.declare_parameter('topic_prefix', '/quest')
        self.declare_parameter('vr_frame', 'vr_world')
        self.declare_parameter('output_frame', 'base_link')   # must be fixed w.r.t. vr_world
        self.declare_parameter('frame_prefix', 'quest_')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('restamp', True)               # headset clock != ROS clock
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('markers_rate_hz', 10.0)       # <= 0 disables ~/markers

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._state_topic = g('state_topic')
        self._vr, self._out = g('vr_frame'), g('output_frame')
        self._restamp, self._stale_s = g('restamp'), g('stale_timeout_s')
        self._frames = {d: g('frame_prefix') + d for d in DEVICES}
        prefix = g('topic_prefix').rstrip('/')
        self._pose_pubs = {d: self.create_publisher(PoseStamped, f'{prefix}/{d}/pose', 10)
                           for d in DEVICES}
        self._joy_pubs = {d: self.create_publisher(Joy, f'{prefix}/{d}/joy', 10)
                          for d in CONTROLLERS}
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self._tf_pub = TransformBroadcaster(self) if g('publish_tf') else None

        self._T_out_vr = np.eye(4) if self._out == self._vr else None
        self._latest = None         # (QuestState, stamp) for the markers
        self._start = time.monotonic()
        self._last_rx = None        # monotonic, so staleness is independent of use_sim_time
        self._stale = False

        if self._T_out_vr is None:
            # Cached rather than looked up per message: base_link <- vr_world is a static TF,
            # and polling it still picks up a re-registration within a second.
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.create_timer(1.0, self._update_registration)
        self.create_subscription(QuestState, self._state_topic, self._on_state,
                                 qos_profile_sensor_data)
        self.create_timer(min(0.5, max(0.05, self._stale_s / 2.0)), self._check_stale)
        rate = g('markers_rate_hz')
        if rate > 0.0:
            self._marker_lifetime = Duration(seconds=max(0.3, 3.0 / rate)).to_msg()
            self.create_timer(1.0 / rate, self._publish_markers)
        tf_info = f'TF {self._vr} -> {self._frames["head"]}/..., ' if self._tf_pub else ''
        self.get_logger().info(
            f'Bridging {self._state_topic} -> {tf_info}{prefix}/<device>/pose in {self._out}, '
            f'{prefix}/<side>_controller/joy')

    # ------------------------------------------------------------------ registration
    def _update_registration(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._out, self._vr, Time())
        except TransformException:
            if self._T_out_vr is None:
                self.get_logger().info(
                    f'Waiting for TF {self._out} <- {self._vr} (vr_registration); the pose '
                    'topics start once it exists.', throttle_duration_sec=30.0)
            return
        T, old = transform_to_matrix(tf.transform), self._T_out_vr
        self._T_out_vr = T
        if old is not None:
            delta = invert(old) @ T
            if np.linalg.norm(delta[:3, 3]) < 1e-4 and rotation_angle(delta) < 1e-4:
                return
        yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
        self.get_logger().info(
            f'Registration {"available" if old is None else "updated"}: {self._vr} in '
            f'{self._out} at {np.round(T[:3, 3], 3).tolist()} m, yaw {yaw:.1f} deg. '
            f'Publishing poses in {self._out}.')

    # ------------------------------------------------------------------ state
    def _on_state(self, msg: QuestState):
        rx = time.monotonic()
        if self._last_rx is None:
            self.get_logger().info(f'Receiving Quest state on {self._state_topic}.')
        elif self._stale:
            self.get_logger().info(f'Quest state resumed after {rx - self._last_rx:.1f} s.')
        self._last_rx, self._stale = rx, False
        if msg.header.frame_id and msg.header.frame_id != self._vr:
            self.get_logger().warn(f'QuestState frame_id is "{msg.header.frame_id}", expected '
                                   f'"{self._vr}"; treating poses as {self._vr}.',
                                   throttle_duration_sec=30.0)

        stamp = msg.header.stamp
        if self._restamp or (stamp.sec == 0 and stamp.nanosec == 0):
            stamp = self.get_clock().now().to_msg()
        transforms = []
        for device, tracked, pose in device_states(msg):
            if not tracked:
                continue
            T_vr_dev = pose_to_matrix(pose)
            if self._tf_pub is not None:
                t = TransformStamped()
                t.header.stamp, t.header.frame_id = stamp, self._vr
                t.child_frame_id = self._frames[device]
                t.transform = matrix_to_transform(T_vr_dev)
                transforms.append(t)
            if self._T_out_vr is not None:
                out = PoseStamped()
                out.header.stamp, out.header.frame_id = stamp, self._out
                out.pose = matrix_to_pose(self._T_out_vr @ T_vr_dev)
                self._pose_pubs[device].publish(out)
        if transforms:
            self._tf_pub.sendTransform(transforms)

        for device in CONTROLLERS:
            joy = Joy()
            joy.header.stamp, joy.header.frame_id = stamp, self._frames[device]
            joy.axes, joy.buttons = joy_arrays(controller_state(msg, device))
            self._joy_pubs[device].publish(joy)
        self._latest = (msg, stamp)

    def _check_stale(self):
        now = time.monotonic()
        if self._last_rx is None:
            if now - self._start > max(5.0, self._stale_s):
                self.get_logger().warn(
                    f'No QuestState on {self._state_topic} yet. Is the Quest app running and '
                    'connected to ros_tcp_endpoint?', throttle_duration_sec=30.0)
            return
        silent = now - self._last_rx
        if silent > self._stale_s:
            self._stale = True
            self.get_logger().warn(
                f'No QuestState on {self._state_topic} for {silent:.1f} s (headset asleep, app '
                'paused or ros_tcp_endpoint disconnected?).', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------ RViz
    def _publish_markers(self):
        if self._latest is None or self._stale:
            return      # markers expire after their lifetime, so stale devices vanish
        msg, stamp = self._latest
        markers = MarkerArray()
        for i, (device, tracked, pose) in enumerate(device_states(msg)):
            if not tracked:
                continue
            (r, g, b), length = _STYLE[device]
            T = pose_to_matrix(pose)
            arrow = Marker()
            arrow.header.stamp, arrow.header.frame_id = stamp, self._vr
            arrow.ns, arrow.id, arrow.type = 'quest', i, Marker.ARROW
            arrow.pose = matrix_to_pose(T)          # normalised quaternion
            arrow.scale.x, arrow.scale.y, arrow.scale.z = length, 0.012, 0.025
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 1.0
            arrow.lifetime = self._marker_lifetime
            label = Marker()
            label.header = arrow.header
            label.ns, label.id, label.type = 'quest_labels', i, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y = float(T[0, 3]), float(T[1, 3])
            label.pose.position.z = float(T[2, 3]) + 0.06
            label.scale.z = 0.03
            label.color.r, label.color.g, label.color.b, label.color.a = r, g, b, 1.0
            label.text = self._frames[device]
            label.lifetime = self._marker_lifetime
            markers.markers += [arrow, label]
        if markers.markers:
            self._marker_pub.publish(markers)


def main():
    rclpy.init()
    node = QuestBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
