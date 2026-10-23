"""ROS 2 node: Quest controller -> UR5 tool target, with a clutch on the grip.

While the grip (Joy axes[clutch_axis], hand_trigger in quest_bridge's layout) is held, publishes
~/target_pose = /vr_teleop/target_pose (PoseStamped in base_frame): the tool's pose at the moment
of the grip, moved and turned by the controller's motion since then (vr_teleop.mapping).

This node never commands a robot. Feed /vr_teleop/target_pose to MoveIt Servo pose tracking, to
your own UR5 controller, or to vr_sim's ur5_kinematic_sim to try it in RViz first.

Subscribes  joy_topic                 sensor_msgs/Joy (vr_tracking's quest_bridge)
TF lookups  base_frame <- controller_frame  (needs the registration base_link -> vr_world)
            base_frame <- vr_frame          (the registration itself; vr_frame '' skips it)
            base_frame <- tool_frame
Publishes   ~/target_pose  PoseStamped (base_frame), only while engaged
            ~/engaged      std_msgs/Bool (transient local), on every change
            ~/markers      MarkerArray (base_frame): target axes, controller pointing arrow (x axis)

Deadman: letting go of the grip, a Joy message older than joy_timeout_s (bridge or headset gone)
or a controller pose older than that (tracking lost) releases the clutch. So does anything that
would move the target without the hand moving: a new registration (base_frame <- vr_frame
changed) or a controller pose that jumps more than max_step_m / max_step_deg between two steps
(tracking snapping back). Engaging always starts from the current controller and tool poses, so
the arm never jumps to where the hand went while the clutch was open; with the grip still held
it re-engages on the next step. It does not engage while tool0 is outside the workspace box.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, ConnectivityException, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from ur5_common.ros_conversions import matrix_to_pose, transform_to_matrix
from vr_teleop import markers
from vr_teleop.mapping import (REGISTRATION_CHANGE_DEG, REGISTRATION_CHANGE_M, ClutchMapper,
                               pose_change)


class TeleopNode(Node):

    def __init__(self):
        super().__init__('vr_teleop')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('controller_frame', 'quest_right_controller')
        self.declare_parameter('vr_frame', 'vr_world')     # '' = don't watch the registration
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('joy_topic', '/quest/right_controller/joy')
        self.declare_parameter('clutch_axis', 3)            # hand_trigger (grip)
        self.declare_parameter('clutch_threshold', 0.5)
        self.declare_parameter('joy_timeout_s', 0.5)
        self.declare_parameter('max_step_m', 0.1)           # controller jump that releases
        self.declare_parameter('max_step_deg', 45.0)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('scale', 1.0)                # tool motion / hand motion
        self.declare_parameter('follow_orientation', True)
        self.declare_parameter('workspace_min', [-1.0, -1.0, -0.05])   # box in base_frame, m
        self.declare_parameter('workspace_max', [1.0, 1.0, 1.2])

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._base, self._ctrl, self._tool = g('base_frame'), g('controller_frame'), g('tool_frame')
        self._vr = g('vr_frame')
        self._axis, self._threshold = g('clutch_axis'), g('clutch_threshold')
        self._timeout = Duration(seconds=g('joy_timeout_s'))
        self._max_step_m, self._max_step_deg = g('max_step_m'), g('max_step_deg')
        self._mapper = ClutchMapper(g('scale'), g('follow_orientation'),
                                    g('workspace_min'), g('workspace_max'))
        self._marker_lifetime = Duration(seconds=max(0.5, 3.0 / g('rate_hz'))).to_msg()
        self._joy = None
        self._joy_time = None
        self._T_reg0 = None         # base_frame <- vr_frame at engage
        self._T_ctrl_prev = None    # controller pose at the previous engaged step

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._target_pub = self.create_publisher(PoseStamped, '~/target_pose', 10)
        self._engaged_pub = self.create_publisher(Bool, '~/engaged', latched)
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self.create_subscription(Joy, g('joy_topic'), self._on_joy, qos_profile_sensor_data)
        self._engaged_pub.publish(Bool(data=False))
        self.create_timer(1.0 / g('rate_hz'), self._step)
        self.get_logger().info(
            f'Teleop {self._ctrl} -> {self._tool} target in {self._base}: hold the grip '
            f'({g("joy_topic")} axes[{self._axis}] > {self._threshold}) to move. '
            'Publishes ~/target_pose only; it does not command the robot.')

    def _on_joy(self, msg: Joy):
        self._joy = msg
        self._joy_time = self.get_clock().now()

    def _clutch(self, now):
        """(pressed, reason if not)."""
        if self._joy is None:
            return False, 'no Joy yet'
        if now - self._joy_time > self._timeout:
            return False, 'Joy timed out'
        if len(self._joy.axes) <= self._axis:
            self.get_logger().warn(f'Joy has {len(self._joy.axes)} axes, clutch_axis is '
                                   f'{self._axis}.', throttle_duration_sec=5.0)
            return False, 'no clutch axis'
        return self._joy.axes[self._axis] > self._threshold, 'grip released'

    def _lookup(self, frame, now, warn_stale=True):
        """T_base_frame, or None (with a throttled explanation) if missing or stale."""
        try:
            tf = self._tf_buffer.lookup_transform(self._base, frame, Time())
        except ConnectivityException as e:
            hint = ('the registration base_link -> vr_world is missing: register the headset '
                    '(vr_tag_detector ~/register or vr_registration) or check that '
                    'vr_registration loaded a saved one' if frame in (self._ctrl, self._vr) else
                    'the robot TF tree is not connected to it')
            self.get_logger().warn(f'{self._base} and {frame} are not connected in TF; {hint}. '
                                   f'({e})', throttle_duration_sec=5.0)
            return None
        except TransformException as e:
            hint = ('is quest_bridge running and the controller tracked?' if frame == self._ctrl
                    else 'is the headset registered (vr_registration)?' if frame == self._vr
                    else 'is the UR driver / robot_state_publisher (or ur5_kinematic_sim) up?')
            self.get_logger().warn(f'No TF {self._base} <- {frame}: {hint} ({e})',
                                   throttle_duration_sec=5.0)
            return None
        # Untracked controllers get no TF from quest_bridge, so the latest one goes stale.
        # Stamp 0 = an all-static chain (e.g. a static_transform_publisher test), never stale.
        stamp = Time.from_msg(tf.header.stamp, clock_type=now.clock_type)
        age = now - stamp
        if frame == self._ctrl and stamp.nanoseconds > 0 and age > self._timeout:
            if warn_stale:
                self.get_logger().warn(f'{frame} pose is {age.nanoseconds * 1e-9:.1f} s old '
                                       '(controller not tracked?).', throttle_duration_sec=5.0)
            return None
        return transform_to_matrix(tf.transform)

    def _set_engaged(self, engaged, reason=''):
        self._engaged_pub.publish(Bool(data=engaged))
        if engaged:
            self.get_logger().info('Clutch engaged: target follows the controller.')
        else:
            self._mapper.release()
            self.get_logger().info(f'Clutch released ({reason}).')

    def _engage(self, T_ctrl, now):
        """Anchor the clutch at the current controller, tool and registration (if all known)."""
        T_tool = self._lookup(self._tool, now)
        if T_tool is None:
            return
        p = T_tool[:3, 3]
        if not self._mapper.in_workspace(p):
            # Engaging would clamp the target into the box and pull the arm to its edge.
            self.get_logger().warn(
                f'Not engaging: {self._tool} at {np.round(p, 3).tolist()} is outside the '
                f'workspace box {self._mapper.workspace_min.tolist()} .. '
                f'{self._mapper.workspace_max.tolist()} ({self._base}). Move the arm into it or '
                'widen workspace_min / workspace_max.', throttle_duration_sec=5.0)
            return
        T_reg = self._lookup(self._vr, now) if self._vr else None
        if self._vr and T_reg is None:
            return
        self._mapper.engage(T_ctrl, T_tool)
        self._T_reg0, self._T_ctrl_prev = T_reg, T_ctrl
        self._set_engaged(True)

    def _jump(self, T_ctrl, now):
        """Why the target would move without the hand moving ('' if it would not)."""
        if self._vr:
            T_reg = self._lookup(self._vr, now)
            if T_reg is None:
                return 'registration lost'
            d_m, d_deg = pose_change(self._T_reg0, T_reg)
            if d_m > REGISTRATION_CHANGE_M or d_deg > REGISTRATION_CHANGE_DEG:
                return f'registration changed by {d_m * 1000:.0f} mm / {d_deg:.2f} deg'
        d_m, d_deg = pose_change(self._T_ctrl_prev, T_ctrl)
        if d_m > self._max_step_m or d_deg > self._max_step_deg:
            return f'controller pose jumped {d_m * 100:.0f} cm / {d_deg:.0f} deg in one step'
        return ''

    def _step(self):
        now = self.get_clock().now()
        pressed, reason = self._clutch(now)
        T_ctrl = self._lookup(self._ctrl, now, warn_stale=pressed)
        header = Header(stamp=now.to_msg(), frame_id=self._base)
        out = MarkerArray()
        if T_ctrl is not None:
            p = T_ctrl[:3, 3]
            out.markers.append(markers.arrow(header, 'controller', 0, p, p + 0.15 * T_ctrl[:3, 0],
                                             markers.YELLOW))
        if not pressed or T_ctrl is None:
            if self._mapper.engaged:
                self._set_engaged(False, reason if not pressed else 'controller pose lost')
        elif self._mapper.engaged and (jump := self._jump(T_ctrl, now)):
            # Re-engage on the next step (grip still held), from the poses as they are then.
            self._set_engaged(False, jump)
        else:
            if not self._mapper.engaged:
                self._engage(T_ctrl, now)
            if self._mapper.engaged:
                self._T_ctrl_prev = T_ctrl
                T_target = self._mapper.target(T_ctrl)
                self._target_pub.publish(PoseStamped(header=header, pose=matrix_to_pose(T_target)))
                out.markers += markers.axes(header, 'target', T_target)
        for m in out.markers:       # expire, not linger, once the clutch opens / tracking drops
            m.lifetime = self._marker_lifetime
        if out.markers:
            self._marker_pub.publish(out)


def main():
    rclpy.init()
    node = TeleopNode()
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
