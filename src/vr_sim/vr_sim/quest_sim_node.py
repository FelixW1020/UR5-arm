"""ROS 2 node: a fake Meta Quest 3, so registration and teleop run in RViz without a headset.

  ros2 run vr_sim quest_sim

It publishes exactly what the Quest app sends through ROS-TCP-Endpoint, built from a known
ground truth T_base_vr (vr_in_base_*), so every downstream estimate can be scored:

  /quest/state         vr_interfaces/QuestState: head + controllers in vr_world, with buttons
  /quest/passthrough   vr_interfaces/PassthroughFrame: the tags rendered from the head camera,
                       JPEG, with the camera pose in vr_world. Like real tracking, the REPORTED
                       camera pose carries noise (camera_noise_*); the image is rendered from
                       the true one.
  TF base -> vr_world_truth (static), vr_world_truth -> quest_{head,left_controller,
     right_controller}_truth: the truth, next to vr_registration's estimated vr_world.
  ~/registration_error_deg, ~/registration_error_m   std_msgs/Float64: estimated base -> vr_world
     (TF) against the truth; the position error is evaluated at the tag centroid.
  ~/markers            MarkerArray: true tags in base_frame + the registration error as text.

At start-up the true tags are written as an aruco_tag_mapper tag map (tag_map_file), so
vr_registration reads the same "robot side" it would get from the cameras.

Interactive markers (namespace quest_sim; RViz "InteractiveMarkers" display, topic
/quest_sim/update): drag the head and both controllers in base_frame; right-click a
controller for checkable Grip / Index trigger / Primary button entries. With
controller_mode 'scripted' the right controller instead loops a square in the TRUE vr_world
horizontal plane with the grip held, from script_start_s on: that is the "move forward, the
arm moves forward" test for vr_teleop and vr_alignment_monitor.

auto_register_after_s > 0 calls register_service (vr_tag_detector's ~/register) once.
"""
import array
import threading

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformBroadcaster,
                     TransformException, TransformListener)
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker, MarkerArray)

from aruco_tag_mapper.tag_map import TagEstimate, save_tag_map
from ur5_common.ros_conversions import (matrix_to_pose, matrix_to_transform, pose_to_matrix,
                                        transform_to_matrix)
from ur5_common.transforms import T_LINK_OPTICAL, invert
from vr_interfaces.msg import ControllerState, PassthroughFrame, QuestState
from vr_interfaces.srv import SolveRegistration
from vr_sim import scene
from vr_sim.render import render_tags

DEVICES = ('head', 'left_controller', 'right_controller')
BUTTONS = (('grip', 'Grip (drive the arm)'), ('trigger', 'Index trigger'),
           ('primary', 'Primary button (A/X)'))
# Control frame whose x axis is the marker's x, y or z (interactive_markers tutorial values).
_S = 0.5 ** 0.5
AXIS_CONTROLS = (('x', (_S, _S, 0.0, 0.0)), ('y', (_S, 0.0, 0.0, _S)), ('z', (_S, 0.0, _S, 0.0)))
PIXEL_NOISE = 2.0                   # grey levels, on top of JPEG artefacts
SCRIPTED_MARKER_PERIOD = 3          # move the scripted controller's marker every 3rd state tick


class QuestSimNode(Node):

    def __init__(self):
        super().__init__('quest_sim')
        p = self.declare_parameter
        p('base_frame', 'base_link')
        p('vr_frame', 'vr_world')
        p('truth_frame', 'vr_world_truth')
        p('vr_in_base_xyz', [1.2, -0.5, -0.76])
        p('vr_in_base_rpy_deg', [0.0, 0.0, 135.0])
        p('tag_ids', [0, 1, 2])
        p('tag_xyz', [0.45, 0.30, 0.0, 0.60, -0.15, 0.0, 0.35, -0.35, 0.0])
        p('tag_yaw_deg', [0.0, 30.0, -45.0])
        p('marker_length', 0.10)
        p('dictionary', 'DICT_4X4_50')
        p('tag_map_file', '~/.ros/vr_sim/tag_map.yaml')
        p('state_rate_hz', 30.0)
        p('passthrough_rate_hz', 5.0)
        p('camera_width', 1280)
        p('camera_height', 960)
        p('camera_fx', 800.0)
        p('camera_fy', 800.0)
        p('camera_in_head_xyz', [0.05, 0.03, -0.02])
        p('camera_in_head_rpy_deg', [0.0, 5.0, 0.0])
        p('jpeg_quality', 85)
        p('head_xyz', [0.45, -0.85, 0.65])
        p('head_sway_m', 0.02)
        p('head_sway_deg', 2.0)
        p('camera_noise_m', 0.002)
        p('camera_noise_deg', 0.2)
        p('pose_noise_m', 0.0)
        p('right_controller_xyz', [0.45, -0.45, 0.25])
        p('right_controller_rpy_deg', [0.0, 0.0, 90.0])
        p('left_controller_xyz', [0.30, -0.60, 0.20])
        p('left_controller_rpy_deg', [0.0, 0.0, 90.0])
        p('controller_mode', 'interactive')            # interactive | scripted
        p('script_side_m', 0.20)
        p('script_period_s', 16.0)
        p('script_start_s', 8.0)
        p('auto_register_after_s', 4.0)                # 0 = off
        p('register_service', '/vr_tag_detector/register')
        p('registration_check_rate_hz', 1.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        if g('controller_mode') not in ('interactive', 'scripted'):
            raise ValueError("controller_mode must be 'interactive' or 'scripted'")
        self._base, self._vr, self._truth = g('base_frame'), g('vr_frame'), g('truth_frame')
        self._T_base_vr = scene.pose_from_xyz_rpy_deg(g('vr_in_base_xyz'), g('vr_in_base_rpy_deg'))
        self._T_vr_base = invert(self._T_base_vr)
        self._tags = scene.tags_from_params(g('tag_ids'), g('tag_xyz'), g('tag_yaw_deg'))
        self._centroid = np.mean([T[:3, 3] for T in self._tags.values()], axis=0)
        self._T_head_camera = scene.pose_from_xyz_rpy_deg(g('camera_in_head_xyz'),
                                                          g('camera_in_head_rpy_deg'))
        w, h = int(g('camera_width')), int(g('camera_height'))
        self._K = np.array([[g('camera_fx'), 0.0, (w - 1) / 2.0],
                            [0.0, g('camera_fy'), (h - 1) / 2.0], [0.0, 0.0, 1.0]])
        self._scripted = g('controller_mode') == 'scripted'
        self._rng = np.random.default_rng()

        self._lock = threading.Lock()
        self._rest = {                 # device poses in base_frame, edited by the markers
            'head': scene.look_at(g('head_xyz'), self._centroid),
            'left_controller': scene.pose_from_xyz_rpy_deg(g('left_controller_xyz'),
                                                           g('left_controller_rpy_deg')),
            'right_controller': scene.pose_from_xyz_rpy_deg(g('right_controller_xyz'),
                                                            g('right_controller_rpy_deg')),
        }
        self._buttons = {d: {b: False for b, _ in BUTTONS} for d in DEVICES[1:]}
        self._script_anchor = None
        self._state_ticks = 0
        self._last_error = None
        self._start = self.get_clock().now()

        self._write_tag_map()
        static = TransformStamped()
        static.header.stamp = self._start.to_msg()
        static.header.frame_id, static.child_frame_id = self._base, self._truth
        static.transform = matrix_to_transform(self._T_base_vr)
        self._static_tf = StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(static)
        self._tf_pub = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._state_pub = self.create_publisher(QuestState, '/quest/state', 10)
        self._frame_pub = self.create_publisher(PassthroughFrame, '/quest/passthrough', 2)
        self._err_deg_pub = self.create_publisher(Float64, '~/registration_error_deg', 10)
        self._err_m_pub = self.create_publisher(Float64, '~/registration_error_m', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self._make_interactive_markers()

        self.create_timer(1.0 / g('state_rate_hz'), self._publish_state)
        self.create_timer(1.0 / g('passthrough_rate_hz'), self._publish_passthrough,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / g('registration_check_rate_hz'), self._check_registration)
        self._register_client = None
        if g('auto_register_after_s') > 0.0:
            self._register_client = self.create_client(SolveRegistration, g('register_service'))
            self._register_tries = 0
            self._register_timer = self.create_timer(1.0, self._auto_register)

        self.get_logger().info(
            f'Simulated Quest: vr_world truth at {np.round(self._T_base_vr[:3, 3], 3).tolist()} '
            f'yaw {g("vr_in_base_rpy_deg")[2]:.0f} deg in {self._base}; tags '
            f'{sorted(self._tags)}; controllers {g("controller_mode")}.')

    # ------------------------------------------------------------------ set-up
    def _write_tag_map(self):
        estimates = {i: TagEstimate(T, 1, 0.0, {'vr_sim': T[:3, 3]}) for i, T in self._tags.items()}
        path = save_tag_map(self.get_parameter('tag_map_file').value, estimates, self._base,
                            self.get_parameter('marker_length').value,
                            self.get_parameter('dictionary').value)
        self.get_logger().info(f'Wrote the simulated tag map to {path}.')

    def _make_interactive_markers(self):
        self._im_server = InteractiveMarkerServer(self, 'quest_sim')
        self._menus = {}
        for name in DEVICES:
            im = InteractiveMarker()
            im.header.frame_id = self._base
            im.name = name
            im.description = name.replace('_', ' ')
            im.scale = 0.25 if name == 'head' else 0.15
            im.pose = matrix_to_pose(self._rest[name])
            body = InteractiveMarkerControl()
            body.always_visible = True
            body.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
            body.markers.append(self._body_marker(name))
            im.controls.append(body)
            for axis, (qw, qx, qy, qz) in AXIS_CONTROLS:
                for mode, prefix in ((InteractiveMarkerControl.ROTATE_AXIS, 'rotate_'),
                                     (InteractiveMarkerControl.MOVE_AXIS, 'move_')):
                    c = InteractiveMarkerControl()
                    c.name = prefix + axis
                    c.orientation.w, c.orientation.x = qw, qx
                    c.orientation.y, c.orientation.z = qy, qz
                    c.interaction_mode = mode
                    im.controls.append(c)
            self._im_server.insert(im, feedback_callback=self._on_feedback)
            if name != 'head':
                menu = MenuHandler()
                for key, title in BUTTONS:
                    handle = menu.insert(title, callback=lambda fb, d=name, k=key:
                                         self._on_menu(d, k, fb))
                    menu.setCheckState(handle, MenuHandler.UNCHECKED)
                menu.apply(self._im_server, name)
                self._menus[name] = menu
        self._im_server.applyChanges()

    @staticmethod
    def _body_marker(name):
        m = Marker()
        m.type = Marker.CUBE
        if name == 'head':
            m.scale.x, m.scale.y, m.scale.z = 0.10, 0.19, 0.11
            m.color.r, m.color.g, m.color.b = 0.85, 0.85, 0.9
        else:                                       # long axis = pointing direction (x)
            m.scale.x, m.scale.y, m.scale.z = 0.12, 0.035, 0.035
            m.color.r, m.color.g, m.color.b = ((0.2, 0.4, 1.0) if name == 'left_controller'
                                               else (1.0, 0.3, 0.2))
        m.color.a = 0.8
        m.pose.orientation.w = 1.0
        return m

    # ------------------------------------------------------------------ interaction
    def _on_feedback(self, fb):
        if fb.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        if fb.header.frame_id not in ('', self._base):
            self.get_logger().warn(f'Ignoring marker feedback in {fb.header.frame_id}.',
                                   throttle_duration_sec=5.0)
            return
        with self._lock:
            self._rest[fb.marker_name] = pose_to_matrix(fb.pose)

    def _on_menu(self, device, key, fb):
        menu = self._menus[device]
        checked = menu.getCheckState(fb.menu_entry_id) != MenuHandler.CHECKED
        menu.setCheckState(fb.menu_entry_id,
                           MenuHandler.CHECKED if checked else MenuHandler.UNCHECKED)
        with self._lock:
            self._buttons[device][key] = checked
        menu.reApply(self._im_server)
        self._im_server.applyChanges()
        self.get_logger().info(f'{device}: {key} {"ON" if checked else "off"}')

    # ------------------------------------------------------------------ truth
    def _elapsed(self):
        return (self.get_clock().now() - self._start).nanoseconds * 1e-9

    def _devices(self, t):
        """True device poses in base_frame and the controllers' button states at time t."""
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        scripted = self._scripted and t >= g('script_start_s')
        with self._lock:
            if scripted and self._script_anchor is None:
                self._script_anchor = self._rest['right_controller'].copy()
                self.get_logger().info('Scripted controller: grip on, looping a '
                                       f'{g("script_side_m") * 100:.0f} cm square in vr_world.')
            poses = {k: T.copy() for k, T in self._rest.items()}
            buttons = {d: dict(b) for d, b in self._buttons.items()}
        offset, R = scene.sway(t, g('head_sway_m'), g('head_sway_deg'))
        poses['head'][:3, :3] = poses['head'][:3, :3] @ R
        poses['head'][:3, 3] += offset
        if scripted:
            dx, dy = scene.square_offset(t - g('script_start_s'), g('script_side_m'),
                                         g('script_period_s'))
            poses['right_controller'] = self._script_anchor.copy()
            poses['right_controller'][:3, 3] += self._T_base_vr[:3, :3] @ [dx, dy, 0.0]
            buttons['right_controller']['grip'] = True
        return poses, buttons

    def _noisy(self, T, sigma_m):
        if sigma_m <= 0.0:
            return T
        T = T.copy()
        T[:3, 3] += self._rng.normal(scale=sigma_m, size=3)
        return T

    def _controller(self, T_vr, buttons, sigma_m):
        c = ControllerState()
        c.tracked = True
        c.pose = matrix_to_pose(self._noisy(T_vr, sigma_m))
        c.index_trigger = 1.0 if buttons['trigger'] else 0.0
        c.hand_trigger = 1.0 if buttons['grip'] else 0.0
        c.button_primary = buttons['primary']
        return c

    def _publish_state(self):
        t = self._elapsed()
        poses, buttons = self._devices(t)
        in_vr = {k: self._T_vr_base @ T for k, T in poses.items()}
        now = self.get_clock().now().to_msg()
        sigma = self.get_parameter('pose_noise_m').value

        msg = QuestState()
        msg.header.stamp, msg.header.frame_id = now, self._vr
        msg.head_tracked = True
        msg.head = matrix_to_pose(self._noisy(in_vr['head'], sigma))
        msg.left = self._controller(in_vr['left_controller'], buttons['left_controller'], sigma)
        msg.right = self._controller(in_vr['right_controller'], buttons['right_controller'],
                                     sigma)
        self._state_pub.publish(msg)

        tfs = []
        for name in DEVICES:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id = now, self._truth
            tf.child_frame_id = f'quest_{name}_truth'
            tf.transform = matrix_to_transform(in_vr[name])
            tfs.append(tf)
        self._tf_pub.sendTransform(tfs)

        self._state_ticks += 1
        if self._script_anchor is not None and self._state_ticks % SCRIPTED_MARKER_PERIOD == 0:
            self._im_server.setPose('right_controller', matrix_to_pose(poses['right_controller']))
            self._im_server.applyChanges()

    def _publish_passthrough(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        poses, _ = self._devices(self._elapsed())
        T_base_camera = poses['head'] @ self._T_head_camera              # link convention
        T_optical_base = invert(T_base_camera @ T_LINK_OPTICAL)
        gray = render_tags(self._K, g('camera_width'), g('camera_height'),
                           {i: T_optical_base @ T for i, T in self._tags.items()},
                           g('marker_length'), g('dictionary'), PIXEL_NOISE, self._rng)
        ok, jpeg = cv2.imencode('.jpg', cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                                [cv2.IMWRITE_JPEG_QUALITY, int(g('jpeg_quality'))])
        if not ok:
            self.get_logger().error('JPEG encoding failed.', throttle_duration_sec=5.0)
            return
        reported = (self._T_vr_base @ T_base_camera
                    @ scene.random_pose_noise(self._rng, g('camera_noise_m'),
                                              g('camera_noise_deg')))
        msg = PassthroughFrame()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self._vr
        msg.image.header = msg.header
        msg.image.format = 'jpeg'
        msg.image.data = array.array('B', jpeg.tobytes())
        info = msg.camera_info
        info.header = msg.header
        info.width, info.height = int(g('camera_width')), int(g('camera_height'))
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [float(v) for v in self._K.reshape(-1)]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [float(v) for v in np.hstack([self._K, np.zeros((3, 1))]).reshape(-1)]
        msg.camera_pose = matrix_to_pose(reported)
        self._frame_pub.publish(msg)

    # ------------------------------------------------------------------ scoring
    def _check_registration(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._base, self._vr, Time())
        except TransformException:
            self._publish_markers('registration: none yet')
            return
        deg, m = scene.registration_error(transform_to_matrix(tf.transform), self._T_base_vr,
                                          self._centroid)
        self._err_deg_pub.publish(Float64(data=deg))
        self._err_m_pub.publish(Float64(data=m))
        summary = f'{m * 1000:.1f} mm, {deg:.2f} deg'
        last = self._last_error
        if last is None or abs(m - last[1]) > 0.001 or abs(deg - last[0]) > 0.1:
            self._last_error = (deg, m)
            self.get_logger().info(f'Registration {self._base} -> {self._vr} vs truth: '
                                   f'{summary} (position error at the tag centroid).')
        self._publish_markers('registration error: ' + summary)

    def _publish_markers(self, text):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        length = self.get_parameter('marker_length').value
        for i, T in sorted(self._tags.items()):
            square = Marker()
            square.header.stamp, square.header.frame_id = stamp, self._base
            square.ns, square.id, square.type = 'true_tags', i, Marker.CUBE
            square.pose = matrix_to_pose(T)
            square.scale.x = square.scale.y = length
            square.scale.z = 0.002
            square.color.r, square.color.g, square.color.b, square.color.a = 0.1, 0.8, 0.3, 0.6
            label = Marker()
            label.header = square.header
            label.ns, label.id, label.type = 'true_tag_labels', i, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y = T[0, 3], T[1, 3]
            label.pose.position.z = T[2, 3] + 0.05
            label.pose.orientation.w = 1.0
            label.scale.z = 0.04
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'tag {i} (truth)'
            arr.markers += [square, label]
        info = Marker()
        info.header.stamp, info.header.frame_id = stamp, self._base
        info.ns, info.id, info.type = 'registration_error', 0, Marker.TEXT_VIEW_FACING
        info.pose.position.x, info.pose.position.y = self._centroid[0], self._centroid[1]
        info.pose.position.z = self._centroid[2] + 0.3
        info.pose.orientation.w = 1.0
        info.scale.z = 0.05
        info.color.r, info.color.g, info.color.b, info.color.a = 1.0, 0.9, 0.2, 1.0
        info.text = text
        arr.markers.append(info)
        self._marker_pub.publish(arr)

    def _auto_register(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        if self._elapsed() < g('auto_register_after_s'):
            return
        if not self._register_client.service_is_ready():
            self._register_tries += 1
            if self._register_tries == 1:
                self.get_logger().info(f'Auto-registration: waiting for {g("register_service")}...')
            elif self._register_tries >= 60:
                self.get_logger().warn(f'Auto-registration: {g("register_service")} did not '
                                       'appear within 60 s; giving up (call it yourself).')
                self._register_timer.cancel()
            return
        self._register_timer.cancel()
        self.get_logger().info(f'Auto-registration: calling {g("register_service")}.')
        future = self._register_client.call_async(SolveRegistration.Request(save=False))
        future.add_done_callback(self._on_registered)

    def _on_registered(self, future):
        try:
            r = future.result()
        except Exception as e:  # noqa: BLE001 - any failure of the call is just reported
            self.get_logger().error(f'Auto-registration call failed: {e}')
            return
        log = self.get_logger().info if r.success else self.get_logger().warn
        log(f'Auto-registration {"succeeded" if r.success else "FAILED"}: {r.message}')


def main():
    rclpy.init()
    node = QuestSimNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
