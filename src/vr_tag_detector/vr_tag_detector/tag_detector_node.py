"""ROS 2 node: register the Quest to the robot by looking at ArUco tags through the headset.

The Quest app streams passthrough frames (JPEG + intrinsics + camera pose in vr_world). Each
tag in view is posed in vr_world and fused over the last `window` sightings. ~/register sends
the fused tags to vr_registration, which pairs them with the robot's tag map (aruco_tag_mapper)
and publishes base_link -> vr_world. With orientation one tag is enough; several tags spread
over the table make the heading less sensitive to the tag's own (monocular) orientation.

Subscribes
  /quest/passthrough  vr_interfaces/PassthroughFrame   (frame_topic; frame_id must be vr_frame)
Publishes
  ~/debug_image       sensor_msgs/Image (bgr8)   detections, throttled to debug_image_rate_hz
  ~/markers           MarkerArray                fused tags in vr_frame, 'tag <id> n=.. +-.. mm'
Services
  ~/register          vr_interfaces/SolveRegistration   fused tags -> registration_service
  ~/reset             std_srvs/Trigger                  forget all sightings

~/register refuses if marker_length or dictionary differ from the ones recorded in the robot's
tag map (tag_map_file): the two must describe the same printed tags.
"""
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Point, Quaternion
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from aruco_tag_mapper.markers import ArucoMarkerDetector
from aruco_tag_mapper.tag_map import TagMap
from ur5_common.ros_conversions import matrix_to_pose, pose_to_matrix
from vr_interfaces.msg import PassthroughFrame, TagObservation
from vr_interfaces.srv import RegisterTags, SolveRegistration
from vr_tag_detector import passthrough as pt

SPREAD_HINT = ('A large spread usually means the headset moved while capturing (hold still, '
               'call ~/reset and look again), or the camera pose / intrinsics convention is '
               'wrong: camera_pose must be the link frame (x = viewing direction, y = image-left, '
               'z = image-up) at the capture time of THIS image, and K must match its size.')


class TagDetectorNode(Node):

    def __init__(self):
        super().__init__('vr_tag_detector')
        self.declare_parameter('frame_topic', '/quest/passthrough')
        self.declare_parameter('vr_frame', 'vr_world')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_length', 0.10)
        self.declare_parameter('tag_ids', '')           # comma-separated; '' = any id
        self.declare_parameter('max_reprojection_error_px', 2.0)
        self.declare_parameter('max_distance_m', 2.5)
        self.declare_parameter('min_observations', 5)
        self.declare_parameter('window', 150)
        self.declare_parameter('use_orientation', True)
        self.declare_parameter('max_spread_m', 0.03)
        self.declare_parameter('registration_service', '/vr_registration/register_tags')
        self.declare_parameter('registration_timeout_s', 10.0)
        self.declare_parameter('tag_map_file', '~/.ros/handeye/tag_map.yaml')   # '' = no check
        self.declare_parameter('debug_image_rate_hz', 2.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        ids = [int(i) for i in g('tag_ids').split(',') if i.strip()]
        self._detector = ArucoMarkerDetector(g('dictionary'), g('marker_length'), ids or None)
        self._vr = g('vr_frame')
        self._map = TagMap(window=g('window'))
        self._lock = threading.Lock()
        self._last_debug = 0.0

        self._debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        # Separate groups: the image callback never blocks ~/register, and ~/register can wait
        # for the registration client's response, which another executor thread delivers.
        self._client = self.create_client(RegisterTags, g('registration_service'),
                                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(SolveRegistration, '~/register', self._register,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(Trigger, '~/reset', self._reset)
        # Depth 1: while a frame is being processed newer ones replace each other, so a slow
        # detector always picks up the newest frame instead of working through a backlog.
        self.create_subscription(PassthroughFrame, g('frame_topic'), self._on_frame,
                                 QoSProfile(depth=1),
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0, self._publish_markers)
        self.get_logger().info(f'Looking for {g("dictionary")} tags {ids or "(any id)"} '
                               f'on {g("frame_topic")}')
        mismatch = self._tag_map_mismatch()
        if mismatch:
            self.get_logger().warn(mismatch)

    # ------------------------------------------------------------------ frames
    def _on_frame(self, msg: PassthroughFrame):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        log = self.get_logger()
        if msg.header.frame_id != self._vr:
            log.warn(f'Ignoring passthrough frame in {msg.header.frame_id!r}; expected '
                     f'{self._vr!r}.', throttle_duration_sec=5.0)
            return
        info = msg.camera_info
        try:
            gray = pt.decode_gray(msg.image.data)
            K, D = pt.intrinsics(info.k, info.d)
        except ValueError as e:
            log.warn(f'Ignoring passthrough frame: {e}', throttle_duration_sec=5.0)
            return
        if info.width and (info.width, info.height) != (gray.shape[1], gray.shape[0]):
            log.warn(f'Ignoring passthrough frame: camera_info is {info.width}x{info.height} but '
                     f'the image is {gray.shape[1]}x{gray.shape[0]}; K must describe the image '
                     'as sent.', throttle_duration_sec=5.0)
            return
        q = msg.camera_pose.orientation
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) > 0.01:
            log.warn('Ignoring passthrough frame: camera_pose orientation is not a unit '
                     'quaternion.', throttle_duration_sec=5.0)
            return

        sightings = pt.detect_tags_in_vr(gray, K, D, pose_to_matrix(msg.camera_pose),
                                         self._detector, g('max_reprojection_error_px'),
                                         g('max_distance_m'))
        with self._lock:
            pt.add_sightings(self._map, sightings)
        if sightings:
            seen = [f'tag {s.id} at {s.distance_m:.2f} m'
                    + (f' rejected ({s.rejected})' if s.rejected else '') for s in sightings]
            log.info('In view: ' + ', '.join(seen), throttle_duration_sec=5.0)

        rate = g('debug_image_rate_hz')
        now = time.monotonic()
        if (rate > 0.0 and now - self._last_debug >= 1.0 / rate
                and self._debug_pub.get_subscription_count() > 0):
            self._last_debug = now
            bgr = pt.draw_debug(gray, sightings, K, D, g('marker_length') / 2)
            img = Image()
            img.header.stamp = self.get_clock().now().to_msg()
            img.header.frame_id = self._vr
            img.height, img.width = bgr.shape[:2]
            img.encoding, img.step = 'bgr8', bgr.shape[1] * 3
            img.data = bgr.tobytes()
            self._debug_pub.publish(img)

    def _estimates(self, min_observations):
        with self._lock:
            return self._map.estimate(min_observations)

    def _tag_map_mismatch(self):
        """'' or why this detector's tags differ from those in tag_map_file."""
        path = os.path.expanduser(self.get_parameter('tag_map_file').value)
        if not path or not os.path.exists(path):
            return ''                   # vr_registration reports a missing map itself
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            return f'Cannot read the tag map {path}: {e}'
        mismatch = pt.tag_map_mismatch(data, self.get_parameter('marker_length').value,
                                       self.get_parameter('dictionary').value)
        return (f'The tag map {path} was made with other tags: {mismatch}. Set marker_length '
                'and dictionary in BOTH the aruco_tag_mapper and vr_tag_detector sections of '
                'config/vr.yaml (in the simulation: vr_tag_detector in config/vr_sim.yaml), and '
                'in a hand-written tag map (measure the printed black square).') if mismatch else ''

    # ------------------------------------------------------------------ services
    def _register(self, request, response):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        response.success = False
        mismatch = self._tag_map_mismatch()
        if mismatch:
            response.message = f'Not registering. {mismatch}'
            self.get_logger().warn(response.message)
            return response
        est = self._estimates(g('min_observations'))
        if not est:
            seen = {i: e.num_observations for i, e in sorted(self._estimates(1).items())}
            response.message = (f'No tag seen in at least min_observations={g("min_observations")}'
                                f' frames yet (sightings so far: {seen or "none"}). Look at the '
                                'tag(s) with the headset while passthrough frames are streaming.')
            return response

        tag_request = RegisterTags.Request()
        tag_request.save = request.save
        for tag_id, p, q in pt.to_observations(est, g('use_orientation')):
            o = TagObservation(id=tag_id, position=Point(x=float(p[0]), y=float(p[1]),
                                                         z=float(p[2])))
            if q is not None:
                o.has_orientation = True
                o.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]),
                                           w=float(q[3]))
            tag_request.observations.append(o)

        service, timeout = g('registration_service'), g('registration_timeout_s')
        # Polls without spinning, so it is safe inside a callback.
        if not self._client.wait_for_service(timeout_sec=min(2.0, timeout)):
            response.message = f'{service} is not available. Is vr_registration running?'
            return response
        future = self._client.call_async(tag_request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            self._client.remove_pending_request(future)
            response.message = f'No reply from {service} within {timeout:g} s.'
            return response
        try:
            result = future.result()
        except Exception as e:  # noqa: BLE001 - anything the service layer raised
            response.message = f'{service} failed: {e}'
            return response

        lines, noisy = pt.summarize(est, g('max_spread_m'),
                                    dict(zip(result.used_ids, result.errors_m)))
        response.success = result.success
        response.base_to_vr = result.base_to_vr
        response.rms_error_m = result.rms_error_m
        response.max_error_m = result.max_error_m
        text = [result.message] + lines
        if noisy:
            text.append(f'WARNING: sightings of tag(s) {noisy} spread more than '
                        f'{g("max_spread_m") * 1000:.0f} mm. {SPREAD_HINT}')
        response.message = '\n'.join(text)
        if result.success:
            self.get_logger().info(response.message)
        else:   # separate call sites: rclpy refuses to change the severity of one
            self.get_logger().warn(response.message)
        return response

    def _reset(self, request, response):
        with self._lock:
            self._map.clear()
        self._publish_markers()
        response.success, response.message = True, 'Passthrough tag sightings cleared.'
        return response

    # ------------------------------------------------------------------ RViz
    def _publish_markers(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        min_obs, max_spread, length = g('min_observations'), g('max_spread_m'), g('marker_length')
        stamp = self.get_clock().now().to_msg()
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = self._vr
        markers = MarkerArray(markers=[clear])
        for tag_id, e in sorted(self._estimates(1).items()):
            if e.num_observations < min_obs:
                rgb = (0.6, 0.6, 0.6)                   # not enough sightings yet
            elif e.position_std_m > max_spread:
                rgb = (1.0, 0.1, 0.1)
            else:
                rgb = (0.1, 0.8, 1.0)
            pose = matrix_to_pose(e.T_base_tag)
            square = Marker(ns='tags', id=tag_id, type=Marker.CUBE, pose=pose)
            square.scale.x = square.scale.y = length
            square.scale.z = 0.002
            x_axis = Marker(ns='tag_x', id=tag_id, type=Marker.ARROW, pose=pose)
            x_axis.scale.x, x_axis.scale.y, x_axis.scale.z = length * 0.75, 0.006, 0.012
            label = Marker(ns='labels', id=tag_id, type=Marker.TEXT_VIEW_FACING,
                           pose=matrix_to_pose(e.T_base_tag))
            label.pose.position.z += 0.05
            label.scale.z = 0.03
            label.text = (f'tag {tag_id} n={e.num_observations} '
                          f'+-{e.position_std_m * 1000:.1f} mm')
            for m, color, alpha in ((square, rgb, 0.8), (x_axis, (1.0, 0.0, 0.0), 1.0),
                                    (label, (1.0, 1.0, 1.0), 1.0)):
                m.header.stamp, m.header.frame_id = stamp, self._vr
                m.color.r, m.color.g, m.color.b = (float(c) for c in color)
                m.color.a = alpha
                markers.markers.append(m)
        self._marker_pub.publish(markers)


def main():
    rclpy.init()
    node = TagDetectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
