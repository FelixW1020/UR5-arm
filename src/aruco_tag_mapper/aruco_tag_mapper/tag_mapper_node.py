"""ROS 2 node: locate fixed ArUco tags in the robot base frame with every calibrated camera.

For each camera in `cameras`, subscribes to /camera/<name>/color/{image_raw,camera_info},
detects tags, and transforms them into base_frame via TF (so the hand-eye calibration
of each camera must already be published by handeye_publisher).

Publishes
  TF base_frame -> tag_<id>          current fused estimate (for RViz and other nodes)
  ~/markers   MarkerArray            tag squares + labels
Services
  ~/save      std_srvs/Trigger       write the tag map to output_file
  ~/reset     std_srvs/Trigger       forget all observations

When a tag is seen by both cameras, the per-camera estimates should agree to a few mm;
`~/save` reports the disagreement, which is a direct check of both hand-eye calibrations.
"""
import threading

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from aruco_tag_mapper.markers import ArucoMarkerDetector
from aruco_tag_mapper.tag_map import TagMap, save_tag_map
from ur5_common.ros_conversions import matrix_to_pose, matrix_to_transform, transform_to_matrix


class TagMapperNode(Node):

    def __init__(self):
        super().__init__('aruco_tag_mapper')
        self.declare_parameter('cameras', ['wrist_cam', 'table_cam'])
        self.declare_parameter('image_topic_template', '/camera/{camera}/color/image_raw')
        self.declare_parameter('camera_info_topic_template', '/camera/{camera}/color/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_length', 0.10)
        self.declare_parameter('tag_ids', '')           # comma-separated; '' = any id
        self.declare_parameter('max_reprojection_error_px', 1.5)
        self.declare_parameter('max_distance_m', 2.0)
        self.declare_parameter('min_observations', 20)
        self.declare_parameter('output_file', '~/.ros/handeye/tag_map.yaml')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        ids = [int(i) for i in g('tag_ids').split(',') if i.strip()]
        self._detector = ArucoMarkerDetector(g('dictionary'), g('marker_length'), ids or None)
        self._base = g('base_frame')
        self._map = TagMap()
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._intrinsics = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_pub = TransformBroadcaster(self)
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)

        for cam in g('cameras'):
            group = MutuallyExclusiveCallbackGroup()   # one image at a time per camera
            self.create_subscription(
                CameraInfo, g('camera_info_topic_template').format(camera=cam),
                lambda msg, c=cam: self._on_info(c, msg), 10, callback_group=group)
            self.create_subscription(
                Image, g('image_topic_template').format(camera=cam),
                lambda msg, c=cam: self._on_image(c, msg), qos_profile_sensor_data,
                callback_group=group)
        self.create_service(Trigger, '~/save', self._save)
        self.create_service(Trigger, '~/reset', self._reset)
        self.create_timer(0.5, self._publish_estimates)
        self.get_logger().info(f'Mapping {g("dictionary")} tags from cameras {g("cameras")}')

    def _on_info(self, cam, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
        self._intrinsics[cam] = (K, D)

    def _on_image(self, cam, msg: Image):
        if cam not in self._intrinsics:
            return
        K, D = self._intrinsics[cam]
        gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        obs = [o for o in self._detector.detect(gray, K, D)
               if o.reprojection_error_px <= self.get_parameter('max_reprojection_error_px').value
               and np.linalg.norm(o.T_cam_marker[:3, 3]) <= self.get_parameter('max_distance_m').value]
        if not obs:
            return
        try:
            tf = self._tf_buffer.lookup_transform(self._base, msg.header.frame_id,
                                                  Time.from_msg(msg.header.stamp),
                                                  timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'{cam}: no TF {self._base} <- {msg.header.frame_id} ({e}). '
                                   'Is handeye_publisher running for this camera?',
                                   throttle_duration_sec=5.0)
            return
        T_base_cam = transform_to_matrix(tf.transform)
        with self._lock:
            for o in obs:
                self._map.add(o.id, T_base_cam @ o.T_cam_marker, cam)

    def _estimates(self):
        with self._lock:
            return self._map.estimate(self.get_parameter('min_observations').value)

    def _publish_estimates(self):
        est = self._estimates()
        if not est:
            return
        now = self.get_clock().now().to_msg()
        length = self.get_parameter('marker_length').value
        markers = MarkerArray()
        for tag_id, e in est.items():
            t = TransformStamped()
            t.header.stamp, t.header.frame_id = now, self._base
            t.child_frame_id = f'tag_{tag_id}'
            t.transform = matrix_to_transform(e.T_base_tag)
            self._tf_pub.sendTransform(t)

            square = Marker()
            square.header = t.header
            square.ns, square.id, square.type = 'tags', tag_id, Marker.CUBE
            square.pose = matrix_to_pose(e.T_base_tag)
            square.scale.x = square.scale.y = length
            square.scale.z = 0.002
            square.color.r, square.color.g, square.color.b, square.color.a = 1.0, 0.6, 0.0, 0.8
            label = Marker()
            label.header = t.header
            label.ns, label.id, label.type = 'labels', tag_id, Marker.TEXT_VIEW_FACING
            label.pose = matrix_to_pose(e.T_base_tag)
            label.pose.position.z += 0.05
            label.scale.z = 0.03
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'tag {tag_id} ({e.num_observations})'
            markers.markers += [square, label]
        self._marker_pub.publish(markers)

    def _save(self, request, response):
        est = self._estimates()
        if not est:
            response.success = False
            response.message = 'No tag has enough observations yet.'
            return response
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        path = save_tag_map(g('output_file'), est, self._base, g('marker_length'), g('dictionary'))
        lines = [f'tag {i}: n={e.num_observations} spread={e.position_std_m * 1000:.1f}mm '
                 f'cameras={sorted(e.per_source_position)}'
                 + (f' camera disagreement={e.source_disagreement_m * 1000:.1f}mm'
                    if len(e.per_source_position) > 1 else '')
                 for i, e in sorted(est.items())]
        response.success = True
        response.message = f'Saved {len(est)} tags to {path}\n' + '\n'.join(lines)
        self.get_logger().info(response.message)
        return response

    def _reset(self, request, response):
        with self._lock:
            self._map.clear()
        response.success, response.message = True, 'Observations cleared.'
        return response


def main():
    rclpy.init()
    node = TagMapperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
