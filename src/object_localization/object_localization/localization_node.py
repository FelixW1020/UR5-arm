"""ROS 2 node: 2D detections + aligned depth -> 3D object poses in the robot base frame.

Subscribes (time-synchronised)
  detections_topic  vision_msgs/Detection2DArray  (any detector; YOLO by default)
  depth_topic       sensor_msgs/Image             depth aligned to the detector's image
  camera_info_topic sensor_msgs/CameraInfo        intrinsics of that image

Publishes
  ~/detections_3d   vision_msgs/Detection3DArray  in target_frame
  ~/target_pose     geometry_msgs/PoseStamped     best detection of target_class
  ~/markers         visualization_msgs/MarkerArray

Services
  ~/locate          handeye_interfaces/LocateObject   "where is the cup?" -> pose in base frame

The camera -> robot transform comes from TF (published by handeye_publisher), so this
node works unchanged for eye-in-hand and eye-to-hand mounts.
"""
import copy
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from handeye_interfaces.srv import LocateObject
from object_localization.geometry import box_depth, deproject, depth_to_metres
from ur5_common.ros_conversions import transform_to_matrix


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('object_localization')
        self.declare_parameter('detections_topic', '/yolo_detector/detections')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('target_class', 'cup')
        self.declare_parameter('sync_slop_s', 0.1)
        self.declare_parameter('roi_fraction', 0.4)
        self.declare_parameter('depth_percentile', 50.0)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 3.0)
        # Push the point this far along the viewing ray, from the visible surface toward
        # the object centre (e.g. a cup's radius, ~0.04).
        self.declare_parameter('surface_to_centre_m', 0.0)
        self.declare_parameter('max_result_age_s', 1.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._target_frame = g('target_frame')
        self._target_class = g('target_class')

        self._bridge = CvBridge()
        self._K = None
        self._lock = threading.Lock()
        self._latest: dict[str, tuple[PoseStamped, float]] = {}   # class -> (pose, score)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cb = ReentrantCallbackGroup()
        self.create_subscription(CameraInfo, g('camera_info_topic'), self._on_info, 10,
                                 callback_group=cb)
        det_sub = Subscriber(self, Detection2DArray, g('detections_topic'), callback_group=cb)
        depth_sub = Subscriber(self, Image, g('depth_topic'), callback_group=cb)
        self._sync = ApproximateTimeSynchronizer([det_sub, depth_sub], queue_size=10,
                                                 slop=g('sync_slop_s'))
        self._sync.registerCallback(self._on_detections)

        self._det_pub = self.create_publisher(Detection3DArray, '~/detections_3d', 10)
        self._pose_pub = self.create_publisher(PoseStamped, '~/target_pose', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self.create_service(LocateObject, '~/locate', self._locate, callback_group=cb)
        self.get_logger().info(
            f'Localizing detections into {self._target_frame} (target class "{self._target_class}")')

    def _on_info(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    # ------------------------------------------------------------------ main pipeline
    def _on_detections(self, dets: Detection2DArray, depth_msg: Image):
        if self._K is None:
            return
        camera_frame = dets.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame, camera_frame, Time.from_msg(dets.header.stamp),
                timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(
                f'No TF {self._target_frame} <- {camera_frame} ({e}). '
                'Is handeye_publisher running?', throttle_duration_sec=5.0)
            return
        T = transform_to_matrix(tf.transform)

        depth = depth_to_metres(
            self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'),
            depth_msg.encoding)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        out = Detection3DArray()
        out.header.stamp = dets.header.stamp
        out.header.frame_id = self._target_frame
        best: dict[str, tuple[PoseStamped, float]] = {}

        for det in dets.detections:
            if not det.results:
                continue
            hyp = max(det.results, key=lambda r: r.hypothesis.score).hypothesis
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            z = box_depth(depth, cx, cy, det.bbox.size_x, det.bbox.size_y,
                          g('roi_fraction'), g('depth_percentile'),
                          g('min_depth_m'), g('max_depth_m'))
            if z is None:
                continue
            p_cam = deproject(cx, cy, z, self._K)
            p_cam += p_cam / np.linalg.norm(p_cam) * g('surface_to_centre_m')
            p = T[:3, :3] @ p_cam + T[:3, 3]

            d3 = Detection3D()
            d3.header = out.header
            d3.bbox.center.position.x, d3.bbox.center.position.y, d3.bbox.center.position.z = (
                float(v) for v in p)
            d3.bbox.center.orientation.w = 1.0
            # Metric size from the pixel box at the measured depth (depth axis ~ width).
            d3.bbox.size.x = d3.bbox.size.z = float(det.bbox.size_x * z / self._K[0, 0])
            d3.bbox.size.y = float(det.bbox.size_y * z / self._K[1, 1])
            r = ObjectHypothesisWithPose()
            r.hypothesis = hyp
            r.pose.pose = d3.bbox.center
            d3.results.append(r)
            out.detections.append(d3)

            if hyp.class_id not in best or hyp.score > best[hyp.class_id][1]:
                pose = PoseStamped()
                pose.header = out.header
                pose.pose = d3.bbox.center
                best[hyp.class_id] = (pose, hyp.score)

        with self._lock:
            self._latest.update(best)
        self._det_pub.publish(out)
        if self._target_class in best:
            self._pose_pub.publish(best[self._target_class][0])
        self._publish_markers(out)

    def _publish_markers(self, dets: Detection3DArray):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, d in enumerate(dets.detections):
            box = Marker()
            box.header = dets.header
            box.ns, box.id, box.type = 'objects', i, Marker.CUBE
            box.pose = d.bbox.center
            box.scale = d.bbox.size
            box.color.r, box.color.g, box.color.a = 0.1, 0.9, 0.5
            box.lifetime = Duration(seconds=0.5).to_msg()
            label = Marker()
            label.header = dets.header
            label.ns, label.id, label.type = 'labels', i, Marker.TEXT_VIEW_FACING
            label.pose = copy.deepcopy(d.bbox.center)  # don't move the detection itself
            label.pose.position.z += d.bbox.size.y / 2 + 0.03
            label.scale.z = 0.04
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            h = d.results[0].hypothesis
            label.text = f'{h.class_id} {h.score:.2f}'
            label.lifetime = box.lifetime
            markers.markers += [box, label]
        self._marker_pub.publish(markers)

    # ------------------------------------------------------------------ service
    def _locate(self, request, response):
        cls = request.class_name or self._target_class
        max_age = self.get_parameter('max_result_age_s').value
        deadline = time.monotonic() + max(0.0, request.timeout_s)
        while True:
            with self._lock:
                hit = self._latest.get(cls)
            if hit is not None:
                pose, score = hit
                age = (self.get_clock().now() - Time.from_msg(pose.header.stamp)).nanoseconds * 1e-9
                if age <= max_age:
                    response.found, response.pose, response.score = True, pose, float(score)
                    p = pose.pose.position
                    response.message = (f'{cls} at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m in '
                                        f'{pose.header.frame_id}, score {score:.2f}')
                    return response
            if time.monotonic() >= deadline:
                response.found = False
                response.message = f'No {cls} seen in the last {max_age:.1f}s.'
                return response
            time.sleep(0.05)


def main():
    rclpy.init()
    node = LocalizationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
