"""ROS 2 node: image + camera_info -> ChArUco board pose in the camera optical frame.

Publishes
  ~/board_pose   geometry_msgs/PoseStamped  (frame = image frame_id)
  ~/debug_image  sensor_msgs/Image          (axes + corners drawn)
  TF camera_optical_frame -> board_frame    (optional, for RViz)
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from calib_target_detector.charuco import CharucoBoardDetector
from ur5_common.ros_conversions import matrix_to_pose, matrix_to_transform
from ur5_common.transforms import make_transform


class TargetDetectorNode(Node):

    def __init__(self):
        super().__init__('target_detector')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('squares_x', 7)
        self.declare_parameter('squares_y', 5)
        self.declare_parameter('square_length', 0.035)
        self.declare_parameter('marker_length', 0.026)
        self.declare_parameter('dictionary', 'DICT_5X5_100')
        self.declare_parameter('min_corners', 6)
        self.declare_parameter('max_reprojection_error_px', 2.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('board_frame', 'calib_board')
        self.declare_parameter('publish_debug_image', True)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._detector = CharucoBoardDetector(
            g('squares_x'), g('squares_y'), g('square_length'), g('marker_length'),
            g('dictionary'), g('min_corners'))
        self._max_err = g('max_reprojection_error_px')
        self._board_frame = g('board_frame')
        self._debug = g('publish_debug_image')

        self._bridge = CvBridge()
        self._K = None
        self._D = None
        self._tf = TransformBroadcaster(self) if g('publish_tf') else None

        self._pose_pub = self.create_publisher(PoseStamped, '~/board_pose', 10)
        self._debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self.create_subscription(CameraInfo, g('camera_info_topic'), self._on_info, 10)
        self.create_subscription(Image, g('image_topic'), self._on_image, qos_profile_sensor_data)
        self.get_logger().info('Waiting for camera_info and images...')

    def _on_info(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def _on_image(self, msg: Image):
        if self._K is None:
            return
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = self._detector.detect(gray, self._K, self._D)

        if result is not None and result.reprojection_error_px <= self._max_err:
            T_cam_board = make_transform(cv2.Rodrigues(result.rvec)[0], result.tvec)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = matrix_to_pose(T_cam_board)
            self._pose_pub.publish(pose)
            if self._tf is not None:
                t = TransformStamped()
                t.header = msg.header
                t.child_frame_id = self._board_frame
                t.transform = matrix_to_transform(T_cam_board)
                self._tf.sendTransform(t)

        if self._debug and self._debug_pub.get_subscription_count() > 0:
            self._draw(frame, result)
            out = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = msg.header
            self._debug_pub.publish(out)

    def _draw(self, frame, result):
        if result is None:
            cv2.putText(frame, 'board: not found', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return
        cv2.aruco.drawDetectedCornersCharuco(frame, result.charuco_corners, result.charuco_ids)
        cv2.drawFrameAxes(frame, self._K, self._D, result.rvec, result.tvec, 0.1)
        ok = result.reprojection_error_px <= self._max_err
        cv2.putText(frame, f'corners={result.num_corners} err={result.reprojection_error_px:.2f}px'
                    f' dist={np.linalg.norm(result.tvec):.3f}m', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if ok else (0, 0, 255), 2)


def main():
    rclpy.init()
    node = TargetDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
