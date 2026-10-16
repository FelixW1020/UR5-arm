"""ROS 2 node: publish the calibrated camera pose as a static TF.

The calibration is solved for the camera *optical* frame, but the camera driver
(e.g. realsense2_camera) already publishes camera_link -> ..._optical_frame. A TF
frame can only have one parent, so instead of parenting the optical frame directly
we compute and publish  robot_frame -> camera_link  such that the driver's own
chain lands the optical frame exactly where the calibration says it is:

  T_robot_cameralink = T_robot_optical @ T_optical_cameralink

Set camera_base_frame to '' to publish robot_frame -> optical frame directly
(for cameras whose driver publishes no TF of its own).
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from handeye_calibration.calibration_io import load_calibration
from ur5_common.ros_conversions import matrix_to_transform, transform_to_matrix


class HandEyePublisher(Node):

    def __init__(self):
        super().__init__('handeye_publisher')
        self.declare_parameter('calibration_file', '~/.ros/handeye/table_cam.yaml')
        self.declare_parameter('camera_base_frame', 'table_cam_link')

        path = self.get_parameter('calibration_file').value
        try:
            self._parent, self._optical, self._T_parent_optical, data = load_calibration(path)
        except FileNotFoundError:
            raise SystemExit(
                f'No calibration file at {path}. Run the hand-eye calibration first '
                '(scripts/handeye_calibrate.sh <ROBOT_IP> <camera>) or set calibration_file:=...')
        self._camera_base = self.get_parameter('camera_base_frame').value
        self.get_logger().info(
            f'Loaded {path}: {data["mount"]}, {self._parent} -> {self._optical}, '
            f'{data["num_samples"]} samples, residual '
            f'{data["translation_residual_m"] * 1000:.1f} mm / {data["rotation_residual_deg"]:.2f} deg')

        self._broadcaster = StaticTransformBroadcaster(self)
        if not self._camera_base or self._camera_base == self._optical:
            self._publish(self._optical, self._T_parent_optical)
            return

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(1.0, self._try_publish)

    def _try_publish(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._optical, self._camera_base, Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            self.get_logger().info(
                f'Waiting for camera driver TF {self._camera_base} -> {self._optical}...',
                throttle_duration_sec=5.0)
            return
        self._timer.cancel()
        self._publish(self._camera_base, self._T_parent_optical @ transform_to_matrix(tf.transform))

    def _publish(self, child, T):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._parent
        msg.child_frame_id = child
        msg.transform = matrix_to_transform(T)
        self._broadcaster.sendTransform(msg)
        self.get_logger().info(f'Publishing static TF {self._parent} -> {child}')


def main():
    rclpy.init()
    try:
        node = HandEyePublisher()
    except SystemExit as e:
        print(f'[handeye_publisher] {e}')
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
