"""ROS 2 node: collect (robot pose, board pose) samples and solve hand-eye calibration.

Subscribes
  board_pose_topic  geometry_msgs/PoseStamped  (from calib_target_detector)
  TF                robot_base_frame -> robot_effector_frame (from the UR driver)

Services
  ~/take_sample   handeye_interfaces/TakeSample        record one sample (arm must be still)
  ~/remove_last   std_srvs/Trigger                     drop the most recent sample
  ~/reset         std_srvs/Trigger                     drop all samples
  ~/compute       handeye_interfaces/ComputeCalibration solve (optionally save to output_file)
"""
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from handeye_calibration import calibration_io, solver
from handeye_interfaces.srv import ComputeCalibration, TakeSample
from ur5_common.ros_conversions import matrix_to_transform, pose_to_matrix, transform_to_matrix
from ur5_common.transforms import invert, rotation_angle


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('handeye_calibration')
        self.declare_parameter('mount', 'eye_to_hand')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('robot_effector_frame', 'tool0')
        self.declare_parameter('board_pose_topic', '/target_detector/board_pose')
        self.declare_parameter('method', 'park')
        self.declare_parameter('max_board_pose_age_s', 0.5)
        # Reject a sample if the flange moved more than this between the image stamp and now.
        self.declare_parameter('max_motion_m', 0.002)
        self.declare_parameter('max_motion_deg', 0.3)
        self.declare_parameter('output_file', '~/.ros/handeye/ur5_handeye.yaml')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._mount = g('mount')
        if self._mount not in solver.MOUNTS:
            raise ValueError(f'mount must be one of {solver.MOUNTS}')
        if g('method') not in solver.METHODS:
            raise ValueError(f'method must be one of {sorted(solver.METHODS)}')
        self._base = g('robot_base_frame')
        self._effector = g('robot_effector_frame')

        self._lock = threading.Lock()
        self._latest_board: PoseStamped | None = None
        self._robot_poses: list[np.ndarray] = []
        self._target_poses: list[np.ndarray] = []
        self._camera_frame: str | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cb = ReentrantCallbackGroup()
        self.create_subscription(PoseStamped, g('board_pose_topic'), self._on_board, 10,
                                 callback_group=cb)
        self.create_service(TakeSample, '~/take_sample', self._take_sample, callback_group=cb)
        self.create_service(Trigger, '~/remove_last', self._remove_last, callback_group=cb)
        self.create_service(Trigger, '~/reset', self._reset, callback_group=cb)
        self.create_service(ComputeCalibration, '~/compute', self._compute, callback_group=cb)

        self.get_logger().info(
            f'Hand-eye calibration ready: mount={self._mount}, '
            f'robot {self._base} -> {self._effector}, method={g("method")}')

    # ------------------------------------------------------------------ callbacks
    def _on_board(self, msg: PoseStamped):
        with self._lock:
            self._latest_board = msg

    def _take_sample(self, request, response):
        response.success = False
        with self._lock:
            board = self._latest_board
            response.num_samples = len(self._robot_poses)
        if board is None:
            response.message = 'No board pose received yet. Is the board in view?'
            return response

        stamp = Time.from_msg(board.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > self.get_parameter('max_board_pose_age_s').value:
            response.message = f'Latest board pose is {age:.2f}s old; board probably not visible.'
            return response

        try:
            tf_at_image = self._tf_buffer.lookup_transform(
                self._base, self._effector, stamp, timeout=Duration(seconds=0.5))
            tf_now = self._tf_buffer.lookup_transform(
                self._base, self._effector, Time(), timeout=Duration(seconds=0.5))
        except TransformException as e:
            response.message = f'TF lookup {self._base} -> {self._effector} failed: {e}'
            return response

        T_bg = transform_to_matrix(tf_at_image.transform)
        drift = invert(T_bg) @ transform_to_matrix(tf_now.transform)
        moved_m = float(np.linalg.norm(drift[:3, 3]))
        moved_deg = float(np.degrees(rotation_angle(drift)))
        if (moved_m > self.get_parameter('max_motion_m').value
                or moved_deg > self.get_parameter('max_motion_deg').value):
            response.message = (f'Arm is moving ({moved_m * 1000:.1f} mm, {moved_deg:.2f} deg); '
                                'wait until it is still.')
            return response

        with self._lock:
            if self._camera_frame not in (None, board.header.frame_id):
                response.message = (f'Board frame changed from {self._camera_frame} to '
                                    f'{board.header.frame_id}; reset first.')
                return response
            self._camera_frame = board.header.frame_id
            self._robot_poses.append(T_bg)
            self._target_poses.append(pose_to_matrix(board.pose))
            n = len(self._robot_poses)
            spread = solver.rotation_spread_deg(self._robot_poses)

        response.success = True
        response.num_samples = n
        response.message = f'Sample {n} recorded (wrist rotation spread so far: {spread:.1f} deg).'
        self.get_logger().info(response.message)
        return response

    def _remove_last(self, request, response):
        with self._lock:
            if self._robot_poses:
                self._robot_poses.pop()
                self._target_poses.pop()
            n = len(self._robot_poses)
            if n == 0:
                self._camera_frame = None
        response.success = True
        response.message = f'{n} samples remaining.'
        return response

    def _reset(self, request, response):
        with self._lock:
            self._robot_poses.clear()
            self._target_poses.clear()
            self._camera_frame = None
        response.success = True
        response.message = 'All samples cleared.'
        return response

    def _compute(self, request, response):
        with self._lock:
            robot = list(self._robot_poses)
            target = list(self._target_poses)
            camera_frame = self._camera_frame
        method = self.get_parameter('method').value
        try:
            result = solver.solve(robot, target, self._mount, method)
        except Exception as e:  # ValueError from our checks, cv2.error from OpenCV
            response.success = False
            response.message = f'Calibration failed: {e}'
            return response

        parent = self._effector if self._mount == 'eye_in_hand' else self._base
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = camera_frame
        msg.transform = matrix_to_transform(result.X)

        response.success = True
        response.transform = msg
        response.translation_residual_m = result.translation_residual_m
        response.rotation_residual_deg = result.rotation_residual_deg
        spread = solver.rotation_spread_deg(robot)
        response.message = (
            f'{result.num_samples} samples, residual {result.translation_residual_m * 1000:.1f} mm'
            f' / {result.rotation_residual_deg:.2f} deg, rotation spread {spread:.0f} deg.')
        if spread < 30.0:
            response.message += ' WARNING: low rotation spread, result may be inaccurate.'

        if request.save:
            out = self.get_parameter('output_file').value
            path = calibration_io.save_calibration(out, result, self._mount, parent, camera_frame)
            calibration_io.save_samples(
                calibration_io.samples_path_for(out), robot, target,
                {'mount': self._mount, 'robot_base_frame': self._base,
                 'robot_effector_frame': self._effector, 'camera_frame': camera_frame})
            response.message += f' Saved to {path}.'
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = CalibrationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
