"""ROS 2 node: register the Meta Quest world (vr_world) to the robot (base_link).

Two methods, same output (static TF base_link -> vr_world, saved to output_file):

  Tags (~/register_tags): the Quest reports where it sees / touches the ArUco tags;
      their robot-frame positions come from aruco_tag_mapper's tag map.
  Controller on flange (~/add_controller_sample, ~/solve_controller): a Touch
      controller is clamped to the tool flange; each sample pairs its VR pose with
      the robot's tool0 pose from TF.

~/get_tag_map returns the tags in vr_world so the game can draw them for a visual check.
A saved registration is re-published on start-up.
"""
import os
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformException,
                     TransformListener)

from aruco_tag_mapper.tag_map import load_tag_map
from ur5_common.ros_conversions import (matrix_to_pose, matrix_to_transform, pose_to_matrix,
                                        transform_to_matrix)
from ur5_common.transforms import invert, rotation_angle
from vr_interfaces.srv import AddControllerSample, GetTagMap, RegisterTags, SolveRegistration
from vr_registration import registration as reg
from vr_registration.registration_io import load_registration, save_registration


class RegistrationNode(Node):

    def __init__(self):
        super().__init__('vr_registration')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('vr_frame', 'vr_world')
        self.declare_parameter('effector_frame', 'tool0')
        self.declare_parameter('tag_map_file', '~/.ros/handeye/tag_map.yaml')
        self.declare_parameter('mode', 'yaw_only')          # yaw_only | full (tag method)
        self.declare_parameter('output_file', '~/.ros/handeye/vr_registration.yaml')
        self.declare_parameter('load_on_start', True)
        self.declare_parameter('warn_rms_m', 0.03)
        self.declare_parameter('max_motion_m', 0.002)
        self.declare_parameter('max_motion_deg', 0.3)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        if g('mode') not in reg.MODES:
            raise ValueError(f'mode must be one of {reg.MODES}')
        self._base, self._vr, self._effector = g('base_frame'), g('vr_frame'), g('effector_frame')
        self._T_base_vr = None
        self._lock = threading.Lock()
        self._samples = []          # (T_base_tool0, T_vr_controller)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._static = StaticTransformBroadcaster(self)

        cb = ReentrantCallbackGroup()
        self.create_service(RegisterTags, '~/register_tags', self._register_tags, callback_group=cb)
        self.create_service(AddControllerSample, '~/add_controller_sample', self._add_sample,
                            callback_group=cb)
        self.create_service(SolveRegistration, '~/solve_controller', self._solve_controller,
                            callback_group=cb)
        self.create_service(Trigger, '~/reset_controller_samples', self._reset_samples,
                            callback_group=cb)
        self.create_service(GetTagMap, '~/get_tag_map', self._get_tag_map, callback_group=cb)

        out = os.path.expanduser(g('output_file'))
        if g('load_on_start') and os.path.exists(out):
            parent, child, T, d = load_registration(out)
            if (parent, child) == (self._base, self._vr):
                self._publish(T)
                self.get_logger().info(f'Loaded registration from {out} ({d["method"]}, '
                                       f'rms {d["rms_error_m"] * 1000:.1f} mm, {d["created"]})')
        self.get_logger().info('VR registration ready.')

    # ------------------------------------------------------------------ helpers
    def _publish(self, T_base_vr):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base
        msg.child_frame_id = self._vr
        msg.transform = matrix_to_transform(T_base_vr)
        self._static.sendTransform(msg)
        with self._lock:
            self._T_base_vr = T_base_vr
        return msg

    def _finish(self, r, request_save, response, extra):
        response.success = True
        response.base_to_vr = self._publish(r.T_base_vr)
        response.rms_error_m = r.rms_error_m
        response.max_error_m = r.max_error_m
        response.message = (f'{r.method}: rms {r.rms_error_m * 1000:.1f} mm, '
                            f'max {r.max_error_m * 1000:.1f} mm.')
        if r.rms_error_m > self.get_parameter('warn_rms_m').value:
            response.message += ' WARNING: large error, check tag ids / measurements.'
        if request_save:
            path = save_registration(self.get_parameter('output_file').value, r,
                                     self._base, self._vr, extra)
            response.message += f' Saved to {path}.'
        self.get_logger().info(response.message)

    # ------------------------------------------------------------------ tag method
    def _register_tags(self, request, response):
        response.success = False
        try:
            frame, tags = load_tag_map(self.get_parameter('tag_map_file').value)
        except (OSError, KeyError) as e:
            response.message = f'Cannot read tag map ({e}). Run aruco_tag_mapper and ~/save first.'
            return response
        if frame != self._base:
            response.message = f'Tag map is in {frame}, expected {self._base}.'
            return response

        seen = {}
        for o in request.observations:      # average repeated observations of the same tag
            seen.setdefault(o.id, []).append([o.position.x, o.position.y, o.position.z])
        used = sorted(i for i in seen if i in tags)
        unknown = sorted(i for i in seen if i not in tags)
        try:
            r = reg.fit_points([np.mean(seen[i], axis=0) for i in used],
                               [tags[i][:3, 3] for i in used],
                               self.get_parameter('mode').value)
        except ValueError as e:
            response.message = f'Registration failed: {e} (matched tags {used}, unknown {unknown}).'
            return response
        response.used_ids = used
        response.errors_m = [float(e) for e in r.errors_m]
        self._finish(r, request.save, response, {'tags': used})
        if unknown:
            response.message += f' Ignored tags not in the map: {unknown}.'
        return response

    # ------------------------------------------------------------------ controller method
    def _add_sample(self, request, response):
        response.success = False
        now = self.get_clock().now()
        try:
            tf_now = self._tf_buffer.lookup_transform(self._base, self._effector, Time(),
                                                      timeout=Duration(seconds=0.5))
            tf_before = self._tf_buffer.lookup_transform(
                self._base, self._effector, now - Duration(seconds=0.3),
                timeout=Duration(seconds=0.5))
        except TransformException as e:
            response.message = f'TF {self._base} -> {self._effector} unavailable: {e}'
            return response
        T_now = transform_to_matrix(tf_now.transform)
        drift = invert(transform_to_matrix(tf_before.transform)) @ T_now
        if (np.linalg.norm(drift[:3, 3]) > self.get_parameter('max_motion_m').value
                or np.degrees(rotation_angle(drift)) > self.get_parameter('max_motion_deg').value):
            response.message = 'Robot is moving; wait until it is still.'
            return response
        with self._lock:
            self._samples.append((T_now, pose_to_matrix(request.controller_pose)))
            response.num_samples = len(self._samples)
        response.success = True
        response.message = f'Sample {response.num_samples} recorded.'
        self.get_logger().info(response.message)
        return response

    def _solve_controller(self, request, response):
        with self._lock:
            samples = list(self._samples)
        try:
            r, T_tool0_ctrl = reg.fit_controller_on_flange([s[0] for s in samples],
                                                           [s[1] for s in samples])
        except Exception as e:  # ValueError from our checks, cv2.error from OpenCV
            response.success = False
            response.message = f'Registration failed: {e}'
            return response
        clamp = matrix_to_pose(T_tool0_ctrl)
        self._finish(r, request.save, response, {
            'num_samples': len(samples),
            'tool0_to_controller': {
                'position': [clamp.position.x, clamp.position.y, clamp.position.z],
                'orientation_xyzw': [clamp.orientation.x, clamp.orientation.y,
                                     clamp.orientation.z, clamp.orientation.w]}})
        response.message += (f' Clamp offset tool0->controller: '
                             f'{np.linalg.norm(T_tool0_ctrl[:3, 3]) * 100:.1f} cm '
                             '(sanity-check this against a ruler).')
        return response

    def _reset_samples(self, request, response):
        with self._lock:
            self._samples.clear()
        response.success, response.message = True, 'Controller samples cleared.'
        return response

    # ------------------------------------------------------------------ query
    def _get_tag_map(self, request, response):
        try:
            frame, tags = load_tag_map(self.get_parameter('tag_map_file').value)
        except (OSError, KeyError) as e:
            response.success, response.message = False, f'Cannot read tag map: {e}'
            return response
        with self._lock:
            T_base_vr = self._T_base_vr
        if T_base_vr is not None:
            T_vr_base = invert(T_base_vr)
            tags = {i: T_vr_base @ T for i, T in tags.items()}
            frame = self._vr
        response.success = True
        response.frame_id = frame
        response.ids = sorted(tags)
        response.poses = [matrix_to_pose(tags[i]) for i in response.ids]
        response.message = f'{len(tags)} tags in {frame}.'
        return response


def main():
    rclpy.init()
    node = RegistrationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
