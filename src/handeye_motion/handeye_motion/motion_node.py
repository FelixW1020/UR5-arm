"""ROS 2 node: record joint poses during freedrive, then replay them for calibration.

Workflow
  1. Put the UR5 in freedrive / jog it with the pendant to a good view of the board.
  2. ros2 service call /handeye_motion/record_pose std_srvs/srv/Trigger   (repeat 12-20x)
  3. Later, with the External Control program running on the pendant:
     ros2 service call /handeye_motion/run std_srvs/srv/Trigger
     The arm visits each pose in joint space, waits to settle, and calls
     /handeye_calibration/take_sample.  ~/stop aborts after the current move.

Moves are joint-space interpolations between poses YOU recorded, at a capped joint
speed, so the arm only travels through the region you already showed it. Keep the
pendant's emergency stop within reach anyway.
"""
import os
import threading

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from handeye_interfaces.srv import TakeSample

UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def _wait(future, timeout_s):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout_s):
        return None
    return future.result()


class MotionNode(Node):

    def __init__(self):
        super().__init__('handeye_motion')
        self.declare_parameter('joint_names', UR_JOINTS)
        self.declare_parameter('poses_file', '~/.ros/handeye/calib_poses.yaml')
        self.declare_parameter(
            'trajectory_action',
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('take_sample_service', '/handeye_calibration/take_sample')
        self.declare_parameter('max_joint_speed', 0.3)   # rad/s, caps every move
        self.declare_parameter('min_move_duration_s', 3.0)
        self.declare_parameter('settle_s', 1.5)

        self._joints = list(self.get_parameter('joint_names').value)
        self._poses_file = os.path.expanduser(self.get_parameter('poses_file').value)
        self._poses = self._load_poses()
        self._current = None
        self._lock = threading.Lock()
        self._worker = None
        self._stop = threading.Event()

        cb = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10, callback_group=cb)
        self._traj = ActionClient(self, FollowJointTrajectory,
                                  self.get_parameter('trajectory_action').value, callback_group=cb)
        self._sample = self.create_client(
            TakeSample, self.get_parameter('take_sample_service').value, callback_group=cb)
        self.create_service(Trigger, '~/record_pose', self._record, callback_group=cb)
        self.create_service(Trigger, '~/clear_poses', self._clear, callback_group=cb)
        self.create_service(Trigger, '~/run', self._run, callback_group=cb)
        self.create_service(Trigger, '~/stop', self._stop_srv, callback_group=cb)
        self.get_logger().info(f'{len(self._poses)} poses loaded from {self._poses_file}')

    # ------------------------------------------------------------------ pose storage
    def _load_poses(self):
        if not os.path.exists(self._poses_file):
            return []
        with open(self._poses_file) as f:
            data = yaml.safe_load(f) or {}
        if data.get('joint_names', self._joints) != self._joints:
            raise ValueError(f'{self._poses_file} was recorded with different joint names')
        return [list(p) for p in data.get('poses', [])]

    def _save_poses(self):
        os.makedirs(os.path.dirname(self._poses_file) or '.', exist_ok=True)
        with open(self._poses_file, 'w') as f:
            yaml.safe_dump({'joint_names': self._joints, 'poses': self._poses}, f)

    def _on_joints(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in self._joints):
            with self._lock:
                self._current = [float(pos[j]) for j in self._joints]

    # ------------------------------------------------------------------ services
    def _record(self, request, response):
        with self._lock:
            current = self._current
        if current is None:
            response.success, response.message = False, 'No /joint_states received yet.'
            return response
        self._poses.append(current)
        self._save_poses()
        response.success = True
        response.message = f'Recorded pose {len(self._poses)} -> {self._poses_file}'
        self.get_logger().info(response.message)
        return response

    def _clear(self, request, response):
        self._poses = []
        self._save_poses()
        response.success, response.message = True, 'Pose list cleared.'
        return response

    def _run(self, request, response):
        if self._worker is not None and self._worker.is_alive():
            response.success, response.message = False, 'Already running.'
        elif not self._poses:
            response.success, response.message = False, 'No poses recorded.'
        elif not self._traj.wait_for_server(timeout_sec=2.0):
            response.success, response.message = False, (
                'Trajectory controller not available. Is External Control running on the pendant?')
        else:
            self._stop.clear()
            self._worker = threading.Thread(target=self._run_all, daemon=True)
            self._worker.start()
            response.success, response.message = True, f'Visiting {len(self._poses)} poses.'
        return response

    def _stop_srv(self, request, response):
        self._stop.set()
        response.success, response.message = True, 'Will stop after the current move.'
        return response

    # ------------------------------------------------------------------ execution
    def _run_all(self):
        log = self.get_logger()
        ok = 0
        for i, target in enumerate(self._poses, start=1):
            if self._stop.is_set():
                log.warn('Stopped by request.')
                break
            log.info(f'Moving to pose {i}/{len(self._poses)}')
            if not self._move_to(target):
                log.error('Move failed; aborting run.')
                break
            self._stop.wait(self.get_parameter('settle_s').value)
            if self._take_sample(i):
                ok += 1
        log.info(f'Run finished: {ok} samples recorded. '
                 'Now call /handeye_calibration/compute.')

    def _move_to(self, target):
        with self._lock:
            current = self._current
        if current is None:
            return False
        delta = max(abs(a - b) for a, b in zip(current, target))
        duration = max(self.get_parameter('min_move_duration_s').value,
                       delta / self.get_parameter('max_joint_speed').value)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joints
        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * len(target)
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]

        handle = _wait(self._traj.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            self.get_logger().error('Trajectory goal rejected.')
            return False
        result = _wait(handle.get_result_async(), duration + 10.0)
        if result is None:
            handle.cancel_goal_async()
            return False
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Trajectory error: {result.result.error_string}')
            return False
        return True

    def _take_sample(self, i):
        if not self._sample.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('take_sample service not available.')
            return False
        for attempt in range(3):
            res = _wait(self._sample.call_async(TakeSample.Request()), 5.0)
            if res is not None and res.success:
                self.get_logger().info(f'Pose {i}: {res.message}')
                return True
            self.get_logger().warn(
                f'Pose {i} attempt {attempt + 1}: {res.message if res else "timeout"}')
            self._stop.wait(0.5)
        return False


def main():
    rclpy.init()
    node = MotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
