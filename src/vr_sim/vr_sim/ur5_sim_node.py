"""ROS 2 node: a kinematic UR5 (no dynamics, no driver) that follows a streamed tool0 target.

  ros2 run vr_sim ur5_kinematic_sim      # + robot_state_publisher with the matching UR URDF

Subscribes target_topic (geometry_msgs/PoseStamped of tool0 in base_frame, e.g. vr_teleop's
/vr_teleop/target_pose) and publishes /joint_states for robot_state_publisher, so RViz shows
the arm and TF base_link -> tool0 exists exactly as with the real driver. Each tick refines a
damped-least-squares IK solution (warm-started from the previous one) and moves the joints
toward it at no more than max_joint_velocity (all joints slowed together, so they arrive
together and the tool keeps to the commanded path).

  ~/home   std_srvs/Trigger   drop the target and return to initial_positions

check_fk compares this node's DH forward kinematics with robot_state_publisher's TF once at
start-up; a mismatch (wrong ur_type, edited URDF) is logged loudly instead of silently
showing a tool pose that disagrees with the arm drawn in RViz.
"""
from collections import deque
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from ur5_common.ros_conversions import pose_to_matrix, transform_to_matrix
from ur5_common.transforms import rotation_angle
from vr_sim import ur_kinematics as uk

IK_ITERATIONS_PER_TICK = 10
UNREACHABLE_M, UNREACHABLE_RAD = 0.005, np.radians(1.0)
FK_OK_M, FK_OK_DEG = 0.001, 0.1


class UR5KinematicSim(Node):

    def __init__(self):
        super().__init__('ur5_kinematic_sim')
        self.declare_parameter('ur_type', 'ur5')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('initial_positions',
                               [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
        self.declare_parameter('target_topic', '/vr_teleop/target_pose')
        self.declare_parameter('max_joint_velocity', 1.0)       # rad/s, every joint
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('check_fk', True)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._ur_type = g('ur_type')
        uk.dh_table(self._ur_type)                               # ValueError if unknown
        self._home = np.asarray(g('initial_positions'), dtype=float)
        if self._home.shape != (6,):
            raise ValueError('initial_positions must hold 6 joint angles (rad)')
        self._home = uk.clamp_to_limits(self._home)
        self._base, self._tool = g('base_frame'), g('tool_frame')
        self._rate = float(g('rate_hz'))
        self._max_step = float(g('max_joint_velocity')) / self._rate

        self._lock = threading.Lock()
        self._q = self._home.copy()
        self._q_goal = self._home.copy()
        self._T_target = None
        self._ik_ticks = 0
        self._history = deque(maxlen=int(5 * self._rate) + 1)   # (stamp ns, q) for check_fk

        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(PoseStamped, g('target_topic'), self._on_target, 10)
        self.create_service(Trigger, '~/home', self._on_home)
        self.create_timer(1.0 / self._rate, self._tick)

        if g('check_fk'):
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._fk_tries = 0
            self._fk_timer = self.create_timer(1.0, self._check_fk)
        self.get_logger().info(
            f'Kinematic {self._ur_type} at {self._rate:.0f} Hz following {g("target_topic")} '
            f'({self._base} -> {self._tool}, <= {g("max_joint_velocity"):.2f} rad/s).')

    def _on_target(self, msg: PoseStamped):
        if msg.header.frame_id != self._base:
            self.get_logger().warn(f'Ignoring target in frame "{msg.header.frame_id}"; expected '
                                   f'"{self._base}".', throttle_duration_sec=5.0)
            return
        with self._lock:
            self._T_target = pose_to_matrix(msg.pose)
            self._ik_ticks = 0

    def _on_home(self, request, response):
        with self._lock:
            self._T_target = None
            self._q_goal = self._home.copy()
        response.success, response.message = True, 'Returning to initial_positions.'
        self.get_logger().info(response.message)
        return response

    def _tick(self):
        with self._lock:
            if self._T_target is not None:
                self._q_goal, pos_err, rot_err = uk.solve_ik(
                    self._q_goal, self._T_target, self._ur_type,
                    max_iterations=IK_ITERATIONS_PER_TICK)
                self._ik_ticks += 1
                if self._ik_ticks > self._rate and (pos_err > UNREACHABLE_M
                                                    or rot_err > UNREACHABLE_RAD):
                    self.get_logger().warn(
                        f'Target not reachable: closest tool0 pose is off by '
                        f'{pos_err * 1000:.0f} mm / {np.degrees(rot_err):.1f} deg.',
                        throttle_duration_sec=5.0)
            dq = uk.limit_joint_step(self._q_goal - self._q, self._max_step)
            self._q = self._q + dq
            q = self._q.copy()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(uk.JOINT_NAMES)
        msg.position = [float(v) for v in q]
        msg.velocity = [float(v) * self._rate for v in dq]
        self._joint_pub.publish(msg)
        self._history.append((Time.from_msg(msg.header.stamp).nanoseconds, q))

    def _check_fk(self):
        self._fk_tries += 1
        if self._fk_tries < 2:                  # give robot_state_publisher ~2 s
            return
        try:
            tf = self._tf_buffer.lookup_transform(self._base, self._tool, Time())
        except TransformException as e:
            if self._fk_tries >= 15:
                self.get_logger().warn(
                    f'check_fk: no TF {self._base} -> {self._tool} after {self._fk_tries} s ({e}). '
                    'Is robot_state_publisher running with the UR URDF? Skipping the check.')
                self._fk_timer.cancel()
            return
        stamp = Time.from_msg(tf.header.stamp).nanoseconds
        match = [q for t, q in list(self._history) if t == stamp]
        if not match:
            return                              # TF from before our history; try again
        T_tf = transform_to_matrix(tf.transform)
        T_fk = uk.fk(match[0], self._ur_type)
        d_m = float(np.linalg.norm(T_tf[:3, 3] - T_fk[:3, 3]))
        d_deg = float(np.degrees(rotation_angle(T_tf[:3, :3].T @ T_fk[:3, :3])))
        if d_m < FK_OK_M and d_deg < FK_OK_DEG:
            self.get_logger().info(
                f'check_fk OK: DH model ({self._ur_type}) matches the URDF TF {self._base} -> '
                f'{self._tool} ({d_m * 1000:.3f} mm, {d_deg:.4f} deg).')
        else:
            self.get_logger().error(
                f'check_fk FAILED: DH model ({self._ur_type}) and the URDF TF {self._base} -> '
                f'{self._tool} disagree by {d_m * 1000:.1f} mm / {d_deg:.2f} deg. The arm drawn '
                'in RViz is NOT where this sim thinks tool0 is. Does ur_type match the URDF?')
        self._fk_timer.cancel()


def main():
    rclpy.init()
    node = UR5KinematicSim()
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
