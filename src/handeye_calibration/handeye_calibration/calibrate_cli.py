"""Interactive terminal front-end for a hand-eye calibration session.

  ros2 run handeye_calibration calibrate_cli

Wraps the calibration node's services so sampling is one keystroke instead of a
`ros2 service call` per pose, and shows whether the board and the robot TF are live.
Line-based (type a command, press Enter) so it works over SSH and in any terminal.
"""
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from handeye_interfaces.srv import ComputeCalibration, TakeSample

HELP = """\
  [Enter]  take a sample (arm must be still, board in view)
  p        take a sample AND record the joint pose for replay (needs use_motion:=true)
  u        undo the last sample
  r        reset (drop all samples)
  c        compute and show the result (does not save)
  s        compute and SAVE the result
  ?        status: board / robot TF / sample count
  h        this help
  q        quit
"""


class CalibrationConsole(Node):

    def __init__(self):
        super().__init__('handeye_console')
        self.declare_parameter('calibration_node', '/handeye_calibration')
        self.declare_parameter('motion_node', '/handeye_motion')
        self.declare_parameter('board_pose_topic', '/target_detector/board_pose')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('robot_effector_frame', 'tool0')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        calib, motion = g('calibration_node'), g('motion_node')
        self._base, self._effector = g('robot_base_frame'), g('robot_effector_frame')

        self._take = self.create_client(TakeSample, f'{calib}/take_sample')
        self._undo = self.create_client(Trigger, f'{calib}/remove_last')
        self._reset = self.create_client(Trigger, f'{calib}/reset')
        self._compute = self.create_client(ComputeCalibration, f'{calib}/compute')
        self._record_pose = self.create_client(Trigger, f'{motion}/record_pose')

        self._lock = threading.Lock()
        self._board: PoseStamped | None = None
        self._board_rx = 0.0      # wall time of the last board pose
        self._num_samples = 0
        self.create_subscription(PoseStamped, g('board_pose_topic'), self._on_board, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _on_board(self, msg):
        with self._lock:
            self._board, self._board_rx = msg, time.monotonic()

    # ------------------------------------------------------------------ status
    def board_status(self):
        with self._lock:
            board, age = self._board, time.monotonic() - self._board_rx
        if board is None:
            return False, 'NOT SEEN yet (check the debug image in RViz)'
        if age > 0.5:
            return False, f'NOT SEEN for {age:.1f}s'
        p = board.pose.position
        dist = (p.x ** 2 + p.y ** 2 + p.z ** 2) ** 0.5
        return True, f'visible, {dist:.2f} m from camera ({board.header.frame_id})'

    def robot_status(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._base, self._effector, Time())
        except TransformException:
            return False, f'no TF {self._base} -> {self._effector} (is the UR driver up?)'
        t = tf.transform.translation
        return True, f'{self._base} -> {self._effector} at ({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f}) m'

    def status_lines(self):
        board_ok, board = self.board_status()
        robot_ok, robot = self.robot_status()
        mark = lambda ok: 'OK ' if ok else '!! '  # noqa: E731
        return [f'  {mark(board_ok)}board : {board}',
                f'  {mark(robot_ok)}robot : {robot}',
                f'     samples: {self._num_samples}']

    # ------------------------------------------------------------------ service calls
    def _call(self, client, request, timeout_s=10.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return None, f'service {client.srv_name} is not available'
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            return None, f'service {client.srv_name} timed out'
        return future.result(), None

    def wait_for_calibration_node(self, timeout_s):
        return self._take.wait_for_service(timeout_sec=timeout_s)

    def take_sample(self, record_pose=False):
        res, err = self._call(self._take, TakeSample.Request())
        if err:
            return err
        self._num_samples = res.num_samples
        if not res.success:
            return f'REJECTED: {res.message}'
        msg = res.message
        if record_pose:
            rec, err = self._call(self._record_pose, Trigger.Request())
            msg += f'\n  pose: {err or rec.message}'
        return msg

    def undo(self):
        res, err = self._call(self._undo, Trigger.Request())
        if err:
            return err
        self._num_samples = max(0, self._num_samples - 1)
        return res.message

    def reset(self):
        res, err = self._call(self._reset, Trigger.Request())
        if err:
            return err
        self._num_samples = 0
        return res.message

    def compute(self, save):
        res, err = self._call(self._compute, ComputeCalibration.Request(save=save), timeout_s=30.0)
        if err:
            return err
        if not res.success:
            return res.message
        tf = res.transform
        t, q = tf.transform.translation, tf.transform.rotation
        return (f'{res.message}\n'
                f'  {tf.header.frame_id} -> {tf.child_frame_id}\n'
                f'  translation [m]  x={t.x:+.4f} y={t.y:+.4f} z={t.z:+.4f}\n'
                f'  quaternion       x={q.x:+.4f} y={q.y:+.4f} z={q.z:+.4f} w={q.w:+.4f}')


def run_console(node, read_line=input, out=print):
    out('Hand-eye calibration console. Type h for help.')
    out('Waiting for the calibration node...')
    if not node.wait_for_calibration_node(timeout_s=15.0):
        out('!! /handeye_calibration services not found. Is calibration.launch.py running?')
        return 1
    deadline = time.monotonic() + 5.0   # give the first board pose and TF time to arrive
    while time.monotonic() < deadline and not (node.board_status()[0] and node.robot_status()[0]):
        time.sleep(0.2)
    out('\n'.join(node.status_lines()))
    out(HELP)

    actions = {
        '': lambda: node.take_sample(),
        't': lambda: node.take_sample(),
        'p': lambda: node.take_sample(record_pose=True),
        'u': node.undo,
        'r': node.reset,
        'c': lambda: node.compute(save=False),
        's': lambda: node.compute(save=True),
        '?': lambda: '\n'.join(node.status_lines()),
        'h': lambda: HELP,
    }
    while rclpy.ok():
        try:
            line = read_line(f'[{node._num_samples} samples] > ').strip().lower()
        except EOFError:
            break
        if line in ('q', 'quit', 'exit'):
            break
        action = actions.get(line)
        out(action() if action else f'Unknown command {line!r}. Type h for help.')
    return 0


def _spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main():
    rclpy.init()
    node = CalibrationConsole()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spinner.start()
    code = 0
    try:
        code = run_console(node)
    except KeyboardInterrupt:
        print()
    finally:
        # Shut the context down first so the spinning thread wakes up and exits;
        # tearing the node down under a live executor aborts the process.
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
    return code
