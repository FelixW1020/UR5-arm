"""Console command dispatch, with the ROS node replaced by a recorder."""
import rclpy

from handeye_calibration.calibrate_cli import run_console


class FakeConsoleNode:
    _num_samples = 0

    def __init__(self):
        self.calls = []

    def wait_for_calibration_node(self, timeout_s):
        return True

    def board_status(self):
        return True, 'visible'

    def robot_status(self):
        return True, 'ok'

    def status_lines(self):
        return ['status']

    def take_sample(self, record_pose=False):
        self.calls.append(('take', record_pose))
        return 'took'

    def undo(self):
        self.calls.append('undo')
        return 'undone'

    def reset(self):
        self.calls.append('reset')
        return 'reset'

    def compute(self, save):
        self.calls.append(('compute', save))
        return 'computed'


def run(lines):
    node, out = FakeConsoleNode(), []
    feed = iter(lines)

    def read_line(_prompt):
        try:
            return next(feed)
        except StopIteration:
            raise EOFError from None

    rclpy.init()
    try:
        code = run_console(node, read_line=read_line, out=out.append)
    finally:
        rclpy.try_shutdown()
    return code, node.calls, out


def test_keys_map_to_service_calls():
    code, calls, _ = run(['', 't', 'P', 'u', 'r', 'c', 's', 'q', 'never reached'])
    assert code == 0
    assert calls == [('take', False), ('take', False), ('take', True), 'undo', 'reset',
                     ('compute', False), ('compute', True)]


def test_unknown_command_then_eof_exits_cleanly():
    code, calls, out = run(['xyz'])
    assert code == 0
    assert calls == []
    assert any('Unknown command' in line for line in out)
