"""Example of robot code that uses the Quest: copy this into your own package.

Needs only standard ROS messages, nothing from this repo:
  /quest/{head,left_controller,right_controller}/pose   geometry_msgs/PoseStamped in base_link
  /quest/right_controller/joy                           sensor_msgs/Joy (axes[2] = trigger)
  TF base_link -> quest_right_controller, and the robot's own base_link -> tool0

Once a second it logs what it received and how far the right controller is from the tool flange,
which works only because the Quest and the robot share one coordinate system.
  ros2 run quest_listener_example listener
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformException, TransformListener

DEVICES = ('head', 'left_controller', 'right_controller')


class QuestListener(Node):
    def __init__(self):
        super().__init__('quest_listener_example')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('tool_frame', 'tool0')
        self._robot = self.get_parameter('robot_frame').value
        self._tool = self.get_parameter('tool_frame').value
        self._latest = {d: None for d in DEVICES}
        self._count = {d: 0 for d in DEVICES}
        self._trigger = None
        for d in DEVICES:
            self.create_subscription(PoseStamped, f'/quest/{d}/pose',
                                     lambda msg, d=d: self._on_pose(d, msg), 10)
        self.create_subscription(Joy, '/quest/right_controller/joy', self._on_joy, 10)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._period = 1.0
        self.create_timer(self._period, self._report)

    def _on_pose(self, device, msg):
        if msg.header.frame_id != self._robot:
            self.get_logger().warn(f'{device} pose is in {msg.header.frame_id!r}, not {self._robot!r}',
                                   throttle_duration_sec=5.0)
        self._latest[device] = msg
        self._count[device] += 1

    def _on_joy(self, msg):
        self._trigger = msg.axes[2] if len(msg.axes) > 2 else None

    def _fresh_tf(self, target, source):
        """TF target <- source, or None if missing or older than 0.2 s (TF keeps the last pose)."""
        try:
            tf = self._tf.lookup_transform(target, source, Time())
        except TransformException:
            return None
        stamp = Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds and self.get_clock().now() - stamp > Duration(seconds=0.2):
            return None
        return tf.transform.translation

    def _report(self):
        parts = []
        for d in DEVICES:
            msg, n = self._latest[d], self._count[d]
            self._count[d] = 0
            if msg is None or n == 0:
                parts.append(f'{d}: no poses (not tracked or not registered)')
            else:
                p = msg.pose.position
                parts.append(f'{d}: {n / self._period:.0f} Hz at ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) m')
        hand = self._fresh_tf(self._robot, 'quest_right_controller')
        try:   # the robot's own frame: static in the sim, joint-driven on the real arm
            tool = self._tf.lookup_transform(self._robot, self._tool, Time()).transform.translation
        except TransformException:
            tool = None
        if hand and tool:
            dist = math.dist((hand.x, hand.y, hand.z), (tool.x, tool.y, tool.z))
            parts.append(f'right controller -> {self._tool}: {dist:.3f} m (TF)')
        if self._trigger is not None:
            parts.append(f'right trigger {self._trigger:.2f}')
        self.get_logger().info(' | '.join(parts))


def main():
    rclpy.init()
    node = QuestListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C or SIGTERM
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
