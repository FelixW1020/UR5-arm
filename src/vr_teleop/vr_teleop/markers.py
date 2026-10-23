"""RViz marker builders shared by the vr_teleop nodes."""
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

RED, GREEN, BLUE = (1.0, 0.1, 0.1, 1.0), (0.1, 0.9, 0.2, 1.0), (0.2, 0.4, 1.0, 1.0)
YELLOW, ORANGE, WHITE = (1.0, 0.9, 0.1, 1.0), (1.0, 0.5, 0.0, 1.0), (1.0, 1.0, 1.0, 1.0)


def _marker(header, ns, marker_id, marker_type, rgba):
    m = Marker()
    m.header = header
    m.ns, m.id, m.type, m.action = ns, marker_id, marker_type, Marker.ADD
    m.pose.orientation.w = 1.0
    m.color.r, m.color.g, m.color.b, m.color.a = (float(c) for c in rgba)
    return m


def arrow(header, ns, marker_id, start, end, rgba, shaft=0.008):
    m = _marker(header, ns, marker_id, Marker.ARROW, rgba)
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (start, end)]
    m.scale.x, m.scale.y, m.scale.z = shaft, 2.0 * shaft, 3.0 * shaft
    return m


def axes(header, ns, T, length=0.08):
    """x/y/z axes of the 4x4 pose T as red/green/blue arrows (ids 0..2)."""
    p = T[:3, 3]
    return [arrow(header, ns, i, p, p + length * T[:3, i], c, shaft=0.006)
            for i, c in enumerate((RED, GREEN, BLUE))]


def text(header, ns, marker_id, position, label, height=0.03):
    m = _marker(header, ns, marker_id, Marker.TEXT_VIEW_FACING, WHITE)
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in position)
    m.scale.z = height
    m.text = label
    return m
