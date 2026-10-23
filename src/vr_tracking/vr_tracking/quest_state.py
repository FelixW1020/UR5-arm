"""What quest_bridge publishes, as plain data (no ROS imports, so it is unit-testable anywhere).

The Joy layout is the contract with the UR5-side code (vr_teleop, the user's own nodes): index
by these tuples, never by magic numbers. Everything here duck-types on the field names of
vr_interfaces/QuestState and ControllerState, so tests can use SimpleNamespace fakes.
"""
import math

DEVICES = ('head', 'left_controller', 'right_controller')
CONTROLLERS = ('left_controller', 'right_controller')
_CONTROLLER_FIELD = {'left_controller': 'left', 'right_controller': 'right'}

# sensor_msgs/Joy layout of /quest/<side>_controller/joy.
JOY_AXES = ('thumbstick_x', 'thumbstick_y', 'index_trigger', 'hand_trigger')
JOY_BUTTONS = ('button_primary', 'button_secondary', 'thumbstick_pressed', 'menu')


def joy_arrays(controller):
    """ControllerState -> (axes, buttons) lists in the JOY_AXES / JOY_BUTTONS order."""
    return ([float(getattr(controller, n)) for n in JOY_AXES],
            [int(bool(getattr(controller, n))) for n in JOY_BUTTONS])


def controller_state(state, device):
    """The ControllerState of 'left_controller' / 'right_controller' in a QuestState."""
    return getattr(state, _CONTROLLER_FIELD[device])


def pose_is_valid(pose):
    """Finite position and a non-zero quaternion. Unity sends identity/zeros before tracking
    starts, and a zero quaternion is rejected by tf2 (and drawn garbled by RViz)."""
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return (all(math.isfinite(v) for v in values)
            and q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-6)


def device_states(state):
    """[(device, tracked, pose)] for a QuestState, in DEVICES order.

    tracked is the device's own flag AND a usable pose, so callers can skip on it alone.
    """
    raw = [('head', state.head_tracked, state.head)]
    raw += [(d, controller_state(state, d).tracked, controller_state(state, d).pose)
            for d in CONTROLLERS]
    return [(d, bool(tracked) and pose_is_valid(pose), pose) for d, tracked, pose in raw]
