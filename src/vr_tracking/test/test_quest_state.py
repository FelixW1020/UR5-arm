import math
import re
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from vr_tracking.quest_state import (CONTROLLERS, DEVICES, JOY_AXES, JOY_BUTTONS,
                                     controller_state, device_states, joy_arrays, pose_is_valid)

MSG_DIR = Path(__file__).resolve().parents[2] / 'vr_interfaces' / 'msg'


def _pose(x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0)):
    return NS(position=NS(x=x, y=y, z=z), orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))


def _controller(tracked=True, pose=None, **inputs):
    fields = dict(index_trigger=0.0, hand_trigger=0.0, thumbstick_x=0.0, thumbstick_y=0.0,
                  button_primary=False, button_secondary=False, thumbstick_pressed=False,
                  menu=False)
    fields.update(inputs)
    return NS(tracked=tracked, pose=pose or _pose(), **fields)


def _state(head_tracked=True, left=None, right=None):
    return NS(header=NS(frame_id='vr_world'), head_tracked=head_tracked, head=_pose(z=1.6),
              left=left or _controller(), right=right or _controller())


def _msg_fields(name):
    path = MSG_DIR / name
    if not path.exists():
        pytest.skip(f'{path} not found (tests run outside the source tree)')
    fields = []
    for line in path.read_text().splitlines():
        m = re.match(r'\s*[\w/\[\]]+\s+(\w+)', line.split('#')[0])
        if m:
            fields.append(m.group(1))
    return fields


def test_joy_layout_is_the_documented_one():
    # The contract in DESIGN.md / the node docstring; consumers index by position.
    assert JOY_AXES == ('thumbstick_x', 'thumbstick_y', 'index_trigger', 'hand_trigger')
    assert JOY_BUTTONS == ('button_primary', 'button_secondary', 'thumbstick_pressed', 'menu')
    assert DEVICES == ('head', 'left_controller', 'right_controller')
    assert CONTROLLERS == DEVICES[1:]


def test_names_match_the_messages():
    assert set(JOY_AXES + JOY_BUTTONS + ('tracked', 'pose')) <= set(
        _msg_fields('ControllerState.msg'))
    assert {'header', 'head_tracked', 'head', 'left', 'right'} <= set(
        _msg_fields('QuestState.msg'))


def test_joy_arrays_order_and_types():
    c = _controller(thumbstick_x=-0.5, thumbstick_y=1.0, index_trigger=0.25, hand_trigger=0.75,
                    button_primary=True, button_secondary=False, thumbstick_pressed=True,
                    menu=False)
    axes, buttons = joy_arrays(c)
    assert axes == [-0.5, 1.0, 0.25, 0.75]
    assert buttons == [1, 0, 1, 0]
    assert all(type(a) is float for a in axes) and all(type(b) is int for b in buttons)
    hand = JOY_AXES.index('hand_trigger')
    assert axes[hand] == 0.75     # vr_teleop's clutch reads axes[3]


def test_controller_state_maps_device_to_field():
    left, right = _controller(menu=True), _controller()
    s = _state(left=left, right=right)
    assert controller_state(s, 'left_controller') is left
    assert controller_state(s, 'right_controller') is right
    with pytest.raises(KeyError):
        controller_state(s, 'head')


def test_device_states_order_and_tracking():
    right_pose = _pose(0.3, -0.2, 1.1, (0.0, 0.0, math.sin(0.4), math.cos(0.4)))
    s = _state(head_tracked=True, left=_controller(tracked=False),
               right=_controller(tracked=True, pose=right_pose))
    out = device_states(s)
    assert [d for d, _, _ in out] == list(DEVICES)
    assert [t for _, t, _ in out] == [True, False, True]
    assert out[2][2] is right_pose and out[0][2] is s.head


def test_unusable_pose_counts_as_untracked():
    zero_q = _pose(q=(0.0, 0.0, 0.0, 0.0))
    nan_p = _pose(x=float('nan'))
    assert not pose_is_valid(zero_q) and not pose_is_valid(nan_p)
    assert pose_is_valid(_pose(q=(0.0, 0.0, 0.0, 2.0)))     # unnormalised: normalised later
    s = _state(head_tracked=True, left=_controller(pose=zero_q), right=_controller(pose=nan_p))
    s.head = _pose(z=float('inf'))
    assert [t for _, t, _ in device_states(s)] == [False, False, False]
