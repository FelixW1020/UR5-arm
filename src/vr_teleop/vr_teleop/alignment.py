"""Does the tool move the way a reference frame moves? (pure numpy, no ROS)

Compares motions, not poses: the displacement of the reference (a controller) and of the tool
since an anchor, both in base_link. Fixed offsets (where the hand holds the controller, a clamp
offset) cancel out, so what remains is the rotation between the two motions, i.e. the rotation
error of whatever links the two frames, in practice the registration base_link -> vr_world. For
horizontal strokes and a yaw_only registration, direction_error_deg is exactly its heading error.

rotation_error_deg compares the rotation increments of the two frames. It catches orientation
mapping mistakes (wrong axis or sign), but a pure heading error of the registration barely shows
in it (a yaw seen through a yaw is the same yaw), so use the direction error for that.
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ur5_common.transforms import rotation_angle


def angle_between_deg(a, b):
    """Angle (deg, 0..180) between two vectors; nan if either is zero."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if np.linalg.norm(a) < 1e-12 or np.linalg.norm(b) < 1e-12:
        return float('nan')
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b))))


def rotation_error_deg(dR_ref, dR_tool):
    """Angle (deg) of dR_tool @ dR_ref.T: how far the tool's rotation increment is off."""
    return float(np.degrees(rotation_angle(np.asarray(dR_tool, dtype=float)[:3, :3]
                                           @ np.asarray(dR_ref, dtype=float)[:3, :3].T)))


class RunningStats:

    def __init__(self):
        self.reset()

    def reset(self):
        self.count, self.last, self.max, self._sum = 0, float('nan'), float('nan'), 0.0

    def add(self, x):
        x = float(x)
        self.count += 1
        self._sum += x
        self.last = x
        self.max = x if self.count == 1 else max(self.max, x)

    @property
    def mean(self):
        return self._sum / self.count if self.count else float('nan')

    def __str__(self):
        return f'{self.last:.1f} deg (mean {self.mean:.1f}, max {self.max:.1f}, n={self.count})'


@dataclass
class Measurement:
    direction_error_deg: Optional[float] = None
    reference_motion: Optional[np.ndarray] = None   # displacement since the anchor, base frame
    tool_motion: Optional[np.ndarray] = None
    rotation_error_deg: Optional[float] = None


class AlignmentTracker:
    """Feed (T_base_reference, T_base_tool) samples; measures each stroke of min_motion_m.

    Direction: once BOTH frames have moved more than min_motion_m since the anchor, the angle
    between the two displacements is measured and both positions are re-anchored.
    Rotation: once the reference has turned more than min_rotation_deg since its anchor,
    angle(dR_tool @ dR_ref.T) with dR = R_now @ R_anchor.T is measured and re-anchored.
    """

    def __init__(self, min_motion_m=0.03, min_rotation_deg=10.0):
        self.min_motion_m = float(min_motion_m)
        self.min_rotation_deg = float(min_rotation_deg)
        self.direction = RunningStats()
        self.rotation = RunningStats()
        self.reset_anchor()

    def reset_anchor(self):
        """Start a new stroke from the next sample (keeps the statistics)."""
        self._p_anchor = None       # (p_ref, p_tool)
        self._R_anchor = None       # (R_ref, R_tool)

    def reset(self):
        self.reset_anchor()
        self.direction.reset()
        self.rotation.reset()

    def update(self, T_base_ref, T_base_tool):
        """Returns a Measurement if this sample completed a stroke or a turn, else None."""
        T_ref = np.asarray(T_base_ref, dtype=float)
        T_tool = np.asarray(T_base_tool, dtype=float)
        if self._p_anchor is None:
            self._p_anchor = (T_ref[:3, 3].copy(), T_tool[:3, 3].copy())
        if self._R_anchor is None:
            self._R_anchor = (T_ref[:3, :3].copy(), T_tool[:3, :3].copy())
        m = Measurement()

        d_ref = T_ref[:3, 3] - self._p_anchor[0]
        d_tool = T_tool[:3, 3] - self._p_anchor[1]
        if min(np.linalg.norm(d_ref), np.linalg.norm(d_tool)) > self.min_motion_m:
            m.direction_error_deg = angle_between_deg(d_ref, d_tool)
            m.reference_motion, m.tool_motion = d_ref, d_tool
            self.direction.add(m.direction_error_deg)
            self._p_anchor = (T_ref[:3, 3].copy(), T_tool[:3, 3].copy())

        dR_ref = T_ref[:3, :3] @ self._R_anchor[0].T
        if np.degrees(rotation_angle(dR_ref)) > self.min_rotation_deg:
            m.rotation_error_deg = rotation_error_deg(dR_ref, T_tool[:3, :3] @ self._R_anchor[1].T)
            self.rotation.add(m.rotation_error_deg)
            self._R_anchor = (T_ref[:3, :3].copy(), T_tool[:3, :3].copy())

        if m.direction_error_deg is None and m.rotation_error_deg is None:
            return None
        return m
