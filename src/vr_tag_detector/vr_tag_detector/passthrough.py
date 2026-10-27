"""Locate ArUco tags in vr_world from Quest 3 passthrough frames (pure numpy/OpenCV, no ROS).

The Quest app sends each passthrough image together with the camera's pose in vr_world at
capture time, so a tag posed by solvePnP maps straight into vr_world:

    T_vr_tag = T_vr_link @ T_LINK_OPTICAL @ T_optical_tag

Unity reports the camera like any other object ("link" convention: x = viewing direction,
y = image-left, z = image-up) while solvePnP works in the OpenCV optical frame, hence
T_LINK_OPTICAL. One frame is not enough: the images are low resolution and the headset pose
jitters by a few mm, so sightings are fused with aruco_tag_mapper's TagMap (median position,
mean rotation), exactly like the robot cameras' tag map. The fused tags go to vr_registration.
"""
from dataclasses import dataclass

import cv2
import numpy as np

from ur5_common.transforms import T_LINK_OPTICAL, matrix_to_quat

SOURCE = 'quest'        # TagMap source name of every passthrough sighting


@dataclass
class Sighting:
    id: int
    T_vr_tag: np.ndarray
    T_optical_tag: np.ndarray       # solvePnP result, kept to draw the tag axes
    reprojection_error_px: float
    distance_m: float               # camera -> tag centre
    corners: np.ndarray             # 4x2 pixel corners (TL, TR, BR, BL)
    rejected: str = ''              # why it must not be fused ('' = accepted)


def decode_gray(data):
    """CompressedImage.data (JPEG or PNG bytes, any buffer or int sequence) -> 8-bit gray."""
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE) if buf.size else None
    if img is None:
        raise ValueError(f'cannot decode a {buf.size}-byte image (expected JPEG or PNG)')
    return img


def intrinsics(k, d):
    """CameraInfo k (row-major 3x3) and d -> (K, D). Empty d means no distortion."""
    K = np.array(k, dtype=np.float64).reshape(3, 3)
    if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
        raise ValueError('camera_info.k is not set (fx and fy must be > 0)')
    D = np.array(d, dtype=np.float64) if len(d) else np.zeros(5)
    return K, D


def detect_tags_in_vr(gray, K, D, T_vr_link, detector, max_reprojection_error_px=2.0,
                      max_distance_m=2.5):
    """Every tag in the image, posed in vr_world.

    detector is an aruco_tag_mapper.markers.ArucoMarkerDetector. Sightings over a limit are
    returned too, with `rejected` saying why, so the debug image can show it; add_sightings
    skips them.
    """
    T_vr_optical = np.asarray(T_vr_link, dtype=float) @ T_LINK_OPTICAL
    out = []
    for o in detector.detect(gray, K, D):
        distance = float(np.linalg.norm(o.T_cam_marker[:3, 3]))
        if o.reprojection_error_px > max_reprojection_error_px:
            why = f'reproj {o.reprojection_error_px:.1f} > {max_reprojection_error_px:.1f} px'
        elif distance > max_distance_m:
            why = f'{distance:.2f} m > {max_distance_m:.2f} m'
        else:
            why = ''
        out.append(Sighting(o.id, T_vr_optical @ o.T_cam_marker, o.T_cam_marker,
                            o.reprojection_error_px, distance, o.corners, why))
    return out


def add_sightings(tag_map, sightings, source=SOURCE):
    """Add the accepted sightings to an aruco_tag_mapper TagMap; returns how many."""
    accepted = [s for s in sightings if not s.rejected]
    for s in accepted:
        tag_map.add(s.id, s.T_vr_tag, source)
    return len(accepted)


def to_observations(estimates, use_orientation=True):
    """TagMap.estimate() -> [(id, xyz, quaternion xyzw or None)] sorted by id.

    The TagMap here holds vr_world poses (its field is named T_base_tag because the robot
    cameras' map lives in base_link). Without orientation vr_registration fits positions only
    and needs >= 2 tags; with it one tag is enough.
    """
    return [(int(i), e.T_base_tag[:3, 3].copy(),
             matrix_to_quat(e.T_base_tag) if use_orientation else None)
            for i, e in sorted(estimates.items())]


def summarize(estimates, max_spread_m, residuals_m=None):
    """One report line per fused tag, and the ids whose sighting spread exceeds max_spread_m."""
    residuals_m = residuals_m or {}
    lines, noisy = [], []
    for i, e in sorted(estimates.items()):
        line = f'tag {i}: n={e.num_observations}, spread {e.position_std_m * 1000:.1f} mm'
        if i in residuals_m:
            line += f', residual {residuals_m[i] * 1000:.1f} mm'
        if e.position_std_m > max_spread_m:
            noisy.append(i)
            line += f' (> {max_spread_m * 1000:.0f} mm)'
        lines.append(line)
    return lines, noisy


def tag_map_mismatch(tag_map, marker_length, dictionary):
    """Why the robot's tag map (its YAML as a dict) describes other tags than this detector.

    Returns '' if it doesn't. A wrong marker_length scales every tag's distance from the
    headset, so a one-tag registration comes out centimetres off with a residual of 0.
    """
    problems = []
    length = tag_map.get('marker_length')
    if length is not None and abs(float(length) - float(marker_length)) > 1e-4:
        problems.append(f'marker_length is {marker_length} m here but {length} m in the tag map')
    name = tag_map.get('dictionary')
    if name and name != dictionary:
        problems.append(f'dictionary is {dictionary} here but {name} in the tag map')
    return '; '.join(problems)


def draw_debug(gray, sightings, K, D, axis_length):
    """BGR image: accepted tags in green with their axes, rejected ones in red with the reason."""
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for s in sightings:
        color = (0, 0, 255) if s.rejected else (0, 200, 0)
        pts = np.round(s.corners).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], True, color, 2)
        if not s.rejected:
            rvec = cv2.Rodrigues(s.T_optical_tag[:3, :3])[0]
            cv2.drawFrameAxes(img, K, D, rvec, s.T_optical_tag[:3, 3].reshape(3, 1), axis_length)
        text = f'id {s.id} {s.distance_m:.2f}m {s.reprojection_error_px:.1f}px'
        if s.rejected:
            text += f' [{s.rejected}]'
        x, y = int(pts[:, 0, 0].min()), int(pts[:, 0, 1].min())
        cv2.putText(img, text, (x, max(y - 8, 20)), font, 0.6, color, 2)
    n = sum(1 for s in sightings if not s.rejected)
    cv2.putText(img, f'{n} tag(s) accepted', (10, img.shape[0] - 15), font, 0.7,
                (0, 200, 0) if n else (0, 0, 255), 2)
    return img
