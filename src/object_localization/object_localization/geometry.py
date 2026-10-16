"""Depth sampling and pinhole deprojection (pure numpy, no ROS)."""
import numpy as np


def box_depth(depth_m, cx, cy, w, h, roi_fraction=0.4, percentile=50.0,
              min_depth=0.1, max_depth=5.0, min_pixels=10):
    """Robust depth (m) of the object inside a bounding box.

    Samples a central window (roi_fraction of the box) to avoid background pixels at
    the box edges, drops invalid readings (0 / NaN / out of range), and returns the
    given percentile. A lower percentile (e.g. 30) favours the nearest surface, which
    helps when the box centre looks into a cup's opening.
    """
    rows, cols = depth_m.shape[:2]
    hw = max(1, int(round(w * roi_fraction / 2)))
    hh = max(1, int(round(h * roi_fraction / 2)))
    u, v = int(round(cx)), int(round(cy))
    roi = depth_m[max(0, v - hh):min(rows, v + hh + 1), max(0, u - hw):min(cols, u + hw + 1)]
    valid = roi[np.isfinite(roi) & (roi > min_depth) & (roi < max_depth)]
    if valid.size < min_pixels:
        return None
    return float(np.percentile(valid, percentile))


def deproject(u, v, z, K):
    """Pixel (u, v) at depth z (m) -> 3D point in the camera optical frame."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])


def depth_to_metres(depth, encoding):
    """Convert a decoded depth image to float metres (RealSense publishes 16UC1 mm)."""
    if encoding in ('16UC1', 'mono16'):
        return depth.astype(np.float32) * 0.001
    if encoding == '32FC1':
        return depth.astype(np.float32)
    raise ValueError(f'Unsupported depth encoding {encoding!r}')
