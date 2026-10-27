"""Render printed ArUco tags into a pinhole camera image (pure numpy/OpenCV, no ROS).

Stands in for the Quest's passthrough camera so the whole tag-registration path (JPEG ->
detection -> solvePnP -> fusion -> registration) runs on pixels, not on shortcut poses. Each
tag, with a one-cell white quiet zone, is warped by the homography of its projected outer
corners and alpha-blended through a warped mask, which gives anti-aliased edges and thus
sub-pixel corners comparable to a real camera.
"""
import functools

import cv2
import numpy as np

from aruco_tag_mapper.markers import ArucoMarkerDetector

MIN_DEPTH_M = 0.05          # tags with a corner closer than this (or behind) are not drawn


@functools.lru_cache(maxsize=4)
def _background(width, height):
    """Mid-grey with smooth low-contrast blotches: a table top, not a flat synthetic field."""
    rng = np.random.default_rng(12345)
    coarse = rng.normal(0.0, 10.0, (height // 40 + 2, width // 40 + 2)).astype(np.float32)
    bg = 128.0 + cv2.resize(coarse, (width, height), interpolation=cv2.INTER_CUBIC)
    bg.setflags(write=False)
    return bg


@functools.lru_cache(maxsize=4)
def _detector(dictionary):
    return ArucoMarkerDetector(dictionary, 1.0)


@functools.lru_cache(maxsize=4)
def _marker_cells(dictionary):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    return int(d.markerSize) + 2            # data bits + the black border


def render_tags(K, width, height, tags, marker_length, dictionary='DICT_4X4_50', noise_std=0.0,
                rng=None):
    """uint8 grey image of the tags seen by an ideal pinhole camera (no distortion).

    tags: {id: T_optical_tag}, tag frame as aruco_tag_mapper.markers (x right, y up, z out of
    the printed face), camera frame OpenCV optical. marker_length is the black square's side.
    Tags with a corner closer than MIN_DEPTH_M or behind the camera, and tags seen from the
    back of the paper, are culled. noise_std adds Gaussian pixel noise (grey levels).
    """
    K = np.asarray(K, dtype=float)
    detector = _detector(dictionary)            # ValueError for an unknown dictionary
    img = _background(int(width), int(height)).copy()
    cells = _marker_cells(dictionary)
    half = 0.5 * marker_length * (cells + 2) / cells        # quiet zone = one cell per side
    outer = np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0],
                      [-half, -half, 0.0]])
    for tag_id, T in tags.items():
        T = np.asarray(T, dtype=float)
        pts = outer @ T[:3, :3].T + T[:3, 3]
        if np.any(pts[:, 2] < MIN_DEPTH_M):
            continue
        if np.dot(T[:3, 2], T[:3, 3]) >= 0.0:           # face normal points away from camera
            continue
        uv = (pts @ K.T)[:, :2] / pts[:, 2:3]
        x0, y0 = np.floor(uv.min(axis=0)).astype(int) - 1
        x1, y1 = np.ceil(uv.max(axis=0)).astype(int) + 2
        x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, width), min(y1, height)
        if x1 <= x0 or y1 <= y0:
            continue
        # Warp at s x the resolution and box-filter down (INTER_AREA): a plain bilinear warp
        # aliases on small, foreshortened tags and biases the detected corners.
        edge = max(np.linalg.norm(uv - np.roll(uv, 1, axis=0), axis=1).max(), 1.0)
        s = int(np.clip(np.ceil(400.0 / edge), 1, 4))
        cell_px = int(np.clip(np.ceil(s * edge / (cells + 2)), 2, 64))
        side = cell_px * cells
        tag_img = np.full((side + 2 * cell_px,) * 2, 255, np.float32)
        tag_img[cell_px:cell_px + side, cell_px:cell_px + side] = \
            detector.render(int(tag_id), side)
        n = tag_img.shape[0]
        # Pixel centres are integers, so an image's outer edges are at -0.5 and n - 0.5, and
        # pixel u of the output is centred on s * u + (s - 1) / 2 of the supersampled one.
        src = np.float32([[-0.5, -0.5], [n - 0.5, -0.5], [n - 0.5, n - 0.5], [-0.5, n - 0.5]])
        dst = (s * (uv - [x0, y0]) + 0.5 * (s - 1)).astype(np.float32)
        H = cv2.getPerspectiveTransform(src, dst)
        size, big = (x1 - x0, y1 - y0), (s * (x1 - x0), s * (y1 - y0))
        warped = cv2.warpPerspective(tag_img, H, big, flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_REPLICATE)   # white past the edge
        alpha = cv2.warpPerspective(np.ones((n, n), np.float32), H, big, flags=cv2.INTER_LINEAR)
        if s > 1:
            warped = cv2.resize(warped * alpha, size, interpolation=cv2.INTER_AREA)
            alpha = cv2.resize(alpha, size, interpolation=cv2.INTER_AREA)
        else:
            warped = warped * alpha
        roi = img[y0:y1, x0:x1]
        roi[:] = roi * (1.0 - alpha) + warped
    if noise_std > 0.0:
        rng = rng if rng is not None else np.random.default_rng()
        img += rng.standard_normal(img.shape, dtype=np.float32) * np.float32(noise_std)
    return np.clip(np.rint(img), 0, 255).astype(np.uint8)
