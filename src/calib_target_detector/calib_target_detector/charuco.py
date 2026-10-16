"""ChArUco board detection + pose estimation (pure OpenCV, no ROS).

Works with both the legacy aruco API (OpenCV < 4.7, e.g. Ubuntu 22.04's python3-opencv)
and the new one (OpenCV >= 4.7, e.g. the pip wheel pulled in by ultralytics).
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class BoardPose:
    rvec: np.ndarray            # Rodrigues rotation, board -> camera
    tvec: np.ndarray            # translation (m), board origin in camera frame
    num_corners: int
    reprojection_error_px: float
    charuco_corners: np.ndarray
    charuco_ids: np.ndarray


class CharucoBoardDetector:

    def __init__(self, squares_x, squares_y, square_length, marker_length,
                 dictionary='DICT_5X5_100', min_corners=6):
        if not hasattr(cv2.aruco, dictionary):
            raise ValueError(f'Unknown ArUco dictionary {dictionary!r}')
        self.min_corners = min_corners
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self._new_api = hasattr(cv2.aruco, 'CharucoDetector')
        if self._new_api:
            self.board = cv2.aruco.CharucoBoard(
                (squares_x, squares_y), square_length, marker_length, aruco_dict)
            self._detector = cv2.aruco.CharucoDetector(self.board)
            self._object_corners = np.asarray(self.board.getChessboardCorners(), dtype=np.float64)
        else:
            self.board = cv2.aruco.CharucoBoard_create(
                squares_x, squares_y, square_length, marker_length, aruco_dict)
            self._aruco_dict = aruco_dict
            self._params = cv2.aruco.DetectorParameters_create()
            self._object_corners = np.asarray(self.board.chessboardCorners, dtype=np.float64)

    def _detect_corners(self, gray):
        if self._new_api:
            corners, ids, _, _ = self._detector.detectBoard(gray)
            return corners, ids
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._params)
        if marker_ids is None or len(marker_ids) == 0:
            return None, None
        _, corners, ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, self.board)
        return corners, ids

    def detect(self, gray, camera_matrix, dist_coeffs):
        """Return a BoardPose, or None if the board is not (reliably) visible."""
        corners, ids = self._detect_corners(gray)
        if ids is None or corners is None or len(ids) < self.min_corners:
            return None
        ids = ids.reshape(-1)
        img_pts = corners.reshape(-1, 2).astype(np.float64)
        obj_pts = self._object_corners[ids].reshape(-1, 3)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
        projected, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
        err = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - img_pts) ** 2, axis=1))))
        return BoardPose(rvec.reshape(3), tvec.reshape(3), len(ids), err, corners, ids)

    def render(self, width_px, height_px, margin_px=20):
        """Printable board image."""
        size = (width_px, height_px)
        if self._new_api:
            return self.board.generateImage(size, marginSize=margin_px)
        return self.board.draw(size, marginSize=margin_px)
