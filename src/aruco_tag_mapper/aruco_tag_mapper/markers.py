"""Single ArUco marker detection + pose (pure OpenCV, no ROS).

Marker frame: origin at the marker centre, x right, y up, z out of the printed face
(toward the viewer). Works with OpenCV < 4.7 (legacy API) and >= 4.7.
"""
from dataclasses import dataclass

import cv2
import numpy as np

from ur5_common.transforms import make_transform


@dataclass
class MarkerObservation:
    id: int
    T_cam_marker: np.ndarray
    reprojection_error_px: float
    corners: np.ndarray        # 4x2 pixel corners (TL, TR, BR, BL)


class ArucoMarkerDetector:

    def __init__(self, dictionary='DICT_4X4_50', marker_length=0.10, allowed_ids=None):
        if not hasattr(cv2.aruco, dictionary):
            raise ValueError(f'Unknown ArUco dictionary {dictionary!r}')
        self._dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self._allowed = set(allowed_ids) if allowed_ids else None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detector = cv2.aruco.ArucoDetector(self._dict, params)
        else:
            self._detector = None
            self._params = cv2.aruco.DetectorParameters_create()
            self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        h = marker_length / 2.0
        # Corner order required by SOLVEPNP_IPPE_SQUARE, matching ArUco's TL, TR, BR, BL.
        self._object_points = np.array(
            [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)

    def detect(self, gray, camera_matrix, dist_coeffs):
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is None:
            return []
        out = []
        for c, marker_id in zip(corners, ids.reshape(-1)):
            if self._allowed is not None and int(marker_id) not in self._allowed:
                continue
            img = c.reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(self._object_points, img, camera_matrix, dist_coeffs,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(self._object_points, rvec, tvec, camera_matrix, dist_coeffs)
            err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img) ** 2, axis=1))))
            out.append(MarkerObservation(int(marker_id),
                                         make_transform(cv2.Rodrigues(rvec)[0], tvec), err, img))
        return out

    def render(self, marker_id, side_px):
        if hasattr(cv2.aruco, 'generateImageMarker'):
            return cv2.aruco.generateImageMarker(self._dict, marker_id, side_px)
        return cv2.aruco.drawMarker(self._dict, marker_id, side_px)
