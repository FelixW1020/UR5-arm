import cv2
import numpy as np

from aruco_tag_mapper.markers import ArucoMarkerDetector
from ur5_common.transforms import invert

K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
D = np.zeros(5)
PX_PER_M = 4000.0


def _facing_camera(tilt):
    """rvec for a tag facing the camera (marker z toward camera = Rx(180deg)), plus a tilt."""
    R = cv2.Rodrigues(np.asarray(tilt, dtype=float))[0] @ np.diag([1.0, -1.0, -1.0])
    return cv2.Rodrigues(R)[0].reshape(3)


def _scene(det, tag_id, rvec, tvec, length=0.10):
    side = int(length * PX_PER_M)
    q = side // 4  # quiet zone
    tag = np.full((side + 2 * q, side + 2 * q), 255, np.uint8)
    tag[q:q + side, q:q + side] = det.render(tag_id, side)
    s = (side + 2 * q) / PX_PER_M / 2      # half-extent of the image incl. quiet zone
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
    img_pts, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    n = tag.shape[0]
    src = np.float32([[0, 0], [n, 0], [n, n], [0, n]])
    H = cv2.getPerspectiveTransform(src, img_pts.reshape(-1, 2).astype(np.float32))
    return cv2.warpPerspective(tag, H, (1280, 720), borderValue=255)


def test_marker_pose():
    det = ArucoMarkerDetector('DICT_4X4_50', 0.10)
    rvec, tvec = _facing_camera([0.3, -0.2, 0.4]), np.array([0.05, -0.03, 0.7])
    obs = det.detect(_scene(det, 7, rvec, tvec), K, D)
    assert [o.id for o in obs] == [7]
    T = obs[0].T_cam_marker
    assert np.linalg.norm(T[:3, 3] - tvec) < 0.003
    R_err = T[:3, :3].T @ cv2.Rodrigues(rvec)[0]
    assert np.degrees(np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))) < 2.0
    assert np.allclose(invert(T) @ T, np.eye(4))


def test_allowed_ids_filter():
    det = ArucoMarkerDetector('DICT_4X4_50', 0.10, allowed_ids=[1, 2])
    img = _scene(det, 7, _facing_camera([0.1, 0.1, 0.0]), np.array([0.0, 0.0, 0.6]))
    assert [o.id for o in ArucoMarkerDetector('DICT_4X4_50', 0.10).detect(img, K, D)] == [7]
    assert det.detect(img, K, D) == []
