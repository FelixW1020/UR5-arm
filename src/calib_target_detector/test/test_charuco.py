"""Render a board, project it into a synthetic camera, and check the recovered pose."""
import cv2
import numpy as np

from calib_target_detector.charuco import CharucoBoardDetector

PX_PER_M = 5000.0  # rendered board resolution


def test_detects_synthetic_board_pose():
    det = CharucoBoardDetector(7, 5, 0.04, 0.03)
    board_img = det.render(int(7 * 0.04 * PX_PER_M), int(5 * 0.04 * PX_PER_M), margin_px=0)

    K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
    D = np.zeros(5)
    rvec_true = np.array([0.25, -0.2, 0.05])
    tvec_true = np.array([-0.12, -0.08, 0.6])

    # Homography from the board plane (metres; image y-axis points down the board
    # in OpenCV's board convention) to the camera image.
    h, w = board_img.shape[:2]
    board_pts = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float64)
    obj = np.column_stack([board_pts / PX_PER_M, np.zeros(4)])
    img_pts, _ = cv2.projectPoints(obj, rvec_true, tvec_true, K, D)
    H = cv2.getPerspectiveTransform(board_pts.astype(np.float32),
                                    img_pts.reshape(-1, 2).astype(np.float32))
    scene = cv2.warpPerspective(board_img, H, (1280, 720), borderValue=255)

    pose = det.detect(scene, K, D)
    assert pose is not None
    assert pose.reprojection_error_px < 1.0

    # OpenCV < 4.6 puts the board origin at a different corner (y flipped), so compare
    # convention-independent quantities: the board centre and the board plane normal.
    def centre_and_normal(rvec, tvec):
        R = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))[0]
        centre = R @ np.array([7 * 0.04 / 2, 5 * 0.04 / 2, 0.0]) + tvec
        return centre, R[:, 2]

    c_true, n_true = centre_and_normal(rvec_true, tvec_true)
    c_est, n_est = centre_and_normal(pose.rvec, pose.tvec)
    assert np.linalg.norm(c_est - c_true) < 0.005
    assert abs(np.dot(n_est, n_true)) > np.cos(np.radians(1.0))
