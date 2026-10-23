import cv2
import numpy as np
import pytest

from aruco_tag_mapper.markers import ArucoMarkerDetector
from ur5_common.transforms import T_LINK_OPTICAL, invert, make_transform, rotation_angle
from vr_sim.render import render_tags
from vr_sim.scene import look_at, pose_from_xyz_rpy_deg, rpy_to_matrix

W, H = 1280, 960
K = np.array([[800.0, 0, (W - 1) / 2], [0, 800.0, (H - 1) / 2], [0, 0, 1]])
L = 0.10


def _facing(xyz, tilt_deg=(0.0, 0.0, 0.0)):
    """T_optical_tag of a tag facing the camera (tag z toward it), tilted by rpy degrees."""
    R = rpy_to_matrix(*np.radians(tilt_deg)) @ np.diag([1.0, -1.0, -1.0])
    return make_transform(R, xyz)


def _jpeg_round_trip(gray, quality=85):
    ok, buf = cv2.imencode('.jpg', cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                           [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)


def test_render_detect_round_trip():
    tags = {0: _facing([0.0, 0.0, 0.8], (20, -15, 30)),
            1: _facing([0.25, -0.12, 1.0], (-35, 10, -60)),
            2: _facing([-0.3, 0.15, 0.9], (40, 25, 100))}
    rng = np.random.default_rng(0)
    gray = _jpeg_round_trip(render_tags(K, W, H, tags, L, noise_std=2.0, rng=rng))
    assert gray.shape == (H, W) and gray.dtype == np.uint8
    obs = {o.id: o for o in ArucoMarkerDetector('DICT_4X4_50', L).detect(gray, K, np.zeros(5))}
    assert sorted(obs) == [0, 1, 2]
    for i, T in tags.items():
        T_est = obs[i].T_cam_marker
        assert np.linalg.norm(T_est[:3, 3] - T[:3, 3]) < 0.004
        assert np.degrees(rotation_angle(T_est[:3, :3].T @ T[:3, :3])) < 1.5
        assert obs[i].reprojection_error_px < 0.5


def test_tag_lying_on_table_seen_obliquely():
    """quest_sim's geometry: camera ~1 m away looking down ~40 degrees at face-up tags."""
    T_base_optical = look_at([0.45, -0.85, 0.65], [0.47, -0.07, 0.0]) @ T_LINK_OPTICAL
    tags_base = {3: pose_from_xyz_rpy_deg([0.45, 0.0, 0.0], [0, 0, 30]),
                 4: pose_from_xyz_rpy_deg([0.6, -0.2, 0.0], [0, 0, -45])}
    tags = {i: invert(T_base_optical) @ T for i, T in tags_base.items()}
    gray = _jpeg_round_trip(render_tags(K, W, H, tags, L, noise_std=2.0,
                                        rng=np.random.default_rng(2)))
    obs = {o.id: o for o in ArucoMarkerDetector('DICT_4X4_50', L).detect(gray, K, np.zeros(5))}
    assert sorted(obs) == [3, 4]
    for i, T in tags.items():
        T_base_est = T_base_optical @ obs[i].T_cam_marker
        assert np.linalg.norm(T_base_est[:3, 3] - tags_base[i][:3, 3]) < 0.006
        assert np.degrees(rotation_angle(T_base_est[:3, :3].T @ tags_base[i][:3, :3])) < 2.0


def test_culling():
    det = ArucoMarkerDetector('DICT_4X4_50', L)
    blank = render_tags(K, W, H, {}, L)
    behind = _facing([0.0, 0.0, -0.8])
    too_close = _facing([0.0, 0.0, 0.04])
    straddling = _facing([0.0, 0.0, 0.07], (60, 0, 0))     # one edge closer than 5 cm
    backside = make_transform(np.eye(3), [0.0, 0.0, 0.8])  # tag z away from the camera
    outside = _facing([5.0, 0.0, 0.8])
    for T in (behind, too_close, straddling, backside, outside):
        img = render_tags(K, W, H, {4: T}, L)
        assert np.array_equal(img, blank)
        assert det.detect(img, K, np.zeros(5)) == []


def test_background_and_determinism():
    img = render_tags(K, W, H, {5: _facing([0.0, 0.0, 0.8])}, L)
    assert np.array_equal(img, render_tags(K, W, H, {5: _facing([0.0, 0.0, 0.8])}, L))
    blank = render_tags(K, W, H, {}, L)
    assert 100 < blank.mean() < 156 and blank.std() > 1.0       # mid-grey, lightly textured
    noisy = render_tags(K, W, H, {}, L, noise_std=3.0, rng=np.random.default_rng(1))
    assert 2.0 < np.std(noisy.astype(float) - blank) < 4.0


def test_unknown_dictionary():
    with pytest.raises(ValueError):
        render_tags(K, W, H, {}, L, dictionary='DICT_NOPE')
