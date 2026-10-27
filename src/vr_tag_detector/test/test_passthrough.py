import array

import cv2
import numpy as np
import pytest

from aruco_tag_mapper.markers import ArucoMarkerDetector
from aruco_tag_mapper.tag_map import TagMap
from ur5_common.transforms import (T_LINK_OPTICAL, invert, make_transform, quat_to_matrix,
                                   rotation_angle)
from vr_tag_detector.passthrough import (Sighting, add_sightings, decode_gray,
                                         detect_tags_in_vr, draw_debug, intrinsics, summarize,
                                         tag_map_mismatch, to_observations)

W, H = 1280, 960                       # Quest 3 passthrough-like image
K = np.array([[880.0, 0, 640], [0, 880.0, 480], [0, 0, 1]])
D = np.zeros(5)
LENGTH = 0.10
DET = ArucoMarkerDetector('DICT_4X4_50', LENGTH)


def _rz(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _look_at(eye, target):
    """Camera pose in vr_world, link convention: x toward the target, y image-left, z image-up."""
    eye = np.asarray(eye, dtype=float)
    x = np.asarray(target, dtype=float) - eye
    x /= np.linalg.norm(x)
    y = np.cross([0.0, 0.0, 1.0], x)
    y /= np.linalg.norm(y)
    return make_transform(np.column_stack([x, y, np.cross(x, y)]), eye)


def _project(T_vr_link, p_vr):
    """Pinhole projection derived by hand in the link frame (no T_LINK_OPTICAL, no OpenCV):
    depth = x, image right = -y, image down = -z."""
    x, y, z = (invert(T_vr_link) @ np.append(p_vr, 1.0))[:3]
    return np.array([K[0, 2] - K[0, 0] * y / x, K[1, 2] - K[1, 1] * z / x])


def _render(tags, T_vr_link, px_per_m=4000.0, ss=2):
    """Gray image of printed tags {id: T_vr_tag} (with a white quiet zone) seen by the camera.
    Rendered at ss x the resolution and area-averaged down, like a real sensor's pixels."""
    img = np.full((H * ss, W * ss), 255, np.uint8)
    side = int(LENGTH * px_per_m)
    q = side // 4
    n = side + 2 * q
    s = n / px_per_m / 2
    # Pixel centres are integers (OpenCV), so the sheet's outer edges are at -0.5 and n - 0.5.
    src = np.float32([[-0.5, -0.5], [n - 0.5, -0.5], [n - 0.5, n - 0.5], [-0.5, n - 0.5]])
    for tag_id, T in tags.items():
        sheet = np.full((n, n), 255, np.uint8)
        sheet[q:q + side, q:q + side] = DET.render(tag_id, side)
        corners = [(T @ [cx, cy, 0.0, 1.0])[:3] for cx, cy in [(-s, s), (s, s), (s, -s), (-s, -s)]]
        dst = np.float32([(_project(T_vr_link, c) + 0.5) * ss - 0.5 for c in corners])
        warped = cv2.warpPerspective(sheet, cv2.getPerspectiveTransform(src, dst),
                                     (W * ss, H * ss), borderValue=255)
        img = np.minimum(img, warped)
    return cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)


def _jpeg(img, quality=90):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return buf.tobytes()


def _errors(T, T_true, eye):
    """(lateral, along-ray position error, rotation error in deg). ArUco corners carry a small
    sub-pixel bias that shows up as depth along the viewing ray (~0.5% of the distance, the
    same as aruco_tag_mapper's own test); a frame-convention bug would show up laterally."""
    d = T[:3, 3] - T_true[:3, 3]
    ray = T_true[:3, 3] - np.asarray(eye, dtype=float)
    ray /= np.linalg.norm(ray)
    along = float(d @ ray)
    return (np.linalg.norm(d - along * ray), along,
            np.degrees(rotation_angle(T[:3, :3].T @ T_true[:3, :3])))


def _check(T, T_true, eye, lateral_m=0.002, rel_depth=0.01, deg=1.0):
    lateral, along, angle = _errors(T, T_true, eye)
    distance = np.linalg.norm(T_true[:3, 3] - np.asarray(eye, dtype=float))
    assert lateral < lateral_m and abs(along) < rel_depth * distance and angle < deg, (
        f'lateral {lateral * 1000:.1f} mm, along-ray {along * 1000:.1f} mm, {angle:.2f} deg')


def _detect(tags, T_vr_link, **limits):
    gray = decode_gray(_jpeg(_render(tags, T_vr_link)))
    return detect_tags_in_vr(gray, K, D, T_vr_link, DET, **limits)


TAG = make_transform(_rz(40), [1.0, 0.3, 0.75])      # face up on a table, yawed 40 deg


def test_link_convention_matches_hand_projection():
    T_vr_link = _look_at([0.4, 0.1, 1.3], TAG[:3, 3])
    T_optical_vr = invert(T_vr_link @ T_LINK_OPTICAL)
    rng = np.random.default_rng(0)
    pts = TAG[:3, 3] + rng.uniform(-0.3, 0.3, size=(20, 3))
    cv_px, _ = cv2.projectPoints(pts, cv2.Rodrigues(T_optical_vr[:3, :3])[0],
                                 T_optical_vr[:3, 3], K, D)
    assert np.allclose(cv_px.reshape(-1, 2), [_project(T_vr_link, p) for p in pts], atol=1e-6)
    assert np.allclose(_project(T_vr_link, TAG[:3, 3]), K[:2, 2])   # looked-at point = centre
    left = (T_vr_link @ [1.0, 0.2, 0.0, 1.0])[:3]
    up = (T_vr_link @ [1.0, 0.0, 0.2, 1.0])[:3]
    assert _project(T_vr_link, left)[0] < K[0, 2] and _project(T_vr_link, up)[1] < K[1, 2]


def test_single_tag_pose_in_vr():
    T_vr_link = _look_at([0.45, 0.1, 1.25], TAG[:3, 3])
    sightings = _detect({7: TAG}, T_vr_link)
    assert [(s.id, s.rejected) for s in sightings] == [(7, '')]
    _check(sightings[0].T_vr_tag, TAG, T_vr_link[:3, 3])
    assert abs(sightings[0].distance_m - np.linalg.norm(TAG[:3, 3] - T_vr_link[:3, 3])) < 0.003


@pytest.mark.parametrize('azimuth_deg,elevation_deg', [(0, 30), (70, 35), (150, 45), (-110, 60)])
def test_oblique_views(azimuth_deg, elevation_deg):
    """Headset looking at the tag from all around, as low as 30 deg above the table."""
    a, e = np.radians(azimuth_deg), np.radians(elevation_deg)
    eye = TAG[:3, 3] + 0.8 * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    sightings = _detect({3: TAG}, _look_at(eye, TAG[:3, 3] + [0.05, -0.04, 0.0]))
    assert [s.id for s in sightings] == [3]
    _check(sightings[0].T_vr_tag, TAG, eye)


def test_several_tags():
    tags = {0: TAG,
            1: make_transform(_rz(-30), [1.1, -0.05, 0.75]),
            5: make_transform(_rz(100), [0.85, 0.55, 0.75])}
    eye = [0.35, 0.25, 1.35]
    sightings = _detect(tags, _look_at(eye, [0.98, 0.27, 0.75]))
    assert sorted(s.id for s in sightings) == [0, 1, 5]
    for s in sightings:
        _check(s.T_vr_tag, tags[s.id], eye)


def test_limits_reject_but_report():
    T_vr_link = _look_at([0.45, 0.1, 1.25], TAG[:3, 3])
    far = _detect({7: TAG}, T_vr_link, max_distance_m=0.5)
    assert len(far) == 1 and 'm >' in far[0].rejected
    noisy = _detect({7: TAG}, T_vr_link, max_reprojection_error_px=0.0)
    assert 'reproj' in noisy[0].rejected
    m = TagMap()
    assert add_sightings(m, far + noisy) == 0 and m.estimate(1) == {}


def test_empty_image_and_bad_input():
    assert detect_tags_in_vr(decode_gray(_jpeg(np.full((H, W), 255, np.uint8))), K, D,
                             np.eye(4), DET) == []
    for bad in (b'', b'not an image', []):
        with pytest.raises(ValueError):
            decode_gray(bad)
    jpeg = _jpeg(_render({2: TAG}, _look_at([0.5, 0.0, 1.2], TAG[:3, 3])))
    img = decode_gray(array.array('B', jpeg))           # rclpy gives uint8[] as array.array
    assert img.shape == (H, W) and img.dtype == np.uint8


def test_intrinsics():
    Ki, Di = intrinsics(list(K.reshape(-1)), [])
    assert np.allclose(Ki, K) and np.allclose(Di, np.zeros(5))
    d = [0.1, -0.2, 0.0, 0.0, 0.03]
    assert np.allclose(intrinsics(K.reshape(-1), d)[1], d)
    with pytest.raises(ValueError):
        intrinsics([0.0] * 9, [])


def test_fusion_over_a_moving_headset():
    """Sightings from many viewpoints fuse to the tag; observations carry the orientation."""
    tags = {0: TAG, 1: make_transform(_rz(-30), [1.1, -0.05, 0.75])}
    m = TagMap(window=150)
    for k, a in enumerate(np.linspace(-0.6, 0.6, 7)):
        eye = [1.0 - 0.6 * np.cos(a), 0.15 + 0.6 * np.sin(a), 1.25 + 0.02 * k]
        add_sightings(m, _detect(tags, _look_at(eye, [1.05, 0.12, 0.75])))
    est = m.estimate(min_observations=5)
    assert sorted(est) == [0, 1]
    for tag_id, p, q in to_observations(est):
        T = make_transform(quat_to_matrix(*q), p)
        assert np.linalg.norm(T[:3, 3] - tags[tag_id][:3, 3]) < 0.006
        assert np.degrees(rotation_angle(T[:3, :3].T @ tags[tag_id][:3, :3])) < 1.0
    assert [o[2] for o in to_observations(est, use_orientation=False)] == [None, None]
    lines, noisy = summarize(est, max_spread_m=0.03, residuals_m={0: 0.0012})
    assert noisy == [] and lines[0].startswith('tag 0: n=7') and 'residual 1.2 mm' in lines[0]


def test_spread_reporting():
    rng = np.random.default_rng(1)
    m = TagMap()
    for _ in range(30):
        for tag_id, sigma in ((0, 0.001), (4, 0.05)):
            T = TAG.copy()
            T[:3, 3] += rng.normal(scale=sigma, size=3)
            m.add(tag_id, T, 'quest')
    est = m.estimate(min_observations=5)
    lines, noisy = summarize(est, max_spread_m=0.03)
    assert noisy == [4] and '(> 30 mm)' in lines[1]
    assert np.linalg.norm(to_observations(est)[0][1] - TAG[:3, 3]) < 0.001


def test_draw_debug():
    T_vr_link = _look_at([0.45, 0.1, 1.25], TAG[:3, 3])
    gray = _render({7: TAG, 2: make_transform(_rz(0), [1.1, 0.0, 0.75])}, T_vr_link)
    sightings = detect_tags_in_vr(gray, K, D, T_vr_link, DET)
    sightings[0].rejected = 'test'
    img = draw_debug(gray, sightings, K, D, LENGTH / 2)
    assert img.shape == (H, W, 3) and img.dtype == np.uint8
    assert np.any(img[..., 1] > img[..., 0] + 100) and np.any(img[..., 2] > img[..., 1] + 100)
    assert draw_debug(gray, [], K, D, 0.05).shape == (H, W, 3)


def test_sighting_defaults_to_accepted():
    s = Sighting(1, np.eye(4), np.eye(4), 0.2, 0.5, np.zeros((4, 2)))
    m = TagMap()
    assert add_sightings(m, [s]) == 1 and list(m.estimate(1)) == [1]


def test_tag_map_mismatch():
    same = {'marker_length': 0.10, 'dictionary': 'DICT_4X4_50'}
    assert tag_map_mismatch(same, 0.10, 'DICT_4X4_50') == ''
    assert tag_map_mismatch({}, 0.10, 'DICT_4X4_50') == ''           # older map: nothing to check
    msg = tag_map_mismatch({'marker_length': 0.095, 'dictionary': 'DICT_4X4_50'}, 0.10,
                           'DICT_4X4_50')
    assert '0.095' in msg and '0.1' in msg and 'dictionary' not in msg
    assert 'DICT_5X5_50' in tag_map_mismatch(same, 0.10, 'DICT_5X5_50')
