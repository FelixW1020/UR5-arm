import numpy as np

from object_localization.geometry import box_depth, deproject, depth_to_metres

K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])


def test_deproject_centre_and_offset():
    assert np.allclose(deproject(320, 240, 0.8, K), [0, 0, 0.8])
    assert np.allclose(deproject(380, 180, 0.6, K), [0.06, -0.06, 0.6])


def test_box_depth_ignores_background_and_holes():
    depth = np.full((480, 640), 2.0, dtype=np.float32)   # background wall
    depth[200:280, 280:360] = 0.75                          # object
    depth[235:245, 315:325] = 0.0                           # invalid hole
    assert abs(box_depth(depth, 320, 240, 100, 100) - 0.75) < 1e-6


def test_box_depth_none_when_no_valid_pixels():
    assert box_depth(np.zeros((480, 640), np.float32), 320, 240, 50, 50) is None


def test_box_depth_clips_at_image_border():
    depth = np.full((480, 640), 1.0, dtype=np.float32)
    assert box_depth(depth, 2, 2, 60, 60) == 1.0


def test_depth_to_metres():
    assert np.allclose(depth_to_metres(np.array([[1000]], np.uint16), '16UC1'), 1.0)
    assert np.allclose(depth_to_metres(np.array([[1.5]], np.float32), '32FC1'), 1.5)
