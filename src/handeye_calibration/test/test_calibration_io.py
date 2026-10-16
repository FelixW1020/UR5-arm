import numpy as np

from handeye_calibration import calibration_io, solver
from ur5_common.transforms import make_transform, quat_to_matrix


def test_round_trip(tmp_path):
    X = make_transform(quat_to_matrix(0.1, 0.2, 0.3, 0.9), [0.05, -0.02, 0.11])
    result = solver.HandEyeResult(X, 0.001, 0.1, 'park', 12)
    path = calibration_io.save_calibration(str(tmp_path / 'c.yaml'), result, 'eye_in_hand',
                                           'tool0', 'camera_color_optical_frame')
    parent, child, T, data = calibration_io.load_calibration(path)
    assert (parent, child) == ('tool0', 'camera_color_optical_frame')
    assert np.allclose(T, X)
    assert data['num_samples'] == 12

    sp = calibration_io.samples_path_for(path)
    assert sp.endswith('c_samples.yaml')
    calibration_io.save_samples(sp, [X, X], [X, np.eye(4)], {'mount': 'eye_in_hand'})
    robot, target, meta = calibration_io.load_samples(sp)
    assert meta['mount'] == 'eye_in_hand'
    assert np.allclose(robot[1], X) and np.allclose(target[1], np.eye(4))
