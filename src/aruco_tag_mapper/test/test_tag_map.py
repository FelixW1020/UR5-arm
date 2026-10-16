import numpy as np

from aruco_tag_mapper.tag_map import TagMap, load_tag_map, save_tag_map
from ur5_common.transforms import make_transform


def test_fusion_and_disagreement(tmp_path):
    rng = np.random.default_rng(0)
    m = TagMap()
    truth = np.array([0.5, 0.2, 0.0])
    for _ in range(50):
        m.add(3, make_transform(np.eye(3), truth + rng.normal(scale=0.001, size=3)), 'table_cam')
        m.add(3, make_transform(np.eye(3), truth + [0.004, 0, 0] + rng.normal(scale=0.001, size=3)),
              'wrist_cam')
    m.add(9, np.eye(4), 'table_cam')          # too few observations -> dropped
    est = m.estimate(min_observations=10)
    assert set(est) == {3}
    e = est[3]
    assert np.linalg.norm(e.T_base_tag[:3, 3] - truth - [0.002, 0, 0]) < 0.002
    assert 0.003 < e.source_disagreement_m < 0.005

    path = save_tag_map(str(tmp_path / 'tags.yaml'), est, 'base_link', 0.1, 'DICT_4X4_50')
    frame, tags = load_tag_map(path)
    assert frame == 'base_link' and np.allclose(tags[3], e.T_base_tag)
