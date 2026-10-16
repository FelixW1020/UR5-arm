"""Fuse many tag observations (already in the robot base frame) into one pose per tag."""
from collections import defaultdict, deque
from dataclasses import dataclass, field
import datetime
import os

import numpy as np
import yaml

from ur5_common.transforms import (make_transform, matrix_to_quat, project_to_rotation,
                                   quat_to_matrix)


@dataclass
class TagEstimate:
    T_base_tag: np.ndarray
    num_observations: int
    position_std_m: float                       # spread of the observations
    per_source_position: dict = field(default_factory=dict)   # camera -> median xyz

    @property
    def source_disagreement_m(self):
        """Largest distance between the per-camera estimates (0 if one camera)."""
        p = list(self.per_source_position.values())
        return max((float(np.linalg.norm(a - b)) for i, a in enumerate(p) for b in p[i + 1:]),
                   default=0.0)


class TagMap:

    def __init__(self, window=300):
        self._obs = defaultdict(lambda: deque(maxlen=window))   # (id, source) -> T list

    def add(self, tag_id, T_base_tag, source):
        self._obs[(int(tag_id), source)].append(np.asarray(T_base_tag, dtype=float))

    def clear(self):
        self._obs.clear()

    def estimate(self, min_observations=10):
        by_id = defaultdict(dict)
        for (tag_id, source), Ts in self._obs.items():
            if Ts:
                by_id[tag_id][source] = list(Ts)
        result = {}
        for tag_id, sources in by_id.items():
            Ts = [T for lst in sources.values() for T in lst]
            if len(Ts) < min_observations:
                continue
            ps = np.array([T[:3, 3] for T in Ts])
            centre = np.median(ps, axis=0)
            R = project_to_rotation(sum(T[:3, :3] for T in Ts))
            result[tag_id] = TagEstimate(
                make_transform(R, centre), len(Ts),
                float(np.sqrt(np.mean(np.sum((ps - centre) ** 2, axis=1)))),
                {s: np.median([T[:3, 3] for T in lst], axis=0) for s, lst in sources.items()})
        return result


def save_tag_map(path, estimates, base_frame, marker_length, dictionary):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tags = {}
    for tag_id, e in sorted(estimates.items()):
        q = matrix_to_quat(e.T_base_tag)
        tags[int(tag_id)] = {
            'position': {k: float(v) for k, v in zip('xyz', e.T_base_tag[:3, 3])},
            'orientation': {k: float(v) for k, v in zip('xyzw', q)},
            'num_observations': e.num_observations,
            'position_std_m': e.position_std_m,
            'source_disagreement_m': e.source_disagreement_m,
            'sources': sorted(e.per_source_position),
        }
    data = {'frame_id': base_frame, 'dictionary': dictionary, 'marker_length': marker_length,
            'created': datetime.datetime.now().isoformat(timespec='seconds'), 'tags': tags}
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def load_tag_map(path):
    """Returns (frame_id, {id: 4x4 T_base_tag})."""
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f)
    tags = {}
    for tag_id, t in (data.get('tags') or {}).items():
        p, q = t['position'], t['orientation']
        tags[int(tag_id)] = make_transform(quat_to_matrix(q['x'], q['y'], q['z'], q['w']),
                                           [p['x'], p['y'], p['z']])
    return data['frame_id'], tags
