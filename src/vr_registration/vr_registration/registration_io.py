"""Save/load the base_link <-> vr_world registration."""
import datetime
import os

import yaml

from ur5_common.transforms import make_transform, matrix_to_quat, quat_to_matrix


def save_registration(path, registration, base_frame, vr_frame, extra=None):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    T = registration.T_base_vr
    data = {
        'parent_frame': base_frame,
        'child_frame': vr_frame,
        'translation': {k: float(v) for k, v in zip('xyz', T[:3, 3])},
        'rotation': {k: float(v) for k, v in zip('xyzw', matrix_to_quat(T))},
        'method': registration.method,
        'rms_error_m': registration.rms_error_m,
        'max_error_m': registration.max_error_m,
        'created': datetime.datetime.now().isoformat(timespec='seconds'),
    }
    data.update(extra or {})
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def load_registration(path):
    """Returns (parent_frame, child_frame, T_parent_child, raw dict)."""
    with open(os.path.expanduser(path)) as f:
        d = yaml.safe_load(f)
    t, q = d['translation'], d['rotation']
    T = make_transform(quat_to_matrix(q['x'], q['y'], q['z'], q['w']), [t['x'], t['y'], t['z']])
    return d['parent_frame'], d['child_frame'], T, d
