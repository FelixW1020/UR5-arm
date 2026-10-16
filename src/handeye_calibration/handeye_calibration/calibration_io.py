"""Read/write calibration results and raw samples as YAML."""
import datetime
import os

import yaml

from ur5_common.transforms import make_transform, matrix_to_quat, quat_to_matrix


def _to_dict(T):
    q = matrix_to_quat(T)
    return {
        'translation': {k: float(v) for k, v in zip('xyz', T[:3, 3])},
        'rotation': {k: float(v) for k, v in zip('xyzw', q)},
    }


def _from_dict(d):
    t, q = d['translation'], d['rotation']
    return make_transform(quat_to_matrix(q['x'], q['y'], q['z'], q['w']),
                          [t['x'], t['y'], t['z']])


def save_calibration(path, result, mount, parent_frame, child_frame):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    data = {
        'mount': mount,
        'parent_frame': parent_frame,
        'child_frame': child_frame,
        'transform': _to_dict(result.X),
        'method': result.method,
        'num_samples': result.num_samples,
        'translation_residual_m': result.translation_residual_m,
        'rotation_residual_deg': result.rotation_residual_deg,
        'created': datetime.datetime.now().isoformat(timespec='seconds'),
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def load_calibration(path):
    """Returns (parent_frame, child_frame, 4x4 T_parent_child, raw dict)."""
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f)
    return data['parent_frame'], data['child_frame'], _from_dict(data['transform']), data


def save_samples(path, T_base_gripper, T_cam_target, meta):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    data = dict(meta)
    data['samples'] = [{'base_gripper': _to_dict(a), 'cam_target': _to_dict(b)}
                       for a, b in zip(T_base_gripper, T_cam_target)]
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def load_samples(path):
    """Returns (list T_base_gripper, list T_cam_target, meta dict)."""
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f)
    samples = data.pop('samples')
    return ([_from_dict(s['base_gripper']) for s in samples],
            [_from_dict(s['cam_target']) for s in samples],
            data)


def samples_path_for(calibration_path):
    root, ext = os.path.splitext(os.path.expanduser(calibration_path))
    return f'{root}_samples{ext or ".yaml"}'
