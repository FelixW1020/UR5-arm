"""Re-solve a saved sample set, e.g. to compare methods without touching the robot.

  ros2 run handeye_calibration solve_offline ~/.ros/handeye/table_cam_samples.yaml
"""
import argparse

import numpy as np

from handeye_calibration import calibration_io, solver
from ur5_common.transforms import matrix_to_quat


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('samples_file')
    p.add_argument('--method', choices=sorted(solver.METHODS), help='default: try all')
    a = p.parse_args()

    robot, target, meta = calibration_io.load_samples(a.samples_file)
    mount = meta['mount']
    print(f'{len(robot)} samples, mount={mount}, '
          f'rotation spread {solver.rotation_spread_deg(robot):.0f} deg\n')
    for method in [a.method] if a.method else sorted(solver.METHODS):
        try:
            r = solver.solve(robot, target, mount, method)
        except Exception as e:
            print(f'{method:>10}: failed ({e})')
            continue
        t = np.round(r.X[:3, 3], 4)
        q = np.round(matrix_to_quat(r.X), 4)
        print(f'{method:>10}: residual {r.translation_residual_m * 1000:6.2f} mm '
              f'{r.rotation_residual_deg:5.2f} deg | xyz={t} q(xyzw)={q}')


if __name__ == '__main__':
    main()
