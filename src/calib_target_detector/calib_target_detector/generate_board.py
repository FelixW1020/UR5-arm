"""Write a printable ChArUco board PNG matching the detector's parameters.

  ros2 run calib_target_detector generate_board --out board.png
Print at 100% scale, then MEASURE a square with calipers and put the measured
size in config/calibration.yaml (square_length / marker_length).
"""
import argparse

import cv2

from calib_target_detector.charuco import CharucoBoardDetector


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--squares-x', type=int, default=7)
    p.add_argument('--squares-y', type=int, default=5)
    p.add_argument('--square-length', type=float, default=0.035, help='metres')
    p.add_argument('--marker-length', type=float, default=0.026, help='metres')
    p.add_argument('--dictionary', default='DICT_5X5_100')
    p.add_argument('--dpi', type=int, default=300)
    p.add_argument('--out', default='charuco_board.png')
    a = p.parse_args()

    det = CharucoBoardDetector(a.squares_x, a.squares_y, a.square_length,
                               a.marker_length, a.dictionary)
    px_per_m = a.dpi / 0.0254
    margin = int(0.01 * px_per_m)
    w = int(a.squares_x * a.square_length * px_per_m) + 2 * margin
    h = int(a.squares_y * a.square_length * px_per_m) + 2 * margin
    cv2.imwrite(a.out, det.render(w, h, margin))
    print(f'Wrote {a.out} ({w}x{h}px @ {a.dpi} dpi). Print at 100% scale.')


if __name__ == '__main__':
    main()
