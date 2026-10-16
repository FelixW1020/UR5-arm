"""Write printable ArUco tag PNGs (one per id, with the id printed under it).

  ros2 run aruco_tag_mapper generate_tags --ids 0 1 2 3 4 5 --out-dir tags/
Print at 100% scale and MEASURE the black square; put it in marker_length.
Larger tags are tracked better by the Quest passthrough cameras.
"""
import argparse
import os

import cv2
import numpy as np

from aruco_tag_mapper.markers import ArucoMarkerDetector


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--ids', type=int, nargs='+', default=[0, 1, 2, 3, 4, 5])
    p.add_argument('--marker-length', type=float, default=0.10, help='black square, metres')
    p.add_argument('--dictionary', default='DICT_4X4_50')
    p.add_argument('--dpi', type=int, default=300)
    p.add_argument('--out-dir', default='aruco_tags')
    a = p.parse_args()

    det = ArucoMarkerDetector(a.dictionary, a.marker_length)
    side = int(round(a.marker_length / 0.0254 * a.dpi))
    border = side // 8                    # white quiet zone around the tag
    os.makedirs(a.out_dir, exist_ok=True)
    for tag_id in a.ids:
        img = np.full((side + 2 * border + 60, side + 2 * border), 255, np.uint8)
        img[border:border + side, border:border + side] = det.render(tag_id, side)
        cv2.putText(img, f'{a.dictionary} id {tag_id}  {a.marker_length * 1000:.0f} mm',
                    (border, side + 2 * border + 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
        path = os.path.join(a.out_dir, f'tag_{tag_id}.png')
        cv2.imwrite(path, img)
        print(f'Wrote {path}')


if __name__ == '__main__':
    main()
