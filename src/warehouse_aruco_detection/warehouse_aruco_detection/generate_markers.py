#!/usr/bin/env python3
"""
generate_markers.py
-------------------
Utility script to generate ArUco marker images that you can print and
attach to boxes in the warehouse.

Run this script ONCE on your development PC (not on the robot):
    python3 generate_markers.py

It creates PNG files:
    marker_id0_good.png       — attach to 'good' product boxes
    marker_id1_defective.png  — attach to 'defective' product boxes

Print them at the correct physical size (see MARKER_SIZE_MM below).
The printed marker size MUST match the marker_size_m parameter in
aruco_real_robot.launch.py.

Usage
-----
    python3 generate_markers.py [--dict DICT_4X4_50] [--size 200] [--ids 0 1 2]

Default: generates IDs 0 and 1 at 200×200 px with a 1-px white border.
Scale the PNG so the BLACK square region = MARKER_SIZE_MM when printed.
(e.g. marker_size_m=0.10 → black region should be 100 mm × 100 mm)
"""

import argparse
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("ERROR: OpenCV not installed.  Run:  pip install opencv-contrib-python")
    sys.exit(1)

ARUCO_DICTS = {
    'DICT_4X4_50':         cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100':        cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250':        cv2.aruco.DICT_4X4_250,
    'DICT_5X5_50':         cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100':        cv2.aruco.DICT_5X5_100,
    'DICT_6X6_50':         cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100':        cv2.aruco.DICT_6X6_100,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}

LABEL_MAP = {0: 'good', 1: 'defective'}

# Physical print size of the BLACK marker square region
MARKER_SIZE_MM = 100   # mm  ← matches marker_size_m=0.10 in launch file


def generate(dict_name: str, marker_ids: list[int], px_size: int):
    dict_id = ARUCO_DICTS.get(dict_name)
    if dict_id is None:
        print(f'Unknown dictionary: {dict_name}')
        print(f'Valid options: {list(ARUCO_DICTS.keys())}')
        sys.exit(1)

    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    border = 1   # modules of white border around the marker

    for mid in marker_ids:
        img = cv2.aruco.generateImageMarker(aruco_dict, mid, px_size)

        # Add a white border so the detector can find the marker near edges
        img_with_border = cv2.copyMakeBorder(
            img,
            border * (px_size // (img.shape[0])),
            border * (px_size // (img.shape[0])),
            border * (px_size // (img.shape[0])),
            border * (px_size // (img.shape[0])),
            cv2.BORDER_CONSTANT, value=255
        )

        label = LABEL_MAP.get(mid, f'id{mid}')
        filename = f'marker_id{mid}_{label}.png'
        cv2.imwrite(filename, img_with_border)

        print(
            f'Saved {filename}  ({img_with_border.shape[1]}x{img_with_border.shape[0]} px)\n'
            f'  → Print so that the BLACK square region = {MARKER_SIZE_MM} mm × {MARKER_SIZE_MM} mm\n'
            f'  → This matches marker_size_m = {MARKER_SIZE_MM/1000:.3f} m in the launch file'
        )

    print('\nDone!  Attach the markers to the product boxes face-up.')
    print('ID 0 (good)       → red-labelled / "good" products')
    print('ID 1 (defective)  → blue-labelled / "defective" products')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate ArUco markers for warehouse project')
    parser.add_argument('--dict',  default='DICT_4X4_50',
                        help='ArUco dictionary (default: DICT_4X4_50)')
    parser.add_argument('--size',  type=int, default=200,
                        help='Marker image size in pixels (default: 200)')
    parser.add_argument('--ids',   type=int, nargs='+', default=[0, 1],
                        help='List of marker IDs to generate (default: 0 1)')
    args = parser.parse_args()

    generate(args.dict, args.ids, args.size)
