#!/usr/bin/env python3
"""
generate_parcel_marker_assets.py
---------------------------------
Build-time asset generator: creates the ArUco marker PNG textures + Ogre
.material scripts that let Gazebo classic render a real, detectable ArUco
pattern on the top face of every spawned parcel model.

Invoked automatically by CMakeLists.txt at colcon-build configure time — do
not run manually unless regenerating assets after changing MARKER_IDS below.
Idempotent: skips generation if the output directory already has all files.

Requires OpenCV with the aruco module (already a dependency of the perception
stack — cv2.aruco). If unavailable at CMake-configure time (e.g. a minimal
build container), this script exits 0 with a warning instead of failing the
whole workspace build; Gazebo will then show a plain grey box in place of the
marker face until assets are generated (perception won't be able to detect it
until then — see the printed instructions).

NOTE: MARKER_IDS here must match warehouse_gazebo/config/parcel_catalog.yaml.
"""

import os
import sys

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'models', 'parcel_markers')
TEX_DIR = os.path.join(OUT_DIR, 'materials', 'textures')
SCRIPT_DIR = os.path.join(OUT_DIR, 'materials', 'scripts')

MARKER_IDS = [0, 1, 2, 3, 4, 5]   # keep in sync with parcel_catalog.yaml
PX_SIZE = 256
ARUCO_DICT_NAME = 'DICT_4X4_50'


def _already_generated() -> bool:
    if not os.path.isdir(TEX_DIR):
        return False
    for mid in MARKER_IDS:
        if not os.path.isfile(os.path.join(TEX_DIR, f'marker_{mid}.png')):
            return False
    return os.path.isfile(os.path.join(SCRIPT_DIR, 'parcel_markers.material'))


def main() -> int:
    if _already_generated():
        print('[generate_parcel_marker_assets] assets already present, skipping.')
        return 0

    try:
        import cv2
    except ImportError:
        print(
            '[generate_parcel_marker_assets] WARNING: OpenCV not available at '
            'configure time — parcel marker textures were NOT generated.\n'
            '  Parcels will spawn as plain grey boxes and will NOT be detectable\n'
            '  by parcel_perception until you run this script manually with a\n'
            '  Python that has opencv-contrib-python installed:\n'
            f'    python3 {os.path.abspath(__file__)}'
        )
        return 0

    os.makedirs(TEX_DIR, exist_ok=True)
    os.makedirs(SCRIPT_DIR, exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, ARUCO_DICT_NAME))

    material_lines = []
    for mid in MARKER_IDS:
        img = cv2.aruco.generateImageMarker(aruco_dict, mid, PX_SIZE)
        # White border so the detector finds the marker cleanly near edges.
        border = PX_SIZE // 8
        img = cv2.copyMakeBorder(
            img, border, border, border, border,
            cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(os.path.join(TEX_DIR, f'marker_{mid}.png'), img)

        material_lines.append(
            f'material ParcelMarker/marker_{mid}\n'
            '{\n'
            '  technique\n'
            '  {\n'
            '    pass\n'
            '    {\n'
            '      texture_unit\n'
            '      {\n'
            f'        texture marker_{mid}.png\n'
            '      }\n'
            '    }\n'
            '  }\n'
            '}\n'
        )

    with open(os.path.join(SCRIPT_DIR, 'parcel_markers.material'), 'w') as f:
        f.write('\n'.join(material_lines))

    model_config = os.path.join(OUT_DIR, 'model.config')
    if not os.path.isfile(model_config):
        with open(model_config, 'w') as f:
            f.write(
                '<?xml version="1.0"?>\n'
                '<model>\n'
                '  <name>Parcel Markers</name>\n'
                '  <version>1.0</version>\n'
                '  <sdf version="1.6">model.sdf</sdf>\n'
                '  <description>\n'
                '    Materials-only resource: ArUco marker textures for spawned\n'
                '    conveyor parcels. Referenced via model://parcel_markers in\n'
                '    material scripts, never spawned as a model itself.\n'
                '  </description>\n'
                '</model>\n'
            )
    model_sdf = os.path.join(OUT_DIR, 'model.sdf')
    if not os.path.isfile(model_sdf):
        with open(model_sdf, 'w') as f:
            f.write(
                '<?xml version="1.0"?>\n'
                '<sdf version="1.6">\n'
                '  <model name="parcel_markers">\n'
                '    <static>true</static>\n'
                '    <link name="link"/>\n'
                '  </model>\n'
                '</sdf>\n'
            )

    print(f'[generate_parcel_marker_assets] generated {len(MARKER_IDS)} marker '
          f'textures + material script under {OUT_DIR}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
