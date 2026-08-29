from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_aruco_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register with ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitanya',
    maintainer_email='chaitanya@todo.todo',
    description='ArUco detection for real GP12 robot with RealSense D455',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'aruco_detector   = warehouse_aruco_detection.aruco_detector:main',
            'aruco_detector_sim = warehouse_aruco_detection.aruco_detector_sim:main',
            'generate_markers = warehouse_aruco_detection.generate_markers:main',
            'extrinsic_calibrator = warehouse_aruco_detection.extrinsic_calibrator:main',
        ],
    },
)
