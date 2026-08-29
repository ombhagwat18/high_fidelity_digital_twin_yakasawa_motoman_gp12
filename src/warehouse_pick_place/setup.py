import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'warehouse_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitanya',
    maintainer_email='chaitanya@todo.todo',
    description='Warehouse pick and place — conveyor-to-bin parcel handling with MoveIt2 and a vacuum gripper',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'pick_place_logic = warehouse_pick_place.pick_place_logic:main',
        ],
    },
)
