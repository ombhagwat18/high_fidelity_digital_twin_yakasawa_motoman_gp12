import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'warehouse_controller'

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
    maintainer='omm',
    maintainer_email='omm@todo.todo',
    description='Simulated YRC1000 controller status/alarm/servo layer',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'yrc1000_controller = warehouse_controller.yrc1000_controller:main',
        ],
    },
)
