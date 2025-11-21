from setuptools import setup
import os
from glob import glob

package_name = 'mission_bt'

setup(
    name=package_name,
    version='0.0.0',
    packages=['mission_bt'],
    package_dir={'mission_bt': 'mission_bt'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover',
    maintainer_email='rover@example.com',
    description='Simple mission executor for GPS waypoint navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor = mission_bt.simple_mission_executor:main',
        ],
    },
)
