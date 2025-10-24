from setuptools import setup
import os
from glob import glob

package_name = 'nav2_teensy_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Djordje Ivanovic',
    maintainer_email='dorde_ivanovic@college.harvard.edu',
    description='Bridge between Nav2 cmd_vel and Teensy motor controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = nav2_teensy_bridge.nav2_teensy_bridge:main',
        ],
    },
)

