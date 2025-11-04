from setuptools import setup
import os
from glob import glob

package_name = 'drive_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC Team',
    maintainer_email='rover@urc.edu',
    description='Wheel-speed based motor control for URC rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_wheels = drive_control.twist_to_wheels:main',
            'wheel_bridge = drive_control.wheel_bridge:main',
        ],
    },
)
