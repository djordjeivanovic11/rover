from setuptools import setup
from glob import glob
import os

package_name = 'rover_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC Team',
    maintainer_email='team@urc.edu',
    description='URC Rover Control System with ROS 2 Control, safety monitoring, and competition-specific features.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_interface = rover_control.hardware_interface:main',
            'safety_monitor = rover_control.safety_monitor:main',
            'action_servers = rover_control.action_servers:main',
            'path_follower = rover_control.path_follower:main',
            'odometry_publisher = rover_control.odometry_publisher:main',
        ],
    },
) 