from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'arm_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Configuration files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Test data files
        (os.path.join('share', package_name, 'test', 'data'),
            glob('test/data/*.yaml')),
        # RViz configuration (if created later)
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC Rover Team',
    maintainer_email='urc@rover.com',
    description='Runtime control layer for URC rover arm - bridges MoveIt planning to hardware execution',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_interface = arm_control.hardware_interface:main',
            'trajectory_executor = arm_control.trajectory_executor:main',
            'safety_monitor = arm_control.safety_monitor:main',
            'gripper_controller = arm_control.gripper_controller:main',
            'tool_manager = arm_control.tool_manager:main',
            'action_servers = arm_control.action_servers:main',
        ],
    },
) 