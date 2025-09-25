from setuptools import setup
import os
from glob import glob

package_name = 'slam_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Config files  
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dorde Ivanovic',
    maintainer_email='dorde_ivanovic@college.harvard.edu',
    description='URC SLAM and Complete Perception System Launch Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_health_monitor.py = slam_launch.perception_health_monitor:main',
            'system_status.py = slam_launch.system_status:main',
            'semantic_mapper.py = slam_launch.semantic_mapper:main',
            'sensor_fusion.py = slam_launch.sensor_fusion:main',
            'perception_guardian.py = slam_launch.perception_guardian:main',
            'calibration_validator.py = slam_launch.calibration_validator:main',
        ],
    },
)
