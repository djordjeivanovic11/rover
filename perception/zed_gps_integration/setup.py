from setuptools import setup
import os
from glob import glob

package_name = 'zed_gps_integration'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web_map'), glob('web_map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='rover@example.com',
    description='ZED Camera + GNSS Fusion with Live Map Visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_gnss_fusion = zed_gps_integration.zed_gnss_fusion:main',
            'map_server = zed_gps_integration.map_server:main',
        ],
    },
)

