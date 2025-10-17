from setuptools import setup
from glob import glob

package_name = 'zed_integration'

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
    maintainer='URC Perception Team',
    maintainer_email='rover@example.com',
    description='Global localization and spatial mapping integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_localization = zed_integration.global_localization:main',
            'spatial_mapping = zed_integration.spatial_mapping:main',
            'coordinate_converter = zed_integration.coordinate_converter:main',
            'occupancy_grid_converter = zed_integration.occupancy_grid_converter:main',
        ],
    },
)
