from setuptools import setup
package_name = 'pointcloud_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/pointcloud_tools']),
        ('share/' + package_name + '/launch', ['launch/pointcloud_tools.launch.py']),
        ('share/' + package_name + '/config', ['config/pointcloud_params.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    entry_points={
        'console_scripts': [
            'depth_to_scan = pointcloud_tools.depth_to_scan_node:main',
            'grid_builder = pointcloud_tools.grid_builder_node:main',
        ],
    },
)
