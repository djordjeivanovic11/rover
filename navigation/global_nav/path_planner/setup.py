from setuptools import setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/launch',
         ['launch/path_planner.launch.py']),
        ('share/' + package_name + '/config',
         ['config/cost_params.yaml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'path_planner_node = path_planner.node:main',
        ],
    },
)
