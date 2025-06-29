from setuptools import setup

package_name = 'rover_boot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # so ROS 2 can find this package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # include our launch file
        ('share/' + package_name + '/launch',
         ['launch/rover.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'supervisor = rover_boot.supervisor_node:main',
        ],
    },
)
