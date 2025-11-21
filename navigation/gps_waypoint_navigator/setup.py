from setuptools import setup

package_name = 'gps_waypoint_navigator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
        ('share/' + package_name + '/launch', ['launch/gps_navigator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Djordje Ivanovic',
    maintainer_email='dorde_ivanovic@college.harvard.edu',
    description='GPS waypoint navigation node bridging GPS, Nav2, and drive_control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_navigator_node = gps_waypoint_navigator.gps_navigator_node:main',
            'record_waypoint = gps_waypoint_navigator.record_waypoint:main',
        ],
    },
)


