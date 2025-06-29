from setuptools import setup

package_name = 'nav_supervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=['nav_supervisor'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/launch',
         ['launch/nav_supervisor.launch.py']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'nav_supervisor = nav_supervisor.node:main',
        ],
    },
)
