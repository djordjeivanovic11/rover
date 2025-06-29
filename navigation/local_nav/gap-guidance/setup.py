from setuptools import setup

package_name = 'gap_guidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/launch',
         ['launch/gap_guidance.launch.py']),
        ('share/' + package_name + '/config',
         ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'gap_guidance_node = gap_guidance.node:main',
        ],
    },
)
