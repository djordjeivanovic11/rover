from setuptools import setup

package_name = 'arm_template'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC Rover Team',
    maintainer_email='urc@rover.com',
    description='Parameter-driven 6-DOF arm template for URC rover missions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_srdf = arm_template.generate_srdf:main',
            'generate_moveit_config = arm_template.generate_moveit_config:main',
            'validate_params = arm_template.validate_params:main',
        ],
    },
) 