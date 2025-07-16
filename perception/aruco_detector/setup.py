from setuptools import setup, find_packages

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}/launch', ['launch/aruco_detector.launch.py']),
        (f'share/{package_name}/config', ['config/detector_params.yaml']),
        (f'share/{package_name}/scripts',
         ['scripts/cam_cal.py', 'scripts/process.py',
          'scripts/generate_markers.py']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Djordje Ivanovic',
    maintainer_email='dorde_ivanovic@college.harvard.edu',
    description='ArUco tag detector that works with ZED 2i camera feeds.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector_node = aruco_detector.node:main',
            'cam_cal.py = scripts.cam_cal:main',
            'process.py = scripts.process:main',
            'generate_markers.py = scripts.generate_markers:main',
        ],
    },
)
