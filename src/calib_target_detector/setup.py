from setuptools import find_packages, setup

package_name = 'calib_target_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Felix Wang',
    maintainer_email='fw124@duke.edu',
    description='Detects a ChArUco calibration board and publishes its pose.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_detector = calib_target_detector.target_detector_node:main',
            'generate_board = calib_target_detector.generate_board:main',
        ],
    },
)
