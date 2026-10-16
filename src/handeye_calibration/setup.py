from setuptools import find_packages, setup

package_name = 'handeye_calibration'

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
    description='Collects robot/board pose pairs and solves hand-eye calibration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = handeye_calibration.calibration_node:main',
            'solve_offline = handeye_calibration.solve_offline:main',
        ],
    },
)
