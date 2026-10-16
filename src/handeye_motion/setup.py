from setuptools import find_packages, setup

package_name = 'handeye_motion'

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
    description='Records and replays UR5 joint poses for hand-eye calibration.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motion_node = handeye_motion.motion_node:main',
        ],
    },
)
