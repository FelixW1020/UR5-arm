from setuptools import find_packages, setup

package_name = 'vr_tag_detector'

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
    description='Locates ArUco tags in vr_world from Quest passthrough frames and registers '
                'the headset to the robot with them.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detector = vr_tag_detector.tag_detector_node:main',
        ],
    },
)
