from setuptools import find_packages, setup

package_name = 'object_localization'

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
    description='Lifts 2D detections to 3D poses in the robot base frame.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = object_localization.localization_node:main',
        ],
    },
)
