from setuptools import find_packages, setup

package_name = 'vr_registration'

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
    description='Registers the Meta Quest world frame to the UR5 base frame.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'registration_node = vr_registration.registration_node:main',
            'pose_relay = vr_registration.pose_relay_node:main',
        ],
    },
)
