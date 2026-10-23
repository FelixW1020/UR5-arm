from setuptools import find_packages, setup

package_name = 'vr_teleop'

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
    description='Quest controller -> UR5 tool target mapping and controller/arm alignment check.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = vr_teleop.teleop_node:main',
            'alignment_monitor = vr_teleop.alignment_monitor_node:main',
        ],
    },
)
