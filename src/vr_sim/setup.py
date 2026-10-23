from setuptools import find_packages, setup

package_name = 'vr_sim'

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
    description='Simulated Quest 3 and kinematic UR5 for testing the VR pipeline in RViz.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quest_sim = vr_sim.quest_sim_node:main',
            'ur5_kinematic_sim = vr_sim.ur5_sim_node:main',
        ],
    },
)
