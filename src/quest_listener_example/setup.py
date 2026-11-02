from setuptools import find_packages, setup

package_name = 'quest_listener_example'

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
    description='Example of robot code using the Quest poses: copy it into your own package.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'listener = quest_listener_example.listener_node:main',
        ],
    },
)
