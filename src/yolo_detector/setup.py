from setuptools import find_packages, setup

package_name = 'yolo_detector'

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
    description='Ultralytics YOLO 2D detector publishing vision_msgs/Detection2DArray.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_detector.yolo_node:main',
        ],
    },
)
