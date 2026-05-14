from setuptools import setup
import os
from glob import glob

package_name = 'image_acquisition'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@robot.com',
    description='Image acquisition and YOLO detection nodes — Module 1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node          = image_acquisition.camera_node:main',
            'yolo_detection_node  = image_acquisition.yolo_detection_node:main',
        ],
    },
)
