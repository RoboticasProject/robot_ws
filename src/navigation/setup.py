from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot.launch.py',
            'launch/robot_line.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afro-robotics',
    maintainer_email='yketchupmayonnaise85@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'encoder_node          = navigation.encoder_node:main',
            'navigation_node       = navigation.navigation_node:main',
            'line_follower_node    = navigation.line_follower_node:main',
            'navigation_line_node  = navigation.navigation_line_node:main',
        ],
    },
)
