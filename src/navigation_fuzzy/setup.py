from setuptools import setup

package_name = 'navigation_fuzzy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_fuzzy.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afro-robotics',
    maintainer_email='ketchupmayonnaise85@gmail.com',
    description='Straight-line navigation with fuzzy wheel sync and fuzzy speed control',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'encoder_node           = navigation_fuzzy.encoder_node:main',
            'navigation_fuzzy_node  = navigation_fuzzy.navigation_fuzzy_node:main',
        ],
    },
)
