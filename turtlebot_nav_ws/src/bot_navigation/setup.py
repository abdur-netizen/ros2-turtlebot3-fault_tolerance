import os
from glob import glob
from setuptools import setup

package_name = 'bot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdur Razak',
    maintainer_email='abdur@example.com',
    description='Fault-tolerant sensor fusion and autonomous waypoint navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_navigator = bot_navigation.waypoint_navigator:main',
            'fault_injector = bot_navigation.fault_injector:main',
            'fault_supervisor = bot_navigation.fault_supervisor:main',
        ],
    },
)
