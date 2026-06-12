import os
from glob import glob

from setuptools import setup

package_name = 'argus_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/argus_bridge.yaml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vittal',
    maintainer_email='vittal.muku@gmail.com',
    description='ARGUS ROS<->gz bringup: parameter_bridge config + camera_info baseline patch.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_info_patch = argus_bringup.camera_info_patch:main',
            'drive_drone = argus_bringup.drive_drone:main',
            'record_bag = argus_bringup.record_bag:main',
            'check_stack = argus_bringup.check_stack:main',
            'acceptance = argus_bringup.acceptance:main',
        ],
    },
)
