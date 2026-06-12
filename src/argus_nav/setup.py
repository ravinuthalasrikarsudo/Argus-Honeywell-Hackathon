from glob import glob

from setuptools import find_packages, setup

package_name = 'argus_nav'

setup(
    name=package_name,
    version='0.6.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vittal',
    maintainer_email='vittal.muku@gmail.com',
    description='ARGUS dense stereo perception + reactive obstacle avoidance.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_depth = argus_nav.stereo_depth:main',
            'reactive_avoider = argus_nav.reactive_avoider:main',
            'occupancy_mapper = argus_nav.occupancy_mapper:main',
        ],
    },
)
