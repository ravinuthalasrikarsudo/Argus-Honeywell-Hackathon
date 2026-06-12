from glob import glob

from setuptools import find_packages, setup

package_name = 'argus_health'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vittal',
    maintainer_email='vittal.muku@gmail.com',
    description='ARGUS VIO health monitor + recovery.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health_monitor = argus_health.health_monitor:main',
        ],
    },
)
