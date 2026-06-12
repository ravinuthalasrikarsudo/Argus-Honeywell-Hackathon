from glob import glob

from setuptools import find_packages, setup

package_name = 'argus_superpoint'

setup(
    name=package_name,
    version='0.3.0',
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
    description='ARGUS SuperPoint/LightGlue ONNX learned-feature front-end (standalone).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # NOTE: this entry point runs under the colcon-selected interpreter
            # (system python3), which lacks onnxruntime-gpu. The supported way to
            # run the node is the venv launcher:
            #   bash scripts/run_superpoint.sh
            # which sources ROS + runs ~/.venvs/argus-sp/bin/python on this module.
            'superpoint_node = argus_superpoint.superpoint_node:main',
        ],
    },
)
