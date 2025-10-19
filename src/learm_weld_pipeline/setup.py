from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'learm_weld_pipeline'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Crack-to-weld pipeline for LeArm (ROS 2, Python)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'crack_to_path = learm_weld_pipeline.crack_to_path:main',
            'camera_to_base = learm_weld_pipeline.camera_to_base:main',
            'path_to_joints = learm_weld_pipeline.path_to_joints:main',
            'welder_control = learm_weld_pipeline.welder_control:main',
        ],
    },
)
