from setuptools import setup
import os
from glob import glob

package_name = 'learm_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khizar',
    maintainer_email='khizar@example.com',
    description='LeArm Gazebo simulation',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
