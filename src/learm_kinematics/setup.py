from setuptools import setup
import os

package_name = 'learm_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # installs ./learm_kinematics/*.py
    data_files=[
        # Package index marker + package.xml
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [
            os.path.join('resource', package_name),
        ]),
        (os.path.join('share', package_name), ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), [
            'launch/teleop_sliders.launch.py',
            'launch/kinematics.launch.py',
            'launch/hw_bridge.launch.py',
        ]),

        # Install params
        (os.path.join('share', package_name, 'params'), [
            'params/dh.yaml',
            'params/hw.yaml',
        ]),

        # Optional: extra resources (ok to remove if you didn't create it)
        (os.path.join('share', package_name, 'resource'), [
            'resource/joint_names.txt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sumayya',
    maintainer_email='you@example.com',
    description='LeArm FK/IK, sliders UI, and PCA9685 hardware bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_sliders = learm_kinematics.teleop_sliders:main',
            'fk_node        = learm_kinematics.fk_node:main',
            'ik_node        = learm_kinematics.ik_node:main',
            'learm_hw_bridge = learm_kinematics.learm_hw_bridge:main',
            'fk_echo = learm_kinematics.fk_echo:main',
        ],
    },
)
