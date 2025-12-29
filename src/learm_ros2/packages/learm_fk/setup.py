from setuptools import setup

package_name = 'learm_fk'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skhizar',
    maintainer_email='skhizar@todo.todo',
    description='Forward kinematics node for LeArm using URDF + joint_states.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_urdf_node = learm_fk.fk_urdf_node:main',
        ],
    },
)
