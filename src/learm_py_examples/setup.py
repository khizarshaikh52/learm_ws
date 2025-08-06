from setuptools import setup

package_name = 'learm_py_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        # This makes ROS 2 aware of your package via ament_index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # This installs your package.xml so ros2 pkg list works
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@your.email',
    description='Simple ROS 2 Python examples (param, pub/sub, service)',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'simple_parameter       = learm_py_examples.simple_parameter:main',
            'simple_publisher       = learm_py_examples.simple_publisher:main',
            'simple_service_server  = learm_py_examples.simple_service_server:main',
            'simple_subscriber      = learm_py_examples.simple_subscriber:main',
        ],
    },
)
