from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rr26_stuff'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mike@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rr26_can_xy_node_lc = rr26_stuff.rr26_can_xy_node_lc:main',
            'rr26_front_sensors_node_lc = rr26_stuff.rr26_front_sensors_node_lc:main',
            'rr26_openmv_serial_node_lc = rr26_stuff.rr26_openmv_serial_node_lc:main',
            "rr26_wheel_controller_node_lc = rr26_stuff.rr26_wheel_controller_node_lc:main",
            "rr26_teleop_node = rr26_stuff.rr26_teleop_node:main",
            "rr26_controller_node_lc = rr26_stuff.rr26_controller_node_lc:main",
            'rr26_lifecycle_node_manager = rr26_stuff.rr26_lifecycle_node_manager:main',
        ],
    },
)
