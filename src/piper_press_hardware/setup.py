from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'piper_press_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piper press team',
    maintainer_email='root@todo.todo',
    description='Piper feedback and guarded J1-J5 commissioning probes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_state_reader = piper_press_hardware.joint_state_reader:main',
            'can_gap_monitor = piper_press_hardware.can_gap_monitor:main',
            'hardware_preflight = piper_press_hardware.hardware_preflight:main',
            'enable_probe = piper_press_hardware.enable_probe:main',
            'zero_home = piper_press_hardware.zero_home:main',
        ],
    },
)
