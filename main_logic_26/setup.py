from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'main_logic_26'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main_launch.py']),
        (os.path.join('share', 'main_logic_26', 'config'),
            glob('config/colorrange.json')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='pleblond20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensors_reading = main_logic_26.sensors_reading:main',
            'line_processing = main_logic_26.line_processing:main',
            'ultrasonic_processing = main_logic_26.ultrasonic_processing:main',
            'motion_controller = main_logic_26.motion_controller:main',
            'state_machine = main_logic_26.state_machine:main',
            'camera_processing = main_logic_26.camera_processing:main',
            'led_processing = main_logic_26.led_processing:main',
            'display_processing = main_logic_26.display_processing:main',
        ],
    },
)