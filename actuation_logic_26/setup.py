from setuptools import find_packages, setup

package_name = 'actuation_logic_26'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main_launch.py']),
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

            # For the actuation logic: calling the in termnial sensors_reading, line_processing, ultrasonic_processing, motion_controller, state_machine
            'sensors_reading = actuation_logic_26.sensors_reading:main',
            'line_processing = actuation_logic_26.line_processing:main',
            'ultrasonic_processing = actuation_logic_26.ultrasonic_processing:main',
            'motion_controller = actuation_logic_26.motion_controller:main',
            'state_machine = actuation_logic_26.state_machine:main',
            'alignment_camera = actuation_logic_26.alignment_camera:main'
        ],
    },
)