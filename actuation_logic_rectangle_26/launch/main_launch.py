from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='actuation_logic_rectangle_26',
            executable='sensors_reading',
            name='sensors_reading_node',
            output='screen'
        ),

        Node(
            package='actuation_logic_rectangle_26',
            executable='line_processing',
            name='line_processing_node',
            output='screen'
        ),

        Node(
            package='actuation_logic_rectangle_26',
            executable='ultrasonic_processing',
            name='ultrasonic_processing_node',
            output='screen'
        ),

        Node(
            package='actuation_logic_rectangle_26',
            executable='motion_controller',
            name='motion_controller_node',
            output='screen'
        ),

        Node(
            package='actuation_logic_rectangle_26',
            executable='state_machine',
            name='state_machine_node',
            output='screen'
        ),

        Node(
            package='actuation_logic_rectangle_26',
            executable='alignment_camera',
            name='alignment_camera_node',
            output='screen'
        )
    ])