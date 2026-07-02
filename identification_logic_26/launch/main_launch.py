from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='identification_logic_26',
            executable='camera_processing',
            name='camera_processing_node',
            output='screen'
        ),

        Node(
            package='identification_logic_26',
            executable='led_processing',
            name='led_processing_node',
            output='screen'
        ),

        Node(
            package='identification_logic_26',
            executable='display',
            name='display_node',
            output='screen'
        ),

        # Node(
        #     package='identification_logic_26',
        #     executable='keyboard_capture',
        #     name='keyboard_capture_node',
        #     output='screen'
        # ),
    ])