from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_ankle',
            executable='can_ankleControl_node',
            name='can_ankleControl_node',
            output='screen',
            prefix='xterm -e',
        ),
        Node(
            package='can_ankle',
            executable='serial_ForceSensor_node',
            name='serial_ForceSensor_node',
            output='screen',
            prefix='xterm -e',
        ),
        Node(
            package='can_ankle',
            executable='serial_encoder_node',
            name='serial_encoder_node',
            output='screen',
            prefix='xterm -e',
        ),
    ])
