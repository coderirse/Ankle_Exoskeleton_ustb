from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_ankle',
            executable='serial_ForceSensor_node',
            name='serial_ForceSensor_node',
            output='screen',
            parameters=[{'debug': False}],
        ),
        Node(
            package='can_ankle',
            executable='serial_encoder_node',
            name='serial_encoder_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB2'}],
        ),
        Node(
            package='can_ankle',
            executable='serial_sendCommand_node',
            name='serial_sendCommand_node',
            output='screen',
        ),
        # 控制节点等传感器稳定后再启动，避免预紧时力传感器还没数据
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='can_ankle',
                    executable='can_ankleControl_node',
                    name='can_ankleControl_node',
                    output='screen',
                    parameters=[{
                        'control_mode': 1,
                        'user_weight': 60.0,
                        'force_limit': 20.0,
                        'force_emergency': 35.0,
                        'force_sign': 1.0,
                        'motor_dir': 1.0,
                        'preload_speed': 10.0,
                        'preload_force': 0.2,
                        'preload_force_min': 0.5,
                        'preload_force_max': 2.0,
                        'preload_timeout': 4.0,
                        'stand_confirm_time': 2.0,
                        'level_pitch_limit': 15.0,
                        'level_encoder_target': -22.14,
                        'level_encoder_limit': 10.0,
                        'init_timeout': 30.0,
                        'ff_gain': 15.0,
                        'pretension_speed': 60.0,
                        'max_speed': 600.0,
                    }],
                ),
            ],
        ),
    ])
