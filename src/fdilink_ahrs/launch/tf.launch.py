from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fdilink_ahrs',
            executable='imu_tf',
            name='imu_tf',
            output='screen',
            parameters=[{
                'imu_topic': '/imu',
                'world_frame_id': '/world',
                'imu_frame_id': '/gyro_link',
                'position_x': 1,
                'position_y': 1,
                'position_z': 0,
            }],
        ),
    ])
