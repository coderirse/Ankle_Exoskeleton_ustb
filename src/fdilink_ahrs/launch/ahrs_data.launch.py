from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fdilink_ahrs',
            executable='ahrs_driver',
            name='ahrs_driver',
            output='screen',
            parameters=[{
                'debug': False,
                'port': '/dev/ttyACM0',
                'baud': 115200,
                'imu_topic': 'imu',
                'imu_frame': 'gyro_link',
                'mag_pose_2d_topic': '/mag_pose_2d',
                'Euler_angles_pub_': '/euler_angles',
                'Magnetic_pub_': '/magnetic',
                'gps_topic_': '/gps/fix',
                'twist_topic_': '/system_speed',
                'NED_odom_topic_': '/NED_odometry',
                'device_type': 1,
            }],
        ),
    ])
