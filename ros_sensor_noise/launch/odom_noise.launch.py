from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    config_dir = os.path.join(
        get_package_share_directory('ros_sensor_noise'),
        'config'
    )
    config_file = os.path.join(config_dir, 'noise.yaml')

    return LaunchDescription([
        Node(
            package='ros_sensor_noise',
            executable='ros_odom_noise_generator',
            name='ros_odom_noise_generator',
            output='screen',
            parameters=[config_file]
        )
    ])
