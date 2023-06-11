from launch.launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='all_sky_publisher',
            executable='all_sky_publisher',
            name='all_sky_publisher'
        ),
        Node(
            package='background_subtractor',
            executable='background_subtractor',
            name='background_subtractor'
        ),
    ])
