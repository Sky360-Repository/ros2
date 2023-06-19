from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sky360_camera',
            executable='all_sky_publisher_node',
            name='all_sky_publisher_node',
            output='screen'
        ),
        Node(
            package='sky360_image_processing',
            executable='frame_provider_node',
            name='frame_provider_node',
            output='screen'
        ),
        Node(
            package='sky360_image_processing',
            executable='background_subtractor_node',
            name='background_subtractor_node',
            output='screen'
        ),
        Node(
            package='sky360_image_processing',
            executable='background_subtractor_detector_node',
            name='background_subtractor_detector_node',
            output='screen'
        ),
        Node(
            package='sky360_tracking',
            executable='track_provider_node',
            name='track_provider_node',
            output='screen'
        ),
        Node(
            package='sky360_visualizers',
            executable='annotated_frame_provider_node',
            name='annotated_frame_provider_node',
            output='screen'
        ),
    ])