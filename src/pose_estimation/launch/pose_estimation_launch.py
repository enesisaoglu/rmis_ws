from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_estimation',
            executable='image_processing_node',
            name='pose_detection_node',
            output='screen'
        ),
        Node(
            package='pose_estimation',
            executable='rmis_mimic_node',
            name='image_sync_node',
            output='screen'
        ),
    ])