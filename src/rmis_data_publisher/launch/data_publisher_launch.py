from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch rosbridge_websocket
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),
        # Launch topic publisher
        Node(
            package='rmis_data_publisher',
            executable='topic_publisher',
            name='topic_publisher',
            output='screen'
        )
    ])