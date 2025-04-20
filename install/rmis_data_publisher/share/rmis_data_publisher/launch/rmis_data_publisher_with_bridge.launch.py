import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Rosbridge WebSocket bağlantısını başlatıyoruz
        Node(
            package='rosbridge_websocket',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        ),

        # Mesajı gönderecek publisher node'unu başlatıyoruz
        Node(
            package='rmis_data_publisher',
            executable='rmis_publisher',
            name='rmis_publisher',
            output='screen'
        ),
    ])
