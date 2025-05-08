from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix
import os

def generate_launch_description():
    package_name = 'rmis'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'rmisurdf.urdf')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(pkg_share, 'worlds', 'stable.world')
    controller_file = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Append package share directory to GAZEBO_MODEL_PATH
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'verbose': 'true', 'world': world_file}.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_description}]
        ),
	Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'rmis_robot', '-topic', '/robot_description', '-z', '4']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
