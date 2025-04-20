import os
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Define package and URDF path
    pkg_rmis_description = FindPackageShare('rmis_description')
    urdf_model_path = PathJoinSubstitution([
        pkg_rmis_description,
        'urdf', 'rmis_robot.urdf'
    ])

    # World file path for Gazebo
    world_file_path = PathJoinSubstitution([
        pkg_rmis_description,
        'worlds', 'empty.sdf'
    ])

    # Set GAZEBO_MODEL_PATH
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', FindPackageShare('rmis_description').find('rmis_description')]
    )

    # Robot description parameter
    params = {
        'robot_description': Command(['xacro ', urdf_model_path]),
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    # Nodes
    rmis_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_config_file = PathJoinSubstitution([
        pkg_rmis_description,
        'rviz', 'robot_view.rviz'
    ])
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Gazebo Classic: Start the simulation
    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        log_cmd=True
    )

    # Gazebo client (for visualization)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rmis_robot',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.2',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        output='screen'
    )

    # Launch description
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_model_path,
            description='Path to the URDF model file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        set_gazebo_model_path,
        rmis_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gazebo,
        gazebo_client,
        spawn_entity
    ])