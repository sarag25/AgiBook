import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('agibot_x2_pkg')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg, 'urdf', 'x2_hand_gazebo.urdf')
    world_file = os.path.join(pkg, 'worlds', 'bookshelf.world')

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # Gazebo Harmonic (Jazzy)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-v4 -r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # Spawn robot tramite service ros_gz_sim (Harmonic)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'x2_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.93',
        ],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_publisher,
    ])
