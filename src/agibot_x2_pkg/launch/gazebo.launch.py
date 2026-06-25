import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('x2_description')

    # Percorso URDF e world
    urdf_file = os.path.join(pkg, 'urdf', 'x2_hand_gazebo.urdf')
    world_file = os.path.join(pkg, 'worlds', 'bookshelf.world')

    # Parametro robot_description
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # Avvia Gazebo Classic con il world della libreria
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Pubblica il modello URDF sul param server
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # Spawn del robot in Gazebo (posizione iniziale: centro scena)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'x2_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.93',   # altezza pelvis da terra (robot in piedi)
        ],
        output='screen'
    )

    # Joint State Publisher (per muovere i giunti manualmente in RViz)
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
