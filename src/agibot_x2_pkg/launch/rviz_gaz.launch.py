import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# corretto il fatto che non trovava le meshes con SetEnvironmentVariable


def generate_launch_description():
    pkg_agibot_x2 = get_package_share_directory('agibot_x2_pkg')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    pkg_parent_dir = os.path.dirname(pkg_agibot_x2)

    world_arg = DeclareLaunchArgument(
        'world', default_value='bookshelf.world',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='x2_hand_gazebo.urdf',
        description='Name of the URDF description to load'
    )

    # Path al file URDF
    urdf_file_path = PathJoinSubstitution([
        pkg_agibot_x2,
        'urdf',
        LaunchConfiguration('model')
    ])

    # SOLUZIONE ERRORE MESHES: Diciamo a Gazebo dove cercare il pacchetto agibot_x2_pkg
    set_gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_parent_dir]
    )

    # Lancia Gazebo con il world
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_agibot_x2,
            'worlds',
            LaunchConfiguration('world')
        ]), TextSubstitution(text=' -r -v -v1 --render-engine ogre')]
        }.items()
    )

    # Lancia RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_agibot_x2, 'launch', 'config.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    # Spawn del robot in Gazebo
    spawn_urdf_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'agibot_x2',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.93', '-Y', '0.0'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Pubblica lo stato del robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', urdf_file_path]),
            'use_sim_time': True,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # GUI per muovere i giunti
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    launchDescriptionObject = LaunchDescription()

    # L'impostazione della variabile d'ambiente deve avvenire PRIMA di lanciare Gazebo
    launchDescriptionObject.add_action(set_gz_model_path)

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_gui_node)

    return launchDescriptionObject
