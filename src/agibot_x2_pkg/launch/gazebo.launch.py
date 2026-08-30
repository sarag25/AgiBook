import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('agibot_x2_pkg')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg, 'urdf', 'x2_hand_gazebo.urdf')

    # world:=bookshelf.world (default, placeholder libreria/tavolo hardcoded)
    # world:=empty.world per spawnare la libreria/tavolo REALI generati da
    # Blender con manual_scene.launch.py, senza sovrapporli al placeholder
    # (vedi Gazebo.md).
    world_arg = DeclareLaunchArgument(
        'world', default_value='bookshelf.world',
        description='Nome del file world in worlds/ da caricare',
    )
    world_file = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])

    # Permette a Gazebo Harmonic di risolvere package:// nei URDF
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_prefix('agibot_x2_pkg'), 'share'),
    )

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # Gazebo Harmonic (Jazzy)
    # world_file è una Substitution (dipende da LaunchConfiguration('world')),
    # va passata come lista concatenata dal sistema di launch a runtime - una
    # f-string qui la stringificherebbe subito al parse, prima che 'world'
    # sia risolto (stesso pattern già usato in rviz_gaz.launch.py).
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-v4 -r '), world_file]
        }.items(),
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

    # Bridge Gazebo Harmonic → ROS2
    # /clock          : necessario per use_sim_time
    # /joint_states   : posizioni dei giunti → robot_state_publisher → /tf
    # /tf e /tf_static: trasformazioni per RViz
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/bookshelf_world/model/x2_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/bookshelf_world/model/x2_robot/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_publisher,
        bridge,
    ])
