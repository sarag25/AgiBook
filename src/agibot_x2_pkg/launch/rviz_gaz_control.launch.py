import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg = get_package_share_directory('agibot_x2_pkg')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='config.rviz',
        description='RViz config file'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='bookshelf.world',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='x2_hand_gazebo.urdf',
        description='Name of the URDF description to load'
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='x coordinate of spawned robot'
    )
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='y coordinate of spawned robot'
    )
    z_arg = DeclareLaunchArgument(
        'z', default_value='1.04',
        description='z coordinate of spawned robot'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='yaw angle of spawned robot'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([pkg, "urdf", LaunchConfiguration('model')])

    gz_bridge_params_path = os.path.join(pkg, 'config', 'gz_bridge.yaml')
    
    robot_controllers = PathJoinSubstitution([pkg, 'config', 'x2_controllers.yaml',])
    
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')]),
                TextSubstitution(text=' -r -v -v1 --render-engine ogre')]
        }.items()
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'config',
            LaunchConfiguration('rviz_config')]
            ),],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mogi_arm",
            "-topic", "robot_description",
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'), "-z", LaunchConfiguration('z'), "-Y",
            LaunchConfiguration('yaw') # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
   
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_arm_controller',
            'right_arm_controller',
            'head_controller',
            'waist_controller',
            '--param-file', robot_controllers,
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # SOLUZIONE ERRORE MESHES: Diciamo a Gazebo dove cercare il pacchetto agibot_x2_pkg
    set_gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        #value=[pkg_bme_ros2_agibot_x2]  # punta a install/agibot_x2_pkg/share/agibot_x2_pkg
        value=os.path.dirname(pkg)  # punta a install/agibot_x2_pkg/share
    )

    # Evento: Esegui i controller delle articolazioni solo DOPO che il broadcaster è terminato con successo
    delay_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    launchDescriptionObject = LaunchDescription()

    # L'impostazione della variabile d'ambiente deve avvenire PRIMA di lanciare Gazebo
    launchDescriptionObject.add_action(set_gz_model_path)

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(z_arg)
    launchDescriptionObject.add_action(yaw_arg)
    launchDescriptionObject.add_action(sim_time_arg)

    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_gui_node)


    #launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    #launchDescriptionObject.add_action(joint_trajectory_controller_spawner)
    # Avviamo solo il broadcaster
    launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    # L'altro spawner verrà chiamato in automatico subito dopo!
    launchDescriptionObject.add_action(delay_controllers_spawner)
    
    return launchDescriptionObject