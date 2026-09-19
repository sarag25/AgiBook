import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('agibot_x2_pkg')

    # Argomenti di Launch
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Avvia RViz2 configurato con MoveIt'
    )

    scene_arg = DeclareLaunchArgument(
        'scene', default_value='grasp_test', description='Scena da caricare (grasp_test | full)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Usa il tempo simulato di Gazebo'
    )

    # 1. Inclusione del launch file principale (Gazebo, robot, ros2_control, bridge, grasp_manager)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'rviz_gaz_control.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scene': LaunchConfiguration('scene'),
            'rviz': 'false',  # Disabilitiamo l'RViz base per lanciare quello con MoveIt
        }.items(),
    )

    # 2. Configurazione MoveIt 2 per AgiBot X2
    
    # Percorso assoluto dell'URDF dal pacchetto agibot_x2_pkg
    urdf_path = os.path.join(
        get_package_share_directory('agibot_x2_pkg'),
        'urdf',
        'x2_hand_gazebo.urdf'
    )

    # MoveItConfigsBuilder caricherà automaticamente SRDF, kinematics e controllers
    # cercandoli in agibot_x2_moveit_config/config/
    moveit_config = (
        MoveItConfigsBuilder("agibot_x2", package_name="agibot_x2_moveit_config")
        .robot_description(file_path=urdf_path)
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 3. Server move_group (Necessario per RViz2, gestisce cinematica, planning scene e collision avoidance)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # 4. RViz con i plugin e la configurazione di MoveIt
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'config.rviz'])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        rviz_arg,
        scene_arg,
        use_sim_time_arg,
        simulation_launch,
        move_group_node,
        rviz_node,
    ])