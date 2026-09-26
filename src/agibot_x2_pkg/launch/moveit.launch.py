#!/usr/bin/env python3
"""
Launch file that starts MoveIt 2 move_group (OMPL with collision checking) for the AgiBot X2,
attached to the ros2_control controllers already running (no new controllers), plus
planning_scene_builder, which fills the scene with bookshelf, table and perceived objects.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Include the simulation launch and add move_group and MoveIt RViz
    """

    pkg_share = get_package_share_directory('agibot_x2_pkg')

    # launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false', description='Start RViz2 configured for MoveIt'
    )

    scene_arg = DeclareLaunchArgument(
        'scene', default_value='grasp_test', description='Scene to load (grasp_test | full)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use Gazebo simulated time'
    )

    urdf_path = os.path.join(
        get_package_share_directory('agibot_x2_pkg'),
        'urdf',
        'x2_hand_gazebo.urdf'
    )

    scene_builder_arg = DeclareLaunchArgument(
        'scene_builder', default_value='true', description='start planning_scene_builder'
    )

    detections_file_arg = DeclareLaunchArgument(
        'detections_file', default_value='/tmp/x2_detections.json'
    )

    # Parametri per la simulazione Gazebo da passare a linea di comando, inoltrati a rviz_gaz_control.launch.py
    gz_gui_arg = DeclareLaunchArgument(
        'gz_gui', default_value='true', description='Avvia l\'interfaccia grafica di Gazebo'
    )
    video_arg = DeclareLaunchArgument(
        'video', default_value='false', description='Abilita la registrazione/stream video'
    )

    # 1. main launch file (Gazebo, robot, ros2_control, bridge, grasp_manager)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'rviz_gaz_control.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scene': LaunchConfiguration('scene'),
            'gz_gui': LaunchConfiguration('gz_gui'),  # Inoltrato a Gazebo
            'video': LaunchConfiguration('video'),  # Inoltrato a Gazebo
            'rviz': 'false',  # base RViz off, the MoveIt one is started below
        }.items(),
    )

    # 2. MoveIt 2 config (URDF from agibot_x2_pkg)
    
    moveit_config = (
        MoveItConfigsBuilder("x2", package_name="agibot_x2_pkg")  # package where to search for the moveit config standard files
        .robot_description(file_path=urdf_path)
        .trajectory_execution(moveit_manage_controllers=False)
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # NODI, ritardando l'avvio di 5 secondi in modo che la simulazione gazebo sia già partita

    # 3. move_group server (needed by the RViz2 MoveIt plugin)
    move_group_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            )
        ]
    )

    builder = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="agibot_x2_pkg_py",
                executable="planning_scene_builder",
                output="screen",
                parameters=[{
                    "use_sim_time": LaunchConfiguration('use_sim_time'),
                    "detections_file": LaunchConfiguration('detections_file')
                }],
                condition=IfCondition(LaunchConfiguration('scene_builder')),
            )
        ]
    )

    # 4. RViz with the MoveIt plugins
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
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
                condition=IfCondition(LaunchConfiguration('rviz')),
            )
        ]
    )

    return LaunchDescription([
        rviz_arg,
        scene_arg,
        use_sim_time_arg,
        scene_builder_arg,
        detections_file_arg,
        simulation_launch,
        move_group_node,
        builder,
        rviz_node,
    ])