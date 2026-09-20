#!/usr/bin/env python3
"""
moveit.launch.py - MoveIt 2 per l'AgiBot X2 (2026-09-18).

Avvia move_group (pianificazione OMPL con controllo di collisioni) collegato
ai controller ros2_control GIA' in esecuzione (x2_controllers.yaml: nessun
controller nuovo, vedi config/moveit/moveit_controllers.yaml) e il nodo
planning_scene_builder che riempie la planning scene con libreria, tavolo e
gli oggetti misurati dalla percezione (/tmp/x2_detections.json).

Da lanciare DOPO rviz_gaz_control.launch.py (a controller attivi):

  ros2 launch agibot_x2_pkg moveit.launch.py

Poi pick_test_book usa MoveIt per i tratti liberi (parametro moveit:=true,
default) e resta com'era per la posa di presa (IK propria) e per i tratti
rettilinei, che vengono solo VERIFICATI contro la planning scene.

robot_description: lo stesso URDF/xacro del launch della simulazione
(x2_hand_gazebo.urdf, finger_mu uguale), cosi' i link e i giunti coincidono
con quelli di Gazebo. Nessun plugin di cinematica inversa: i goal sono
sempre a giunti (l'IK resta quella di arm_kinematics.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    finger_mu = DeclareLaunchArgument('finger_mu', default_value='1.0',
                                      description='come nel launch della simulazione')
    scene_builder = DeclareLaunchArgument('scene_builder', default_value='true',
                                          description='avvia planning_scene_builder')
    detections_file = DeclareLaunchArgument('detections_file', default_value='/tmp/x2_detections.json')

    moveit_config = (
        MoveItConfigsBuilder("x2", package_name="agibot_x2_pkg")
        .robot_description(file_path="urdf/x2_hand_gazebo.urdf",
                           mappings={"finger_mu": LaunchConfiguration('finger_mu')})
        .robot_description_semantic(file_path="config/moveit/x2.srdf")
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit/moveit_controllers.yaml",
                              moveit_manage_controllers=False)
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True,
                                              # la libreria e' a x ~0.5, il tavolo a y ~ -1: spazio piccolo
                                              "publish_planning_scene_hz": 2.0}],
    )

    builder = Node(
        package="agibot_x2_pkg_py",
        executable="planning_scene_builder",
        output="screen",
        parameters=[{"use_sim_time": True,
                     "detections_file": LaunchConfiguration('detections_file')}],
        condition=None,
    )

    return LaunchDescription([finger_mu, scene_builder, detections_file, move_group, builder])
