#!/usr/bin/env python3
"""
Launch file that starts MoveIt 2 move_group (OMPL with collision checking) for the AgiBot X2,
attached to the ros2_control controllers already running (no new controllers), plus
planning_scene_builder, which fills the scene with bookshelf, table and perceived objects.
Uses the same x2_hand_gazebo.urdf as the simulation; no IK plugin, goals are always joint goals.
Run it AFTER rviz_gaz_control.launch.py (controllers active):
  ros2 launch agibot_x2_pkg moveit.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Build the MoveIt config and return move_group and planning_scene_builder
    """
    finger_mu = DeclareLaunchArgument('finger_mu', default_value='1.0',
                                      description='same as in the simulation launch')
    scene_builder = DeclareLaunchArgument('scene_builder', default_value='true',
                                          description='start planning_scene_builder')
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
                                              # bookshelf at x ~0.5, table at y ~ -1: small workspace
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
