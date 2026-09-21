"""
Launch file that starts the simulation (rviz_gaz_control.launch.py) with MoveIt 2 move_group
and an RViz2 configured for MoveIt (config from agibot_x2_moveit_config).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        'rviz', default_value='true', description='Start RViz2 configured for MoveIt'
    )

    scene_arg = DeclareLaunchArgument(
        'scene', default_value='grasp_test', description='Scene to load (grasp_test | full)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use Gazebo simulated time'
    )

    # 1. main launch file (Gazebo, robot, ros2_control, bridge, grasp_manager)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'rviz_gaz_control.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scene': LaunchConfiguration('scene'),
            'rviz': 'false',  # base RViz off, the MoveIt one is started below
        }.items(),
    )

    # 2. MoveIt 2 config (URDF from agibot_x2_pkg)
    
    urdf_path = os.path.join(
        get_package_share_directory('agibot_x2_pkg'),
        'urdf',
        'x2_hand_gazebo.urdf'
    )

    # SRDF, kinematics and controllers are loaded from agibot_x2_moveit_config/config/
    moveit_config = (
        MoveItConfigsBuilder("agibot_x2", package_name="agibot_x2_moveit_config")
        .robot_description(file_path=urdf_path)
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 3. move_group server (needed by the RViz2 MoveIt plugin)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # 4. RViz with the MoveIt plugins
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