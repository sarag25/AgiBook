import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agibot_x2_pkg.book_placer import BookPlacer, generate_scene_urdf


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('agibot_x2_pkg')

    seed      = int(LaunchConfiguration('book_seed').perform(context))
    shelf_x   = float(LaunchConfiguration('shelf_x').perform(context))
    shelf_y   = float(LaunchConfiguration('shelf_y').perform(context))
    shelf_yaw = math.radians(float(LaunchConfiguration('shelf_yaw_deg').perform(context)))
    robot_z   = float(LaunchConfiguration('robot_z').perform(context))

    robot_urdf = os.path.join(pkg, 'urdf', 'x2_hand_gazebo.urdf')
    rviz_config = os.path.join(pkg, 'launch', 'config.rviz')

    # Genera placements e URDF scena
    placer = BookPlacer(seed=seed, shelf_x=shelf_x, shelf_y=shelf_y, shelf_yaw=shelf_yaw)
    placements = placer.generate()
    scene_urdf = generate_scene_urdf(robot_urdf, placements, shelf_x, shelf_y, shelf_yaw, robot_z)

    # Scrivi URDF su file temporaneo
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_scene.urdf', delete=False, encoding='utf-8'
    )
    tmp.write(scene_urdf)
    tmp.flush()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': scene_urdf}],
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return [robot_state_publisher, joint_state_publisher_gui, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('book_seed',     default_value='42'),
        DeclareLaunchArgument('shelf_x',       default_value='1.0'),
        DeclareLaunchArgument('shelf_y',       default_value='0.0'),
        DeclareLaunchArgument('shelf_yaw_deg', default_value='180.0'),
        DeclareLaunchArgument('robot_z',       default_value='0.93'),

        OpaqueFunction(function=_setup),
    ])
