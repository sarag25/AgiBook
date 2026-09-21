"""
Launch file that shows the AgiBot X2 in front of the bookshelf with books in RViz2.
A single scene URDF (robot + bookshelf + books) on /robot_description avoids TF conflicts.
Default layout: robot at (0, 0) facing +X, bookshelf at (shelf_x=1.5, 0) with yaw=90 deg,
so the open side and the book spines face the robot.
"""

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
    """
    Build the scene URDF from the launch arguments and return the RSP, joint GUI and RViz nodes
    """
    pkg        = get_package_share_directory('agibot_x2_pkg')
    robot_urdf = os.path.join(pkg, 'urdf', 'x2_hand_gazebo.urdf')
    rviz_cfg   = os.path.join(pkg, 'launch', 'config.rviz')

    seed      = int(  LaunchConfiguration('book_seed').perform(context))
    shelf_x   = float(LaunchConfiguration('shelf_x').perform(context))
    shelf_y   = float(LaunchConfiguration('shelf_y').perform(context))
    shelf_yaw = math.radians(float(LaunchConfiguration('shelf_yaw_deg').perform(context)))
    robot_z   = float(LaunchConfiguration('robot_z').perform(context))
    face_yaw  = math.radians(float(LaunchConfiguration('book_face_yaw_deg').perform(context)))

    placer     = BookPlacer(seed=seed, shelf_x=shelf_x, shelf_y=shelf_y, shelf_yaw=shelf_yaw)
    placements = placer.generate()

    scene_urdf = generate_scene_urdf(
        robot_urdf_path=robot_urdf,
        placements=placements,
        shelf_x=shelf_x,
        shelf_y=shelf_y,
        shelf_yaw=shelf_yaw,
        robot_z=robot_z,
        book_face_yaw=face_yaw,
    )

    # save for debugging
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_scene.urdf', delete=False, encoding='utf-8'
    )
    tmp.write(scene_urdf)
    tmp.flush()
    tmp.close()
    print(f'\n[display] seed={seed}  bookshelf=({shelf_x},{shelf_y}) yaw={math.degrees(shelf_yaw):.0f}°  '
          f'face_yaw={math.degrees(face_yaw):.0f}°  robot_z={robot_z}  books={len(placements)}')
    print(f'[display] Scene URDF → {tmp.name}\n')

    return [
        # single scene URDF: robot + bookshelf + books
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': scene_urdf,
                'use_sim_time': False,
            }],
            output='screen',
        ),

        # robot joint sliders
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'use_sim_time': False}],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ]


def generate_launch_description():
    """
    Declare the scene arguments and defer node creation to _setup
    """
    return LaunchDescription([
        DeclareLaunchArgument('book_seed',         default_value='42'),
        DeclareLaunchArgument('shelf_x',           default_value='1.5'),
        DeclareLaunchArgument('shelf_y',           default_value='0.0'),
        DeclareLaunchArgument('shelf_yaw_deg',     default_value='90.0'),
        # pelvis at 0.64 m: ankle = 0.64 - 0.602 = 0.038 m, feet on the ground
        DeclareLaunchArgument('robot_z',           default_value='0.64'),
        # 90 deg: Rz(90)*Rx(90) turns the spine (GLB +X) towards the robot
        DeclareLaunchArgument('book_face_yaw_deg', default_value='90.0'),
        OpaqueFunction(function=_setup),
    ])
