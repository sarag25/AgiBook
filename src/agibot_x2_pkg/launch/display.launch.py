"""
display.launch.py
=================
Visualizza in RViz2: robot (Agibot X2) davanti alla libreria con i libri.

Architettura (URDF unico, nessun conflitto TF):
  rsp            → URDF scena completa (robot + libreria + libri)
                   → /robot_description  → TF completo
  joint_state_pub_gui → slider joint robot
  rviz2

Layout di default:
  - Robot a (0, 0) che guarda +X, pelvis a z=robot_z
  - Libreria a (shelf_x=1.5, 0) con yaw=90°
    → lato aperto verso -X (verso il robot)
    → spine dei libri visibili al robot
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
    pkg        = get_package_share_directory('agibot_x2_pkg')
    robot_urdf = os.path.join(pkg, 'urdf', 'x2_hand_gazebo.urdf')
    rviz_cfg   = os.path.join(pkg, 'launch', 'config.rviz')

    seed      = int(LaunchConfiguration('book_seed').perform(context))
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

    # Salva per debug
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_scene.urdf', delete=False, encoding='utf-8'
    )
    tmp.write(scene_urdf)
    tmp.flush()
    tmp.close()
    print(f'\n[display] seed={seed}  libreria=({shelf_x},{shelf_y}) yaw={math.degrees(shelf_yaw):.0f}°  '
          f'face_yaw={math.degrees(face_yaw):.0f}°  robot_z={robot_z}  libri={len(placements)}')
    print(f'[display] Scene URDF → {tmp.name}\n')

    return [
        # ── URDF scena unico: robot + libreria + libri ────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': scene_urdf,
                'use_sim_time': False,
            }],
            output='screen',
        ),

        # ── slider joint robot ─────────────────────────────────────────────────
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'use_sim_time': False}],
        ),

        # ── RViz2 ──────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('book_seed', default_value='42'),
        DeclareLaunchArgument('shelf_x', default_value='1.5'),
        DeclareLaunchArgument('shelf_y', default_value='0.0'),
        DeclareLaunchArgument('shelf_yaw_deg', default_value='90.0'),
        # pelvis a 0.64 m: ankle = 0.64 - 0.602 = 0.038 m → piedi a terra
        DeclareLaunchArgument('robot_z', default_value='0.64'),
        # book_face_yaw_deg: 90° → Rz(90°)*Rx(90°) → dorso (GLB +X) verso robot
        DeclareLaunchArgument('book_face_yaw_deg', default_value='90.0'),
        OpaqueFunction(function=_setup),
    ])
