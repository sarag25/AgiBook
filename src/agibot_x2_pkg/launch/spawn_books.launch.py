"""
spawn_books.launch.py
=====================
Spawna la libreria e i libri in Gazebo Classic con layout casuale riproducibile.

Uso:
    # layout di default (seed 42, libreria all'origine)
    ros2 launch agibot_x2_pkg spawn_books.launch.py

    # layout diverso
    ros2 launch agibot_x2_pkg spawn_books.launch.py book_seed:=99

    # libreria spostata nel world
    ros2 launch agibot_x2_pkg spawn_books.launch.py \\
        book_seed:=7 shelf_x:=1.5 shelf_y:=0.0 shelf_yaw:=0.0

Precondizioni:
    - Gazebo Classic già avviato (es. tramite gazebo.launch.py)
    - Il package agibot_x2_pkg è nel workspace compilato
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agibot_x2_pkg.book_placer import BookPlacer


# ─────────────────────────────────────────────────────────────────────────────
# Helper: nodo spawn_entity per un singolo modello
# ─────────────────────────────────────────────────────────────────────────────
def _spawn_node(entity_name: str, urdf_path: str,
                x: float, y: float, z: float, yaw: float) -> Node:
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{entity_name}",
        arguments=[
            "-name",  entity_name,
            "-file",  urdf_path,
            "-x",     str(round(x,   4)),
            "-y",     str(round(y,   4)),
            "-z",     str(round(z,   4)),
            "-Y",     str(round(yaw, 4)),
        ],
        output="screen",
    )


# ─────────────────────────────────────────────────────────────────────────────
# OpaqueFunction: viene eseguita a runtime, con accesso ai LaunchConfig
# ─────────────────────────────────────────────────────────────────────────────
def _spawn_all(context, *args, **kwargs):
    pkg = get_package_share_directory("agibot_x2_pkg")

    # ── leggi argomenti ──────────────────────────────────────────────────────
    seed      = int(LaunchConfiguration("book_seed").perform(context))
    shelf_x   = float(LaunchConfiguration("shelf_x").perform(context))
    shelf_y   = float(LaunchConfiguration("shelf_y").perform(context))
    shelf_yaw = math.radians(
        float(LaunchConfiguration("shelf_yaw_deg").perform(context))
    )

    actions = []

    # ── 1. Spawna la libreria ────────────────────────────────────────────────
    bookshelf_urdf = os.path.join(pkg, "urdf", "bookshelf.urdf")
    actions.append(
        _spawn_node(
            entity_name="bookshelf",
            urdf_path=bookshelf_urdf,
            x=shelf_x,
            y=shelf_y,
            z=0.0,
            yaw=shelf_yaw,
        )
    )

    # ── 2. Genera il layout dei libri ────────────────────────────────────────
    placer = BookPlacer(
        seed=seed,
        shelf_x=shelf_x,
        shelf_y=shelf_y,
        shelf_yaw=shelf_yaw,
    )
    placements = placer.generate()

    # ── 3. Spawna ogni libro ─────────────────────────────────────────────────
    #
    # spawn_entity.py accetta un file URDF.
    # I libri non hanno tutti un file .urdf installato, quindi scriviamo il
    # contenuto URDF generato dinamicamente su un file temporaneo per ciascuno.
    # I file sono creati nella cartella temporanea del sistema e rimangono
    # validi per tutta la durata del processo di launch.
    #
    tmp_dir = tempfile.mkdtemp(prefix="bookshelf_spawn_")

    for p in placements:
        tmp_path = os.path.join(tmp_dir, f"{p.name}.urdf")
        with open(tmp_path, "w", encoding="utf-8") as f:
            f.write(p.urdf)

        actions.append(
            _spawn_node(
                entity_name=p.name,
                urdf_path=tmp_path,
                x=p.x,
                y=p.y,
                z=p.z,
                yaw=p.yaw,
            )
        )

    print(
        f"\n[spawn_books] seed={seed}  libreria=({shelf_x:.2f},{shelf_y:.2f})  "
        f"libri={len(placements)}\n"
    )

    return actions


# ─────────────────────────────────────────────────────────────────────────────
# LaunchDescription
# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "book_seed",
            default_value="42",
            description="Seed per il layout casuale (stesso seed = stesso layout)",
        ),
        DeclareLaunchArgument(
            "shelf_x",
            default_value="0.0",
            description="Posizione X della libreria nel world frame (metri)",
        ),
        DeclareLaunchArgument(
            "shelf_y",
            default_value="0.0",
            description="Posizione Y della libreria nel world frame (metri)",
        ),
        DeclareLaunchArgument(
            "shelf_yaw_deg",
            default_value="0.0",
            description="Rotazione della libreria attorno Z (gradi)",
        ),

        OpaqueFunction(function=_spawn_all),
    ])
