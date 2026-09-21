"""
Launch file that spawns the bookshelf and books in Gazebo with a reproducible random layout (BookPlacer).
Requires Gazebo already running.
    ros2 launch agibot_x2_pkg spawn_books.launch.py
    ros2 launch agibot_x2_pkg spawn_books.launch.py book_seed:=99
    ros2 launch agibot_x2_pkg spawn_books.launch.py \\
        book_seed:=7 shelf_x:=1.5 shelf_y:=0.0 shelf_yaw:=0.0
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


def _spawn_node(entity_name: str, urdf_path: str,
                x: float, y: float, z: float, yaw: float) -> Node:
    """
    ros_gz_sim create node that spawns one URDF model at (x, y, z, yaw)
    """
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


def _spawn_all(context, *args, **kwargs):
    """
    Resolve the launch arguments at runtime and return one spawn node per object
    """
    pkg = get_package_share_directory("agibot_x2_pkg")

    # read arguments
    seed      = int(LaunchConfiguration("book_seed").perform(context))
    shelf_x   = float(LaunchConfiguration("shelf_x").perform(context))
    shelf_y   = float(LaunchConfiguration("shelf_y").perform(context))
    shelf_yaw = math.radians(
        float(LaunchConfiguration("shelf_yaw_deg").perform(context))
    )

    actions = []

    # 1. spawn the bookshelf
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

    # 2. generate the book layout
    placer = BookPlacer(
        seed=seed,
        shelf_x=shelf_x,
        shelf_y=shelf_y,
        shelf_yaw=shelf_yaw,
    )
    placements = placer.generate()

    # 3. spawn every book (generated URDFs written to temp files, the spawner needs a file)
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
        f"\n[spawn_books] seed={seed}  bookshelf=({shelf_x:.2f},{shelf_y:.2f})  "
        f"books={len(placements)}\n"
    )

    return actions


def generate_launch_description():
    """
    Declare the seed and bookshelf pose arguments and defer spawning to _spawn_all
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "book_seed",
            default_value="42",
            description="Seed of the random layout (same seed = same layout)",
        ),
        DeclareLaunchArgument(
            "shelf_x",
            default_value="0.0",
            description="Bookshelf X position in the world frame (m)",
        ),
        DeclareLaunchArgument(
            "shelf_y",
            default_value="0.0",
            description="Bookshelf Y position in the world frame (m)",
        ),
        DeclareLaunchArgument(
            "shelf_yaw_deg",
            default_value="0.0",
            description="Bookshelf rotation about Z (deg)",
        ),

        OpaqueFunction(function=_spawn_all),
    ])
