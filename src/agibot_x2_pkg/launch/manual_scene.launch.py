"""
Launch file that spawns in Gazebo the bookshelf, books, decorations and the empty staging table
with the EXACT layout photographed in Blender, read from manual_layout.json (export_manual_layout.py).
shelf_x/shelf_y/shelf_yaw_deg must match the SHELF_ORIGIN used in setup_render_camera.py, else the
scene is shifted/rotated w.r.t. the photo; use world:=empty.world to avoid the placeholder shelf/table.
    ros2 launch agibot_x2_pkg manual_scene.launch.py \\
        layout_json:=/path/to/manual_layout.json \\
        shelf_x:=1.5 shelf_y:=0.0 shelf_yaw_deg:=90.0
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agibot_x2_pkg.manual_scene_placer import ManualScenePlacer


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
            "-world", "bookshelf_world",
            # explicit world: auto-detection hangs forever when GZ Transport discovery is broken (Docker/WSL2)
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

    layout_json = LaunchConfiguration("layout_json").perform(context)
    if not layout_json:
        raise RuntimeError(
            "layout_json not specified. Example:\n"
            "  ros2 launch agibot_x2_pkg manual_scene.launch.py "
            "layout_json:=/path/to/manual_layout.json"
        )

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

    # 2. load the manual layout (books + decorations)
    placer = ManualScenePlacer(
        layout_json,
        shelf_x=shelf_x,
        shelf_y=shelf_y,
        shelf_yaw=shelf_yaw,
    )
    placements = placer.generate()

    # 2b. empty staging table, spawned directly like the bookshelf (single pose, ready URDF); absent in old layouts
    table_pose = placer.table_world_pose()
    if table_pose is not None:
        table_x, table_y, table_yaw = table_pose
        table_urdf = os.path.join(pkg, "urdf", "table.urdf")
        actions.append(
            _spawn_node(
                entity_name="table",
                urdf_path=table_urdf,
                x=table_x,
                y=table_y,
                z=0.0,
                yaw=table_yaw,
            )
        )

    # 3. spawn every object
    tmp_dir = tempfile.mkdtemp(prefix="manual_scene_spawn_")

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

    n_books = sum(1 for p in placements if p.kind == "book")
    n_decor = sum(1 for p in placements if p.kind == "decoration")
    table_info = f"table=({table_pose[0]:.2f},{table_pose[1]:.2f})" if table_pose else "table=absent"
    print(
        f"\n[manual_scene] bookshelf=({shelf_x:.2f},{shelf_y:.2f})  "
        f"books={n_books}  decorations={n_decor}  {table_info}  layout={layout_json}\n"
    )

    return actions


def generate_launch_description():
    """
    Declare the layout and bookshelf pose arguments and defer spawning to _spawn_all
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "layout_json",
            default_value="",
            description=(
                "Absolute path to manual_layout.json, saved next to the "
                ".blend file by environment/export_manual_layout.py"
            ),
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
