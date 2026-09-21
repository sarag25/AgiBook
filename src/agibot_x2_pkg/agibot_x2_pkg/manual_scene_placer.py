"""
Helpers that turn manual_layout.json (from environment/export_manual_layout.py) into world-frame
placements for books and decorations, so the Gazebo scene matches the one photographed in Blender.
Fixed-layout alternative to BookPlacer: uses the real Blender poses instead of a random layout.
Usage: placements = ManualScenePlacer("manual_layout.json", shelf_x=1.5, shelf_y=0.0, shelf_yaw=math.pi / 2).generate()
"""

from __future__ import annotations

import json
import math
import textwrap
from dataclasses import dataclass
from typing import List

from agibot_x2_pkg.book_placer import BOOK_CATALOG

# desk decorations: "size" is an approximate (x, y, z) bounding box in m from create_desk_decorations.py,
# used only for the URDF collision box (the visual is the real .glb)
DECOR_CATALOG: dict[str, dict] = {
    "pen_holder":       {"size": (0.056, 0.056, 0.095), "mass": 0.12},
    "paperweight_ball": {"size": (0.064, 0.064, 0.064), "mass": 0.15},
    "desk_globe":       {"size": (0.080, 0.080, 0.122), "mass": 0.28},
    "potted_plant":     {"size": (0.100, 0.100, 0.140), "mass": 0.22},
    "coffee_mug":       {"size": (0.090, 0.075, 0.090), "mass": 0.20},
    "coaster":          {"size": (0.100, 0.100, 0.008), "mass": 0.05},
}

_MESH_DIR = {"book": "books", "decoration": "desk_decorations"}


def _catalog_for(kind: str) -> dict:
    """
    BOOK_CATALOG for books, DECOR_CATALOG otherwise
    """
    return BOOK_CATALOG if kind == "book" else DECOR_CATALOG


def read_table_pose(layout_path: str):
    """
    (local_x, local_y, local_yaw) of the "table" entry relative to SHELF_ORIGIN,
    or None if the layout has no table (library-only scenes, not an error)
    """
    with open(layout_path, "r", encoding="utf-8") as f:
        entries = json.load(f)
    for e in entries:
        if e["kind"] == "table":
            return e["local_x"], e["local_y"], e["local_yaw"]
    return None


@dataclass
class ScenePlacement:
    """
    One object to spawn, with world and bookshelf-local pose
    """
    name: str          # unique Gazebo entity name (e.g. "it_book_0")
    kind: str          # "book" | "decoration"
    object_key: str    # key in BOOK_CATALOG or DECOR_CATALOG
    urdf: str          # full URDF XML
    x: float           # world frame (object center)
    y: float
    z: float
    yaw: float         # rad
    local_x: float      # bookshelf local frame, read from Blender
    local_y: float
    local_z: float
    local_yaw: float


def _make_urdf(entity_name: str, kind: str, object_key: str) -> str:
    """
    Inline URDF: visual = real mesh, collision = approximate box, <static>true</static>
    Static because the Blender poses are already at rest; dynamic approximate boxes
    start slightly interpenetrating and the solver throws them off the shelf.
    """
    info = _catalog_for(kind)[object_key]
    sx, sy, sz = info["size"]
    m = info["mass"]
    ixx = m / 12.0 * (sy**2 + sz**2)
    iyy = m / 12.0 * (sx**2 + sz**2)
    izz = m / 12.0 * (sx**2 + sy**2)
    mesh_dir = _MESH_DIR[kind]

    return textwrap.dedent(f"""\
        <?xml version="1.0" encoding="utf-8"?>
        <robot name="{entity_name}">
          <link name="base_link">
            <inertial>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <mass value="{m}"/>
              <inertia ixx="{ixx:.6f}" ixy="0" ixz="0"
                       iyy="{iyy:.6f}" iyz="0"
                       izz="{izz:.6f}"/>
            </inertial>
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <mesh filename="package://agibot_x2_pkg/meshes/{mesh_dir}/{object_key}.glb"/>
              </geometry>
            </visual>
            <collision>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{sx} {sy} {sz}"/>
              </geometry>
            </collision>
          </link>
          <gazebo>
            <static>true</static>
          </gazebo>
        </robot>
    """)


class ManualScenePlacer:
    """
    Places the objects of manual_layout.json around a bookshelf at (shelf_x, shelf_y, shelf_yaw [rad])
    The shelf pose must match the bookshelf.urdf spawn and the SHELF_ORIGIN used by
    setup_render_camera.py, so the scene matches the photos.
    """

    def __init__(
        self,
        layout_path: str,
        shelf_x: float = 0.0,
        shelf_y: float = 0.0,
        shelf_yaw: float = 0.0,
    ):
        """
        Store the layout path and the bookshelf world pose
        """
        self.layout_path = layout_path
        self.shelf_x = shelf_x
        self.shelf_y = shelf_y
        self.shelf_yaw = shelf_yaw

    def generate(self) -> List[ScenePlacement]:
        """
        ScenePlacement list for every book/decoration in manual_layout.json
        """

        with open(self.layout_path, "r", encoding="utf-8") as f:
            entries = json.load(f)

        cos_a = math.cos(self.shelf_yaw)
        sin_a = math.sin(self.shelf_yaw)
        counters: dict[str, int] = {}
        placements: List[ScenePlacement] = []

        for e in entries:
            kind = e["kind"]
            object_key = e["object_key"]

            if kind == "table":
                # spawned separately from urdf/table.urdf via table_world_pose()
                continue

            if object_key not in _catalog_for(kind):
                print(
                    f"  skip {e['source_object']}: '{object_key}' is not in the "
                    f"{kind} catalog (BOOK_CATALOG/DECOR_CATALOG)"
                )
                continue

            local_x, local_y, local_z = e["local_x"], e["local_y"], e["local_z"]
            local_yaw = e["local_yaw"]

            # world frame: 2D rotation about Z, same formula as BookPlacer.generate()
            world_x = self.shelf_x + cos_a * local_x - sin_a * local_y
            world_y = self.shelf_y + sin_a * local_x + cos_a * local_y
            world_z = local_z
            world_yaw = self.shelf_yaw + local_yaw

            counters[object_key] = counters.get(object_key, -1) + 1
            entity_name = f"{object_key}_{counters[object_key]}"

            placements.append(
                ScenePlacement(
                    name=entity_name,
                    kind=kind,
                    object_key=object_key,
                    urdf=_make_urdf(entity_name, kind, object_key),
                    x=world_x,
                    y=world_y,
                    z=world_z,
                    yaw=world_yaw,
                    local_x=local_x,
                    local_y=local_y,
                    local_z=local_z,
                    local_yaw=local_yaw,
                )
            )

        return placements

    def table_world_pose(self):
        """
        World-frame (x, y, yaw) of the staging table from manual_layout.json, or None if absent
        """
        pose = read_table_pose(self.layout_path)
        if pose is None:
            return None

        local_x, local_y, local_yaw = pose
        cos_a = math.cos(self.shelf_yaw)
        sin_a = math.sin(self.shelf_yaw)
        world_x = self.shelf_x + cos_a * local_x - sin_a * local_y
        world_y = self.shelf_y + sin_a * local_x + cos_a * local_y
        world_yaw = self.shelf_yaw + local_yaw
        return world_x, world_y, world_yaw


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Show the manual layout loaded from JSON")
    parser.add_argument("layout_json", help="Path to manual_layout.json")
    parser.add_argument("--shelf-x", type=float, default=0.0)
    parser.add_argument("--shelf-y", type=float, default=0.0)
    parser.add_argument("--shelf-yaw", type=float, default=0.0,
                        help="bookshelf rotation in degrees")
    args = parser.parse_args()

    placer = ManualScenePlacer(
        args.layout_json,
        shelf_x=args.shelf_x,
        shelf_y=args.shelf_y,
        shelf_yaw=math.radians(args.shelf_yaw),
    )
    results = placer.generate()

    print(f"\n{len(results)} objects loaded from {args.layout_json}\n")
    print(f"{'Entità':<25} {'Tipo':<12} {'x':>7} {'y':>7} {'z':>7} {'yaw°':>6}")
    print("-" * 65)
    for p in results:
        print(
            f"{p.name:<25} {p.kind:<12} "
            f"{p.x:>7.3f} {p.y:>7.3f} {p.z:>7.3f} "
            f"{math.degrees(p.yaw):>6.1f}"
        )
