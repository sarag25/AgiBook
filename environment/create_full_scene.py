"""
Script for Blender 5.1.2 to build the complete scene handed off to Gazebo:
the library (bookshelf + books + decorations) plus an empty table at the robot's right.
The robot has a fixed base in Gazebo, so everything is kept within arm reach.
Gravity is baked before saving full_scene.blend next to this file.
"""

import importlib
import os
import sys

import bpy      # import Blender Python API

# bpy.path.abspath(): text.filepath can be relative to the .blend ("//" prefix)
SCRIPT_DIR = os.path.dirname(bpy.path.abspath(bpy.context.space_data.text.filepath))
for _sub in ("bookshelf", "books", "desk_decorations", "table", "library"):
    _dir = os.path.join(SCRIPT_DIR, _sub)
    if _dir not in sys.path:
        sys.path.append(_dir)
if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)

import create_bookshelf as bookshelf
import create_table as table
import library_scene_builder as library
import scene_physics as physics

for _mod in (bookshelf, table, library, physics):
    importlib.reload(_mod)

BOOKS_IMAGES_DIR = os.path.join(SCRIPT_DIR, "books")
BLEND_OUTPUT_PATH = os.path.join(SCRIPT_DIR, "full_scene.blend")

LIBRARY_CONFIG = "classic"   # same options as create_scene.ACTIVE_CONFIG
RANDOM_SEED = 1

# Layout in the local frame of the bookshelf (built at the origin, front towards +Y).
# The robot faces -Y (towards the shelf), so its right is local -X.
# Distances are kept small so the ~0.50 m arm+hand span reaches both shelf and table.
ROBOT_TO_SHELF_DISTANCE = 0.25     # robot stand-off from the shelf front
# Far enough that the table footprint clears the shelf Y-span
TABLE_LATERAL_DISTANCE = 1.05      # table center, to the robot's right (local -X)
TABLE_TO_SHELF_DISTANCE = 0.65     # table center, in front of the shelf

TABLE_WIDTH = table.DEFAULT_WIDTH  # 1.20 m, must match the exported table.glb
TABLE_DEPTH = table.DEFAULT_DEPTH  # 0.90 m

# Near shoulder height for the fixed-base robot (table.TABLE_HEIGHT stays 0.50 m for the photo scenes)
GAZEBO_TABLE_HEIGHT = 0.75

ROBOT_POSITION = (0.0, library.SHELF_FRONT_Y + ROBOT_TO_SHELF_DISTANCE, 0.0)
TABLE_POSITION = (
    ROBOT_POSITION[0] - TABLE_LATERAL_DISTANCE,
    library.SHELF_FRONT_Y + TABLE_TO_SHELF_DISTANCE,
    0.0,
)


def add_robot_reference(location):
    """
    Add a plain-axes Empty marking where the robot stands.
    Its name has no "shelf_"/"table_" prefix, so the exporters never pick it up.
    """
    empty = bpy.data.objects.new("robot_reference", None)
    empty.empty_display_type = "PLAIN_AXES"
    empty.empty_display_size = 0.3
    empty.location = location
    bpy.context.collection.objects.link(empty)
    return empty


def place_table(location):
    """
    Build the empty table at the origin and translate all its parts to location
    """
    parts = table.build_table(width=TABLE_WIDTH, depth=TABLE_DEPTH, height=GAZEBO_TABLE_HEIGHT)
    for obj in parts:
        obj.location.x += location[0]
        obj.location.y += location[1]
        obj.location.z += location[2]
    return parts


def main():
    """
    Build library, table and robot marker, bake gravity and save the .blend
    """
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    bookshelf.clear_scene()

    label, library_objects = library.build_library(
        BOOKS_IMAGES_DIR, config_name=LIBRARY_CONFIG, seed=RANDOM_SEED)

    place_table(TABLE_POSITION)
    add_robot_reference(ROBOT_POSITION)

    physics.settle_physics(library_objects)

    print(f"\nScene assembled [{label}]: bookshelf + {len(library_objects)} "
          f"books/decorations + empty table at "
          f"({TABLE_POSITION[0]:.2f}, {TABLE_POSITION[1]:.2f}). Robot "
          f"reference at ({ROBOT_POSITION[0]:.2f}, {ROBOT_POSITION[1]:.2f}). "
          "Gravity baked.")

    bpy.ops.wm.save_as_mainfile(filepath=BLEND_OUTPUT_PATH)
    print(f"Blend file saved to: {BLEND_OUTPUT_PATH}")


if __name__ == "__main__":
    main()
