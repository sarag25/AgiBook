"""
Script for Blender 5.1.2 to build the "books on table" scene with every book lying flat, front cover facing up.
Uses table/table_scene_builder.py (shared with create_table_scene_retro.py) and saves table_scene_cover.blend
next to this file; setup_table_camera.py can then photograph it from above.
"""

import importlib
import math
import os
import sys

import bpy      # import Blender Python API

# bpy.path.abspath() resolves the "//" prefix (path relative to the .blend), os.path.abspath() does not
SCRIPT_DIR = os.path.dirname(bpy.path.abspath(bpy.context.space_data.text.filepath))
for _sub in ("table", "books"):
    _dir = os.path.join(SCRIPT_DIR, _sub)
    if _dir not in sys.path:
        sys.path.append(_dir)

import create_table as table
import create_books as books
import table_scene_builder as builder

for _mod in (table, books, builder):
    importlib.reload(_mod)

BOOKS_IMAGES_DIR = os.path.join(SCRIPT_DIR, "books")
BLEND_OUTPUT_PATH = os.path.join(SCRIPT_DIR, "table_scene_cover.blend")

# Rx(+90 deg): local +Y (cover) -> world +Z, book lying flat
COVER_UP_ROTATION = (math.pi / 2.0, 0.0, 0.0)


def main():
    """
    Clear the scene, build a table sized to fit all the books in a grid, place them cover-up,
    then run and bake gravity so the saved scene is already settled
    """
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    table_width, table_depth = builder.table_footprint(books.BOOKS)

    table.clear_scene()
    table.build_table(width=table_width, depth=table_depth)

    book_objs = builder.place_books_on_table(
        books, BOOKS_IMAGES_DIR, table.TABLE_HEIGHT, COVER_UP_ROTATION)
    builder.settle_physics(book_objs)

    print(f"\nScene assembled: table ({table_width:.3f} x {table_depth:.3f} m) + "
          f"{len(book_objs)} books, cover-up. Gravity already settled (baked).")

    bpy.ops.wm.save_as_mainfile(filepath=BLEND_OUTPUT_PATH)
    print(f"Blend file saved to: {BLEND_OUTPUT_PATH}")


if __name__ == "__main__":
    main()
