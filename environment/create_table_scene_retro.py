"""
Script for Blender 5.1.2 to build the "books on table" scene, retro-up
variant: a wooden table sized for the AgiBot X2 (see
create_table.TABLE_HEIGHT), with every book from create_books.BOOKS lying
flat on top, retro cover facing up.

Sibling entry point: environment/create_table_scene_cover.py builds the
same scene with the covers facing up instead. Both are thin orchestrators
around table/table_scene_builder.py (grid layout, placement, gravity
settle/bake) - only the rotation and the output filename differ, see that
module for the shared logic.

Separate entry point from create_scene.py (bookshelf scene): different
furniture, different book orientation, nothing in common to share besides
create_books.create_book() itself. Same reasoning already applied to
create_gripper.py (see Blender.md): a distinct scene/purpose gets its own
top-level script instead of a branch inside an existing one.

Uso: apri questo script nello Scripting tab di Blender ed esegui (Run
Script / Alt+P). Salva table_scene_retro.blend accanto a questo file.
Per fotografare la scena dall'alto, apri poi setup_table_camera.py (stesso
script, riusabile anche per create_table_scene_cover.py) ed eseguilo.
"""

import importlib
import math
import os
import sys

import bpy      # import Blender Python API

# bpy.path.abspath() (non os.path.abspath()) perche' text.filepath puo'
# essere un percorso relativo al .blend corrente (prefisso "//", convenzione
# di Blender quando il .blend e' gia' stato salvato prima di aprire questo
# script dal file browser): os.path.abspath() non sa risolvere "//" e su
# Windows lo confonde per un percorso UNC, producendo un SCRIPT_DIR
# spazzatura (e di conseguenza un RuntimeError "Cannot read ...jpg" da
# bpy.data.images.load in create_books.load_image).
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
BLEND_OUTPUT_PATH = os.path.join(SCRIPT_DIR, "table_scene_retro.blend")

# Retro is the -Y local face, cover is +Y (see create_books.assign_materials).
# Rx(-90 deg) sends local +Y -> world -Z, i.e. local -Y (retro) -> world +Z
# (retro facing up); local +Z (book height) -> world +Y, so the book ends
# up lying flat with its height axis horizontal. Unaffected by the "manga"
# flag: that only swaps which X face is the spine, not the Y (cover/retro)
# faces this rotation acts on.
RETRO_UP_ROTATION = (-math.pi / 2.0, 0.0, 0.0)


def main():
    """
    Clear the scene, build a table sized to fit all 15 books in a 5x3
    grid, place them retro-up, then run+bake gravity so the saved scene
    is already settled (no manual "Press Play" needed).
    """
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    table_width, table_depth = builder.table_footprint(books.BOOKS)

    table.clear_scene()
    table.build_table(width=table_width, depth=table_depth)

    book_objs = builder.place_books_on_table(
        books, BOOKS_IMAGES_DIR, table.TABLE_HEIGHT, RETRO_UP_ROTATION)
    builder.settle_physics(book_objs)

    print(f"\nScene assembled: table ({table_width:.3f} x {table_depth:.3f} m) + "
          f"{len(book_objs)} books, retro-up. Gravity already settled (baked).")

    bpy.ops.wm.save_as_mainfile(filepath=BLEND_OUTPUT_PATH)
    print(f"Blend file saved to: {BLEND_OUTPUT_PATH}")


if __name__ == "__main__":
    main()
