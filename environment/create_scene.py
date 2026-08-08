"""
Script for Blender 5.1.2 to build the complete library scene:
the bookshelf, all the books and the desk decorations placed on the shelves

Thin entry point: the shelf-filling logic (configurations, book/decoration
placement maths, config validation) lives in
library/library_scene_builder.py, shared with create_full_scene.py
(library + empty table) so the two don't duplicate ~200 lines of layout
code - same pattern already used by table/table_scene_builder.py for the
two table-scene entry points.
"""

import importlib
import os
import sys

import bpy      # import Blender Python API

SCRIPT_DIR = os.path.dirname(os.path.abspath(bpy.context.space_data.text.filepath))
for _sub in ("bookshelf", "books", "desk_decorations", "library"):
    _dir = os.path.join(SCRIPT_DIR, _sub)
    if _dir not in sys.path:
        sys.path.append(_dir)
if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)

import create_bookshelf as bookshelf
import library_scene_builder as library
import scene_physics as physics

for _mod in (bookshelf, library, physics):
    importlib.reload(_mod)

BOOKS_IMAGES_DIR = os.path.join(SCRIPT_DIR, "books")
BLEND_OUTPUT_PATH = os.path.join(SCRIPT_DIR, "library_scene.blend")

ACTIVE_CONFIG = "classic"   # "classic", "flipped", "by_height", "mixed" or "random"
RANDOM_SEED = 1             # only used when ACTIVE_CONFIG == "random"


def main():
    """
    Clear the scene, build the bookshelf and fill every shelf following
    the layout selected by ACTIVE_CONFIG: books (spine out) plus
    decorations beside and in front of the rows. Gravity is baked
    (scene_physics.settle_physics) before saving, so the scene is already
    settled on load - no manual "Press Play" needed.
    """
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    bookshelf.clear_scene()
    label, objects = library.build_library(
        BOOKS_IMAGES_DIR, config_name=ACTIVE_CONFIG, seed=RANDOM_SEED)
    physics.settle_physics(objects)

    print(f"\nScene assembled [{label}]: bookshelf + {len(objects)} books/decorations. "
          "Gravity already settled (baked).")

    bpy.ops.wm.save_as_mainfile(filepath=BLEND_OUTPUT_PATH)
    print(f"Blend file saved to: {BLEND_OUTPUT_PATH}")


if __name__ == "__main__":
    main()
