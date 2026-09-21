"""
Script for Blender 5.1.2 to export the manual poses of books, decorations and staging table to JSON.
Gazebo loads it (manual_scene_placer.py) instead of the random BookPlacer layout, so the
spawned scene matches the one rendered by setup_render_camera.py.
Usage: open the .blend with the objects placed and run it in the Scripting tab
(same session, native Z-up frame). Writes manual_layout.json next to the .blend file.
"""
import json
import os

import bpy      # import Blender Python API

# Blender world point matching the local (0,0,0) of book_placer.py; must match SHELF_ORIGIN in setup_render_camera.py
SHELF_ORIGIN = (0.0, 0.0, 0.0)

OUTPUT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(bpy.data.filepath)), "manual_layout.json"
)

# Must match the BOOK_CATALOG keys in book_placer.py
BOOK_KEYS = [
    "alba_mietitura_book", "ballata_usignolo_book", "black_widow_book",
    "cane_stelle_book", "cane_stelle_racconti_book", "cats_cradle_book",
    "emma_book", "enciclopedia_animali_book", "enciclopedia_terra_vol1_book",
    "enciclopedia_terra_vol2_book", "fantasticos_4_book", "hunger_games_book",
    "it_book", "never_flinch_book", "werther_book",
]

# Must match DECORATIONS in create_desk_decorations.py and DECOR_CATALOG in manual_scene_placer.py
DECOR_KEYS = [
    "pen_holder", "paperweight_ball", "desk_globe",
    "potted_plant", "coffee_mug", "coaster",
]

# Top and 4 legs are moved as one rigid block: the top pose represents the whole table
TABLE_TOP_NAME = "table_top"

# Name -> (object_key, kind); the table key is "table", used to find urdf/table.urdf
OBJECT_KEY_KIND = {
    **{k: (k, "book") for k in BOOK_KEYS},
    **{k: (k, "decoration") for k in DECOR_KEYS},
    TABLE_TOP_NAME: ("table", "table"),
}


def match_object_key(obj_name: str):
    """
    Return (object_key, kind) for the first matching name (also "name.001" duplicates), or (None, None)
    """
    for match_name, (object_key, kind) in OBJECT_KEY_KIND.items():
        if obj_name == match_name or obj_name.startswith(match_name + "."):
            return object_key, kind
    return None, None


def main():
    """
    Collect the local pose (x, y, z, yaw) of every known object and write it to OUTPUT_PATH.
    The bookshelf rotation is 0°, so the local frame is the Blender world frame
    shifted by SHELF_ORIGIN
    """
    ox, oy, oz = SHELF_ORIGIN
    entries = []

    for obj in bpy.data.objects:
        object_key, kind = match_object_key(obj.name)
        if object_key is None:
            continue

        loc = obj.matrix_world.translation
        yaw = obj.matrix_world.to_euler('XYZ').z

        entries.append({
            "object_key": object_key,
            "kind": kind,
            "source_object": obj.name,
            "local_x": round(loc.x - ox, 5),
            "local_y": round(loc.y - oy, 5),
            "local_z": round(loc.z - oz, 5),
            "local_yaw": round(yaw, 5),
        })

    if not entries:
        print("No book/decoration found: check BOOK_KEYS/DECOR_KEYS and the object names.")
        return

    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump(entries, f, indent=2, ensure_ascii=False)

    n_books = sum(1 for e in entries if e["kind"] == "book")
    n_decor = sum(1 for e in entries if e["kind"] == "decoration")
    n_table = sum(1 for e in entries if e["kind"] == "table")
    print(f"Exported {len(entries)} objects ({n_books} books, {n_decor} decorations, "
          f"{n_table} table) to: {OUTPUT_PATH}")
    for e in entries:
        print(
            f"  {e['source_object']:<30} -> {e['object_key']:<25} [{e['kind']:<10}] "
            f"x={e['local_x']:.3f} y={e['local_y']:.3f} z={e['local_z']:.3f} "
            f"yaw={e['local_yaw']:.3f}"
        )


if __name__ == "__main__":
    main()
