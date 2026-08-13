"""
Script for Blender 5.1.2: export the WHOLE assembled scene (bookshelf +
books + decorations + table, whatever create_full_scene.py built) as a
SINGLE .glb file, instead of exporting each object separately.

Why: the per-object pipeline (create_books.py exporting 15 separate .glb
files, then manual_scene_placer.py re-synthesizing a URDF per object from
manual_layout.json + a hand-maintained size/mass catalog) turned out very
fragile in practice - a batch-export bug in create_books.py alone caused
every book to render meters away from its correct position in Gazebo (see
Bugs.md), on top of everything else that has to stay in sync by hand
(shelf_x/y/yaw, BOOK_CATALOG/DECOR_CATALOG box sizes, static flags...).

Exporting the scene exactly as Blender already has it - one file, objects
keeping their relative positions as built - removes all of that: there is
no separate position source to get out of sync with, no per-object export
step that could silently freeze the wrong transform. The trade-off is that
individual books/decorations are no longer separate Gazebo entities (see
Gazebo.md "Scena unica" for how to still isolate one object later, e.g. to
actually grasp it).

Usage: after running create_full_scene.py (or opening full_scene.blend,
already built) in the SAME Blender session, open this script in the
Scripting tab and run it (Run Script / Alt+P). Writes
src/agibot_x2_pkg/meshes/full_scene.glb, overwriting any previous export.

Uses bpy.data.filepath (the currently opened/saved .blend), same as
setup_render_camera.py - NOT bpy.context.space_data.text.filepath, so this
script works whether it was pasted into a fresh text block or opened from
disk after the .blend was already saved (see the create_table_scene_retro.py
bug in Bugs.md for why that distinction matters).
"""
import os

import bpy


def default_output_dir():
    """
    meshes/ directory of the ROS2 package, resolved from the currently
    open .blend's location (environment/full_scene.blend -> ../../src/...).
    """
    blend_dir = os.path.dirname(bpy.path.abspath(bpy.data.filepath))
    return os.path.normpath(
        os.path.join(blend_dir, "..", "src", "agibot_x2_pkg", "meshes")
    )


# Objects with these exact names are excluded from the export: not part of
# the physical scene. "robot_reference" is a PLAIN_AXES Empty (no mesh) -
# excluded explicitly anyway, in case a future version of create_full_scene.py
# gives it actual geometry.
EXCLUDE_NAMES = {"robot_reference"}


def pack_all_images():
    """
    Make sure every image used by a material in the scene is either backed
    by a real file on disk (books' JPGs, loaded with a valid path by
    create_books.py) or packed into the .blend (bookshelf/table's
    procedural wood textures, generated in-memory by make_wood_image() -
    no file to read from, must be packed or the export silently drops
    them). Safe to call even if already packed/on-disk.
    """
    for image in bpy.data.images:
        if image.source == "FILE" and not image.packed_file and image.filepath:
            continue  # real file on disk, exporter will read it directly
        if not image.packed_file and image.has_data:
            image.pack()


def main():
    mesh_objects = [
        obj for obj in bpy.data.objects
        if obj.type == "MESH" and obj.name not in EXCLUDE_NAMES
    ]
    if not mesh_objects:
        print("Nessun oggetto mesh trovato nella scena: hai eseguito "
              "create_full_scene.py in questa sessione prima di questo script?")
        return

    pack_all_images()

    bpy.ops.object.select_all(action="DESELECT")
    for obj in mesh_objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_objects[0]

    output_dir = default_output_dir()
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, "full_scene.glb")

    bpy.ops.export_scene.gltf(
        filepath=out_path,
        use_selection=True,
        export_format="GLB",
        export_texcoords=True,
        export_materials="EXPORT",
        export_apply=True,
        export_yup=False,  # keep Blender's Z-up, same convention as bookshelf.glb/table.glb
    )

    print(f"\nEsportati {len(mesh_objects)} oggetti in un unico file: {out_path}")
    for obj in mesh_objects:
        print(f"  {obj.name:<30} loc={tuple(round(c, 3) for c in obj.location)}")


if __name__ == "__main__":
    main()
