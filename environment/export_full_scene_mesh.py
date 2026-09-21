"""
Script for Blender 5.1.2 to export the whole assembled scene (bookshelf, books, decorations, table)
as a single .glb file (src/agibot_x2_pkg/meshes/full_scene.glb), keeping the relative positions as built.
Run it in the same Blender session after create_full_scene.py (or with full_scene.blend open).
"""
import os

import bpy      # import Blender Python API


def default_output_dir():
    """
    meshes/ directory of the ROS2 package, resolved from the open .blend's location
    (bpy.data.filepath, so it works however this script was opened)
    """
    blend_dir = os.path.dirname(bpy.path.abspath(bpy.data.filepath))
    return os.path.normpath(
        os.path.join(blend_dir, "..", "src", "agibot_x2_pkg", "meshes")
    )


# Not part of the physical scene
EXCLUDE_NAMES = {"robot_reference"}


def pack_all_images():
    """
    Pack every image that is not backed by a file on disk
    (the in-memory procedural wood textures would otherwise be silently dropped by the export)
    """
    for image in bpy.data.images:
        if image.source == "FILE" and not image.packed_file and image.filepath:
            continue  # real file on disk, exporter will read it directly
        if not image.packed_file and image.has_data:
            image.pack()


def main():
    """
    Select all mesh objects of the scene and export them as a single .glb
    """
    mesh_objects = [
        obj for obj in bpy.data.objects
        if obj.type == "MESH" and obj.name not in EXCLUDE_NAMES
    ]
    if not mesh_objects:
        print("No mesh object found in the scene: did you run "
              "create_full_scene.py in this session before this script?")
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

    print(f"\nExported {len(mesh_objects)} objects into a single file: {out_path}")
    for obj in mesh_objects:
        print(f"  {obj.name:<30} loc={tuple(round(c, 3) for c in obj.location)}")


if __name__ == "__main__":
    main()
