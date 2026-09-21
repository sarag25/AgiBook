"""
Script for Blender 5.1.2 to create a simple wooden table (top slab + 4 legs) and export it in the .glb format.
TABLE_HEIGHT is fixed for the AgiBot X2 arm, while width/depth are parameters of
build_table() so callers can size the top to what they place on it.
"""

import math
import os
import bpy      # import Blender Python API
import random


def script_dir():
    """
    Directory of the script currently open in Blender's text editor.
    Only valid when this file is the entry script: when imported, pass output_dir to export_table().
    Uses bpy.path.abspath() because text.filepath can be relative to the .blend ("//" prefix),
    which os.path.abspath() does not resolve.
    """
    return os.path.dirname(bpy.path.abspath(bpy.context.space_data.text.filepath))


def default_output_dir():
    """
    meshes/ directory of the ROS2 package, resolved from this script's location
    """
    return os.path.normpath(
        os.path.join(script_dir(), "..", "..", "src", "agibot_x2_pkg", "meshes")
    )


TOP_THICKNESS = 0.03
LEG_SIZE = 0.04
LEG_INSET = 0.05   # legs set back from the top edges

# Close to the arm's "reach_shelf_low" pose (~0.5 m), reachable without extreme extension
TABLE_HEIGHT = 0.50   # top surface Z (Gazebo table overrides it via build_table(height=...))

DEFAULT_WIDTH = 1.20   # X, used only when build_table() is run standalone
DEFAULT_DEPTH = 0.90   # Y


def clear_scene():
    """
    Clean the scene in Blender
    """
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def make_wood_image(name="table_wood_tex", size=256, seed=11):
    """
    Create a procedural image with the wood effect
    (distinct name/seed from the bookshelf's, so they don't share the same image datablock)
    """
    image = bpy.data.images.new(name, width=size, height=size)
    rng = random.Random(seed)
    base_r, base_g, base_b = 0.52, 0.35, 0.20

    pixels = [0.0] * (size * size * 4)
    for y in range(size):
        grain = 0.85 + 0.15 * math.sin(y * 0.35) + rng.uniform(-0.04, 0.04)
        row = y * size * 4
        for x in range(size):
            noise = rng.uniform(-0.03, 0.03)
            idx = row + x * 4
            pixels[idx + 0] = min(1.0, max(0.0, base_r * grain + noise))
            pixels[idx + 1] = min(1.0, max(0.0, base_g * grain + noise))
            pixels[idx + 2] = min(1.0, max(0.0, base_b * grain + noise))
            pixels[idx + 3] = 1.0

    image.pixels = pixels
    image.update()
    return image


def make_wood_material():
    """
    Build the table's wood material (procedural image texture)
    """
    image = make_wood_image()

    mat = bpy.data.materials.new(name="table_wood")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    bsdf = nodes.get("Principled BSDF")
    bsdf.inputs["Roughness"].default_value = 0.55

    tex = nodes.new("ShaderNodeTexImage")
    tex.image = image
    tex.location = (-300, 0)
    links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    nodes.active = tex

    return mat


def add_board(name, size, location, material):
    """
    Add a board (top slab or leg) to the table as a passive rigid body
    """
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = size
    bpy.ops.object.transform_apply(scale=True, location=False, rotation=False)

    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.cube_project(cube_size=1.0)
    bpy.ops.object.mode_set(mode="OBJECT")

    obj.data.materials.append(material)

    bpy.ops.rigidbody.object_add(type="PASSIVE")
    obj.rigid_body.collision_shape = "BOX"

    return obj


def build_table(width=DEFAULT_WIDTH, depth=DEFAULT_DEPTH, height=TABLE_HEIGHT):
    """
    Create the table (top centered on X/Y with its surface at `height`, 4 inset legs)
    and return the created objects.
    `height` lets the Gazebo table in create_full_scene.py use a taller surface
    without changing the TABLE_HEIGHT default used by the book table scenes.
    """
    wood = make_wood_material()

    parts = [add_board(
        "table_top",
        (width, depth, TOP_THICKNESS),
        (0.0, 0.0, height - TOP_THICKNESS / 2.0),
        wood,
    )]

    leg_height = height - TOP_THICKNESS
    half_w = width / 2.0 - LEG_INSET
    half_d = depth / 2.0 - LEG_INSET
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        parts.append(add_board(
            f"table_leg_{i}",
            (LEG_SIZE, LEG_SIZE, leg_height),
            (sx * half_w, sy * half_d, leg_height / 2.0),
            wood,
        ))

    return parts


def export_table(output_dir=None):
    """
    Export the table (objects named "table_*") as meshes/table.glb (glTF binary).
    The in-memory wood image is packed first so the exporter embeds it in the .glb.
    export_yup=False keeps the mesh Z-up, consistent with bookshelf.glb.
    """
    if output_dir is None:
        output_dir = default_output_dir()
    os.makedirs(output_dir, exist_ok=True)

    image = bpy.data.images.get("table_wood_tex")
    if image is not None and not image.packed_file:
        image.pack()

    bpy.ops.object.select_all(action="DESELECT")
    for obj in bpy.data.objects:
        if obj.name.startswith("table_"):
            obj.select_set(True)

    out_path = os.path.join(output_dir, "table.glb")
    bpy.ops.export_scene.gltf(
        filepath=out_path,
        use_selection=True,
        export_format="GLB",
        export_texcoords=True,
        export_materials="EXPORT",
        export_apply=True,
        export_yup=False,
    )
    print(f"  -> {out_path}")


def main():
    """
    Clear the scene, create a standalone table (default size) and export it as .glb
    """
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    clear_scene()
    build_table()
    export_table()

    print(f"\nTable created: {DEFAULT_WIDTH:.3f} x {DEFAULT_DEPTH:.3f} x {TABLE_HEIGHT:.3f} m (WxDxH)")


if __name__ == "__main__":
    main()
