"""
Script for Blender 5.1.2 to create a wooden bookshelf (4 shelves, total height 1.35 m).
Geometry matches bookshelf.urdf and book_placer.py: 22 mm boards, shelf
surfaces at z = [0.022, 0.349, 0.671, 0.993], case centered on Y
(back panel at -Y, front opening towards +Y).
"""

import random
import math
import os
import bpy      # import Blender Python API


def script_dir():
    """
    Directory of the script currently open in Blender's text editor
    """
    return os.path.dirname(os.path.abspath(bpy.context.space_data.text.filepath))


def default_output_dir():
    """
    meshes/ directory of the ROS2 package, resolved from this script's location
    """
    return os.path.normpath(
        os.path.join(script_dir(), "..", "..", "src", "agibot_x2_pkg", "meshes")
    )


BOARD_T = 0.022
SHELF_CLEARANCE_Z = [0.305, 0.300, 0.300, 0.335]
INTERIOR_WIDTH = 0.756   # (X)
INTERIOR_DEPTH = 0.278   # (Y)

HALF_W = INTERIOR_WIDTH / 2.0
CASE_WIDTH = INTERIOR_WIDTH + 2 * BOARD_T
CASE_DEPTH = INTERIOR_DEPTH + BOARD_T
CASE_HEIGHT = sum(SHELF_CLEARANCE_Z) + 5 * BOARD_T
Y_OFFSET = -CASE_DEPTH / 2.0   # center the case on Y, like bookshelf.urdf

assert CASE_HEIGHT <= 1.35 + 1e-6, f"Bookshelf too high: {CASE_HEIGHT:.3f} m"

SHELF_SURFACES_Z = []
_z = BOARD_T
for _clearance in SHELF_CLEARANCE_Z:
    SHELF_SURFACES_Z.append(_z)
    _z += _clearance + BOARD_T


def clear_scene():
    """
    Clean the scene in Blender
    """
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def make_wood_image(name="shelf_wood_tex", size=256, seed=7):
    """
    Create a procedural image with the wood effect
    """
    image = bpy.data.images.new(name, width=size, height=size)
    rng = random.Random(seed)
    base_r, base_g, base_b = 0.45, 0.29, 0.16

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
    Add to the shelves the wood material
    """
    image = make_wood_image()

    mat = bpy.data.materials.new(name="shelf_wood")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    bsdf = nodes.get("Principled BSDF")
    bsdf.inputs["Roughness"].default_value = 0.6

    tex = nodes.new("ShaderNodeTexImage")
    tex.image = image
    tex.location = (-300, 0)
    links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    nodes.active = tex

    return mat


def add_board(name, size, location, material):
    """
    Add a board to the bookshelf
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


def build_bookshelf():
    """
    Create the bookshelf
    """
    wood = make_wood_material()

    add_board(
        "shelf_back",
        (CASE_WIDTH, BOARD_T, CASE_HEIGHT),
        (0.0, Y_OFFSET + BOARD_T / 2.0, CASE_HEIGHT / 2.0),
        wood,
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        add_board(
            f"shelf_side_{side}",
            (BOARD_T, CASE_DEPTH, CASE_HEIGHT),
            (sign * (HALF_W + BOARD_T / 2.0), Y_OFFSET + CASE_DEPTH / 2.0, CASE_HEIGHT / 2.0),
            wood,
        )
    z_centers = [BOARD_T / 2.0]
    z_centers += [s - BOARD_T / 2.0 for s in SHELF_SURFACES_Z[1:]]
    z_centers += [CASE_HEIGHT - BOARD_T / 2.0]
    for i, zc in enumerate(z_centers):
        add_board(
            f"shelf_board_{i}",
            (CASE_WIDTH, CASE_DEPTH, BOARD_T),
            (0.0, Y_OFFSET + CASE_DEPTH / 2.0, zc),
            wood,
        )


def export_bookshelf(output_dir=None):
    """
    Export the whole bookshelf as meshes/bookshelf.glb (glTF binary).
    The procedural wood image only exists in memory, so it is packed into
    the blend data first: the glTF exporter embeds packed images directly
    in the .glb, no external texture file needed.
    export_yup=False keeps the mesh Z-up as authored, so URDF and RViz
    markers can reference it with no rotation correction (unlike the book
    GLBs, exported Y-up).
    """
    if output_dir is None:
        output_dir = default_output_dir()
    os.makedirs(output_dir, exist_ok=True)

    image = bpy.data.images.get("shelf_wood_tex")
    if image is not None and not image.packed_file:
        image.pack()

    out_path = os.path.join(output_dir, "bookshelf.glb")
    bpy.ops.object.select_all(action="SELECT")
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
    Clear the scene, create the bookshelf in the scene and export it as .glb
    """
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    clear_scene()
    build_bookshelf()
    export_bookshelf()

    print(f"\nBookshelf created: {CASE_WIDTH:.3f} x {CASE_DEPTH:.3f} x {CASE_HEIGHT:.3f} m (WxDxH)")


if __name__ == "__main__":
    main()
