"""
Script for Blender 5.1.2 to create desk decorations and export in the .glb format
"""

import bpy      # import Blender Python API
import bmesh    # import Blender Python API
import math
import os
import random

def script_dir():
    """
    Directory of the script currently open in Blender's text editor.
    Only valid when this file is the entry script: when imported from
    another script (e.g. create_scene.py), pass export_dir explicitly.
    """
    return os.path.dirname(os.path.abspath(bpy.context.space_data.text.filepath))


GRIPPER_MAX_OPENING = 0.08

DECORATIONS = [
    {
        "name": "pen_holder",
        "type": "cylinder",
        "radius": 0.028,          
        "height": 0.095,
        "mat": {"base_color": (0.10, 0.10, 0.12, 1.0), "texture": "noise",
                "roughness": 0.35, "scale": 60.0, "bump_strength": 0.04},
        "mass": 0.12,
    },
    {
        "name": "paperweight_ball",
        "type": "sphere",
        "radius": 0.032,          
        "mat": {"base_color": (0.55, 0.08, 0.08, 1.0), "texture": "marble",
                "extra_color": (0.92, 0.85, 0.80, 1.0), "roughness": 0.12, "scale": 8.0},
        "mass": 0.15,
    },
    {
        "name": "desk_globe",
        "type": "globe",
        "sphere_radius": 0.040,
        "base_radius": 0.032,
        "base_height": 0.014,
        "stem_radius": 0.006,
        "stem_height": 0.028,
        "mat_sphere": {"base_color": (0.08, 0.30, 0.55, 1.0), "texture": "continents",
                       "extra_color": (0.20, 0.45, 0.18, 1.0), "roughness": 0.45, "scale": 6.0},
        "mat_base": {"base_color": (0.35, 0.24, 0.14, 1.0), "texture": "wood",
                     "extra_color": (0.18, 0.11, 0.06, 1.0), "roughness": 0.4, "scale": 40.0},
        "mass": 0.28,
    },
    {
        "name": "potted_plant",
        "type": "plant",
        "pot_r1": 0.038,          
        "pot_r2": 0.050,          
        "pot_height": 0.06,
        "foliage_radius": 0.05,
        "mat_pot": {"base_color": (0.55, 0.30, 0.20, 1.0), "texture": "noise",
                   "roughness": 0.85, "scale": 25.0, "bump_strength": 0.12},
        "mat_foliage": {"base_color": (0.15, 0.42, 0.18, 1.0), "texture": "noise",
                        "roughness": 0.65, "scale": 35.0, "bump_strength": 0.2},
        "mass": 0.22,
    },
    {
        "name": "coffee_mug",
        "type": "mug",
        "body_radius": 0.032,
        "body_height": 0.09,
        "handle_major_radius": 0.022,
        "handle_minor_radius": 0.006,
        "mat": {"base_color": (0.92, 0.92, 0.90, 1.0), "texture": "noise",
                "roughness": 0.2, "scale": 15.0, "bump_strength": 0.02},
        "mass": 0.20,
    },
    {
        "name": "coaster",
        "type": "cylinder",
        "radius": 0.05,
        "height": 0.008,
        "mat": {"base_color": (0.42, 0.28, 0.16, 1.0), "texture": "wood",
                "extra_color": (0.22, 0.13, 0.07, 1.0), "roughness": 0.45, "scale": 22.0},
        "mass": 0.05,
    },
]


def _lerp(a, b, t):
    """
    Linear interpolation between two RGB triples
    """
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def make_texture_image(name, texture, base_color, extra_color, scale, size=256, seed=7):
    """
    Generate a procedural image pixel by pixel (like the bookshelf wood
    texture). Image textures show up in the viewport and are embedded in
    exported GLB files, unlike shader-node procedural textures, which
    glTF cannot represent (they were silently lost on export).
    Patterns:
      - "noise": grainy variation, for plastic, ceramic, terracotta, foliage
      - "marble": bright veins, for the paperweight
      - "wood": grain/rings, for the globe base and the coaster
      - "continents": land/ocean blobs, for the globe sphere
    """
    rng = random.Random(seed)
    base = base_color[:3]
    if extra_color is not None:
        extra = extra_color[:3]
    elif texture == "marble":
        extra = (1.0, 1.0, 1.0)
    else:
        extra = tuple(c * 0.5 for c in base)

    image = bpy.data.images.new(name, width=size, height=size)
    pixels = [0.0] * (size * size * 4)
    phase_u = rng.uniform(0.0, math.tau)
    phase_v = rng.uniform(0.0, math.tau)
    freq = max(2.0, scale * 0.2)

    for y in range(size):
        v = y / size
        row = y * size * 4
        for x in range(size):
            u = x / size
            if texture == "marble":
                t = 0.5 + 0.5 * math.sin(
                    freq * u * math.tau
                    + 3.0 * math.sin(freq * 0.5 * v * math.tau + phase_v)
                )
                t = t ** 3          # thin bright veins on the base color
            elif texture == "wood":
                rings = math.sin(freq * v * math.tau
                                 + 1.5 * math.sin(u * math.tau + phase_u))
                t = 0.5 + 0.5 * rings
            elif texture == "continents":
                blob = (math.sin(3.0 * u * math.tau + phase_u)
                        * math.sin(2.0 * v * math.tau + phase_v)
                        + math.sin(5.0 * u * math.tau + phase_v)
                        * math.sin(3.0 * v * math.tau + phase_u))
                t = 1.0 if blob > 0.55 else 0.0   # land over ocean
            else:  # noise
                t = rng.uniform(0.0, 1.0) * 0.35
            r, g, b = _lerp(base, extra, t)
            idx = row + x * 4
            pixels[idx + 0] = min(1.0, max(0.0, r))
            pixels[idx + 1] = min(1.0, max(0.0, g))
            pixels[idx + 2] = min(1.0, max(0.0, b))
            pixels[idx + 3] = 1.0

    image.pixels = pixels
    image.update()
    image.pack()   # embedded in the .blend and in exported GLB files
    return image


def make_material(name, base_color, texture="noise", roughness=0.5, metallic=0.0,
                   scale=15.0, bump_strength=0.1, extra_color=None):
    """
    Create a material driven by a generated image texture (see
    make_texture_image), mapped through the primitives' UVs. The same
    image also drives a bump node for surface relief.
    """
    image = make_texture_image(name + "_img", texture, base_color, extra_color, scale)

    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    out = nodes.new("ShaderNodeOutputMaterial")
    out.location = (600, 0)
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (350, 0)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])

    tex = nodes.new("ShaderNodeTexImage")
    tex.image = image
    tex.location = (0, 0)
    links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])

    if bump_strength:
        bump = nodes.new("ShaderNodeBump")
        bump.location = (150, -250)
        bump.inputs["Strength"].default_value = bump_strength
        links.new(tex.outputs["Color"], bump.inputs["Height"])
        links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])

    return mat


def bake_and_reset_origin(obj):
    """
    Apply the object's location, rotation, and scale to the mesh, then reset
    the object's origin to (0, 0, 0), so that multiple parts created at different
    positions can be joined while preserving their absolute mesh coordinates
    """
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)


def add_cylinder_part(radius, height, z_base, mat, name):
    """
    Create a cylinder with its base at the specified Z position, assign
    the given material, apply the object's transforms to the mesh, reset
    the origin, and return the created object
    """
    bpy.ops.mesh.primitive_cylinder_add(
        radius=radius, depth=height, location=(0, 0, z_base + height / 2.0)
    )
    obj = bpy.context.active_object
    obj.name = name
    obj.data.materials.append(mat)
    bake_and_reset_origin(obj)
    return obj


def add_sphere_part(radius, z_center, mat, name):
    """
    Create a UV sphere, assign a material, apply transforms, reset the
    origin, and return the resulting object
    """
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=(0, 0, z_center))
    obj = bpy.context.active_object
    obj.name = name
    obj.data.materials.append(mat)
    bake_and_reset_origin(obj)
    return obj


def add_cone_part(radius1, radius2, height, z_base, mat, name):
    """
    Create a cone with its base at the specified Z position, assign the
    given material, apply the object's transforms to the mesh, reset the
    origin, and return the created object
    """
    bpy.ops.mesh.primitive_cone_add(
        radius1=radius1, radius2=radius2, depth=height,
        location=(0, 0, z_base + height / 2.0),
    )
    obj = bpy.context.active_object
    obj.name = name
    obj.data.materials.append(mat)
    bake_and_reset_origin(obj)
    return obj


def add_torus_part(major_radius, minor_radius, location, rotation, mat, name, scale=(1.0, 1.0, 1.0)):
    """
    Create a torus at the specified location, apply rotation and scale,
    assign the given material, bake transforms into the mesh, reset the
    origin, and return the created object
    """
    bpy.ops.mesh.primitive_torus_add(
        major_radius=major_radius, minor_radius=minor_radius, location=location,
    )
    obj = bpy.context.active_object
    obj.name = name
    obj.rotation_euler = rotation
    obj.scale = scale
    obj.data.materials.append(mat)
    bake_and_reset_origin(obj)
    return obj


def join_parts(parts, final_name):
    """
    Join multiple mesh objects into a single object, assign the specified
    name, and return the resulting combined object
    """
    bpy.ops.object.select_all(action="DESELECT")
    for p in parts:
        p.select_set(True)
    bpy.context.view_layer.objects.active = parts[0]
    bpy.ops.object.join()
    obj = bpy.context.active_object
    obj.name = final_name
    return obj


def create_cylinder_decor(deco):
    """
    Create a pen holder/coaster as a single solid cylinder with the
    specified material and return the object along with its grasp width
    """
    mat = make_material(deco["name"] + "_mat", **deco["mat"])
    obj = add_cylinder_part(deco["radius"], deco["height"], 0.0, mat, deco["name"])
    grasp_width = 2 * deco["radius"]
    return obj, grasp_width


def create_sphere_decor(deco):
    """
    Create a paperweight as a sphere resting on the ground plane, with
    the object's origin positioned at the base, and return it with its
    grasp width
    """
    r = deco["radius"]
    mat = make_material(deco["name"] + "_mat", **deco["mat"])
    obj = add_sphere_part(r, r, mat, deco["name"])
    grasp_width = 2 * r
    return obj, grasp_width


def create_globe_decor(deco):
    """
    Create a globe composed of a wooden cylindrical base, a stem, and a
    sphere with continent-like material, then join all parts into one object
    """
    mat_base = make_material(deco["name"] + "_base_mat", **deco["mat_base"])
    mat_sphere = make_material(deco["name"] + "_sphere_mat", **deco["mat_sphere"])

    base = add_cylinder_part(
        deco["base_radius"], deco["base_height"], 0.0, mat_base, deco["name"] + "_base",
    )
    stem_z = deco["base_height"]
    stem = add_cylinder_part(
        deco["stem_radius"], deco["stem_height"], stem_z, mat_base, deco["name"] + "_stem",
    )
    sphere_z = stem_z + deco["stem_height"] + deco["sphere_radius"]
    sphere = add_sphere_part(
        deco["sphere_radius"], sphere_z, mat_sphere, deco["name"] + "_sphere",
    )
    obj = join_parts([base, stem, sphere], deco["name"])
    grasp_width = 2 * deco["sphere_radius"]
    return obj, grasp_width


def create_plant_decor(deco):
    """
    Create a plant composed of a terracotta truncated cone pot and a
    spherical foliage part, then join them into a single object
    """
    mat_pot = make_material(deco["name"] + "_pot_mat", **deco["mat_pot"])
    mat_foliage = make_material(deco["name"] + "_foliage_mat", **deco["mat_foliage"])

    pot = add_cone_part(
        deco["pot_r1"], deco["pot_r2"], deco["pot_height"], 0.0, mat_pot, deco["name"] + "_pot",
    )
    foliage_z = deco["pot_height"] + deco["foliage_radius"] * 0.6
    foliage = add_sphere_part(
        deco["foliage_radius"], foliage_z, mat_foliage, deco["name"] + "_foliage",
    )
    obj = join_parts([pot, foliage], deco["name"])
    grasp_width = 2 * deco["pot_r1"]
    return obj, grasp_width


def create_mug_decor(deco):
    """
    Create a mug composed of a cylindrical body and a torus-based handle.
    The torus is rotated 90° around X so its ring stands in the vertical
    XZ plane (like a real handle seen from the side): the top and bottom
    of the ring sink into the body wall, welding the handle to the mug.
    """
    mat = make_material(deco["name"] + "_mat", **deco["mat"])

    body = add_cylinder_part(
        deco["body_radius"], deco["body_height"], 0.0, mat, deco["name"] + "_body",
    )
    handle_z = deco["body_height"] / 2.0
    handle_x = deco["body_radius"] - deco["handle_minor_radius"] * 0.3
    handle = add_torus_part(
        deco["handle_major_radius"], deco["handle_minor_radius"],
        (handle_x, 0.0, handle_z), (1.5707963, 0.0, 0.0),
        mat, deco["name"] + "_handle",
        scale=(1.0, 1.3, 1.0),   # local Y → world Z after the rotation
    )
    obj = join_parts([body, handle], deco["name"])
    grasp_width = 2 * deco["body_radius"]
    return obj, grasp_width


BUILDERS = {
    "cylinder": create_cylinder_decor,
    "sphere": create_sphere_decor,
    "globe": create_globe_decor,
    "plant": create_plant_decor,
    "mug": create_mug_decor,
}


def export_decoration(obj, name, export_dir):
    """
    Export the selected decoration object as a GLB file, including its
    applied geometry and materials, and save it to export_dir
    """
    out_path = os.path.join(export_dir, name + ".glb")
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.export_scene.gltf(
        filepath=out_path,
        use_selection=True,
        export_format="GLB",
        export_materials="EXPORT",
        export_apply=True,
    )
    print(f"  -> {out_path}")


def create_decoration(deco, location=(0, 0, 0), export_dir=None):
    """
    Create a decoration object using the appropriate builder, assign its
    position and mass, check whether it can be handled by the gripper,
    export it if export_dir is given, and return the created object
    """
    print(f"\n=== {deco['name']} ===")
    builder = BUILDERS[deco["type"]]
    obj, grasp_width = builder(deco)
    obj.location = location
    obj["mass"] = deco["mass"]

    if grasp_width > GRIPPER_MAX_OPENING:
        print(
            f"  ATTENZIONE: grasp_width={grasp_width:.3f} m supera "
            f"GRIPPER_MAX_OPENING={GRIPPER_MAX_OPENING:.3f} m, il gripper "
            f"potrebbe non riuscire ad afferrarlo."
        )
    else:
        print(f"  grasp_width={grasp_width:.3f} m (ok, < {GRIPPER_MAX_OPENING:.3f} m)")

    if export_dir is not None:
        export_decoration(obj, deco["name"], export_dir)
    return obj


def main():
    """
    Clear the Blender scene, create all configured decorations at spaced
    positions, and export the resulting GLB files
    """
    export_dir = os.path.normpath(
        os.path.join(script_dir(), "..", "..", "src", "agibot_x2_pkg", "meshes", "desk_decorations")
    )
    os.makedirs(export_dir, exist_ok=True)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    for i, deco in enumerate(DECORATIONS):
        create_decoration(deco, location=(i * 0.25, 0, 0), export_dir=export_dir)

    print("\nCreation completed. GLB files saved in:", export_dir)


if __name__ == "__main__":
    main()
