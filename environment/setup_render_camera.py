"""
Script for Blender 5.1.2 to render an orthographic front "photo" of the bookshelf for SAM detection/OCR.
Usage: open the .blend with the books already placed and run it in the Scripting tab.
Writes library_photo.png and camera_pose.json (camera pose relative to the
bookshelf) to render_output/ next to the .blend file.
"""
import json
import math
import os

import bpy      # import Blender Python API
from mathutils import Vector      # import Blender Python API

# The bookshelf is made of several unparented parts (all at 0° rotation), matched by name prefix
SHELF_PART_PREFIXES = (
    "shelf_back", "shelf_board_", "shelf_side_left", "shelf_side_right",
)
SHELF_ORIGIN = (0.0, 0.0, 0.0)   # must match SHELF_ORIGIN in export_manual_layout.py
CAMERA_NAME = "SAM_Camera"
if not bpy.data.filepath:
    raise RuntimeError(
        "The .blend file has not been saved yet: OUTPUT_DIR cannot "
        "be computed relative to it (bpy.data.filepath is empty) and "
        "would end up in the Blender process working directory, often "
        "not writable. Save the .blend (Ctrl+S) before running the script."
    )
OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(bpy.data.filepath)), "render_output"
)
CAMERA_DISTANCE = 1.5        # meters in front of the bookshelf
COVER_ROUGHNESS = 0.9        # matte covers to avoid specular reflections
FRAME_MARGIN = 1.05          # 5% margin around the bookshelf
RESOLUTION_LONG_EDGE = 2000  # px on the long edge of the image

try:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
except PermissionError as exc:
    raise PermissionError(
        f"Insufficient permissions to create/write {OUTPUT_DIR}: "
        "check that the folder of the .blend file is writable "
        "by the user running Blender (on Docker/WSL a chown/chmod on the "
        "mount may be needed, or save the .blend on a native path "
        "instead of a Windows mount)."
    ) from exc


def get_shelf_parts():
    """
    Mesh objects of the bookshelf, selected by SHELF_PART_PREFIXES
    """
    parts = [
        obj for obj in bpy.data.objects
        if obj.type == 'MESH' and obj.name.startswith(SHELF_PART_PREFIXES)
    ]
    if not parts:
        raise RuntimeError(
            "No bookshelf part found with the prefixes "
            f"{SHELF_PART_PREFIXES}. Edit SHELF_PART_PREFIXES at the top of the script."
        )
    return parts


def frame_bounds(parts):
    """
    World bounding box enclosing all the bookshelf parts
    """
    coords = []
    for obj in parts:
        coords += [obj.matrix_world @ v.co for v in obj.data.vertices]

    xs = [c.x for c in coords]
    ys = [c.y for c in coords]
    zs = [c.z for c in coords]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def setup_camera(shelf_parts):
    """
    Create or move an orthographic camera facing the front of the bookshelf.
    The resolution follows the real aspect ratio of the bookshelf and sensor_fit is set
    on the dominant axis, so ortho_scale and resolution_x/y match with no crop.
    Returns the camera and its pose in the bookshelf local frame
    """
    (xmin, ymin, zmin), (xmax, ymax, zmax) = frame_bounds(shelf_parts)
    width = xmax - xmin
    height = zmax - zmin
    center_x = (xmin + xmax) / 2.0
    center_z = (zmin + zmax) / 2.0
    front_y = ymax + CAMERA_DISTANCE  # +Y = open side / covers, as in create_books.py

    print(
        f"[setup_render_camera] bookshelf bounding box: "
        f"width(X)={width:.3f}m height(Z)={height:.3f}m "
        f"center=({center_x:.3f}, {center_z:.3f})"
    )

    if height >= width:
        res_y = RESOLUTION_LONG_EDGE
        res_x = max(1, round(RESOLUTION_LONG_EDGE * width / height))
    else:
        res_x = RESOLUTION_LONG_EDGE
        res_y = max(1, round(RESOLUTION_LONG_EDGE * height / width))
    bpy.context.scene.render.resolution_x = res_x
    bpy.context.scene.render.resolution_y = res_y

    cam_data = bpy.data.cameras.get(CAMERA_NAME) or bpy.data.cameras.new(CAMERA_NAME)
    cam_data.type = 'ORTHO'
    if height >= width:
        cam_data.sensor_fit = 'VERTICAL'
        cam_data.ortho_scale = height * FRAME_MARGIN
    else:
        cam_data.sensor_fit = 'HORIZONTAL'
        cam_data.ortho_scale = width * FRAME_MARGIN

    cam_obj = bpy.data.objects.get(CAMERA_NAME)
    if cam_obj is None:
        cam_obj = bpy.data.objects.new(CAMERA_NAME, cam_data)
        bpy.context.collection.objects.link(cam_obj)

    cam_obj.location = (center_x, front_y, center_z)
    # Look towards -Y (the bookshelf), no roll/tilt
    cam_obj.rotation_euler = (math.radians(90.0), 0.0, math.radians(180.0))
    bpy.context.scene.camera = cam_obj

    # Bookshelf rotation is 0°: local frame = world frame shifted by SHELF_ORIGIN
    cam_local_pose = cam_obj.matrix_world.copy()
    cam_local_pose.translation -= Vector(SHELF_ORIGIN)
    return cam_obj, cam_local_pose


def setup_lighting():
    """
    Three tilted area lights in front of the bookshelf for diffuse lighting
    """
    for name, loc, rot_x in [
        ("SAM_Light_Center", (0.0, 1.0, 1.0), 45.0),
        ("SAM_Light_Left",   (-1.5, 0.8, 0.8), 60.0),
        ("SAM_Light_Right",  (1.5, 0.8, 0.8), 60.0),
    ]:
        light_data = bpy.data.lights.get(name) or bpy.data.lights.new(name, type='AREA')
        light_data.energy = 300.0
        light_data.size = 1.5

        light_obj = bpy.data.objects.get(name)
        if light_obj is None:
            light_obj = bpy.data.objects.new(name, light_data)
            bpy.context.collection.objects.link(light_obj)
        light_obj.location = loc
        light_obj.rotation_euler = (math.radians(rot_x), 0.0, 0.0)


def setup_world_background():
    """
    Light grey world background
    """
    world = bpy.context.scene.world or bpy.data.worlds.new("SAM_World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = (0.85, 0.85, 0.85, 1.0)
        bg.inputs[1].default_value = 1.0


def reduce_cover_gloss():
    """
    Raise the Roughness of the *_cover/_retro/_spine materials to avoid reflections hiding the text
    """
    for mat in bpy.data.materials:
        if mat.name.endswith(("_cover", "_retro", "_spine")) and mat.use_nodes:
            bsdf = mat.node_tree.nodes.get("Principled BSDF")
            if bsdf:
                bsdf.inputs["Roughness"].default_value = COVER_ROUGHNESS


def setup_render_settings():
    """
    Cycles render settings.
    resolution_x/y are left untouched: setup_camera() already set them from the
    bookshelf aspect ratio, and a fixed aspect ratio would crop the image
    """
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 128
    scene.cycles.use_denoising = True
    scene.render.image_settings.file_format = 'PNG'
    scene.view_settings.view_transform = 'Standard'


def main():
    """
    Set up camera, lights and render settings, render the PNG and save the camera pose as JSON
    """
    shelf_parts = get_shelf_parts()
    cam_obj, cam_local_pose = setup_camera(shelf_parts)
    setup_lighting()
    setup_world_background()
    reduce_cover_gloss()
    setup_render_settings()

    out_png = os.path.join(OUTPUT_DIR, "library_photo.png")
    bpy.context.scene.render.filepath = out_png
    bpy.ops.render.render(write_still=True)

    loc = cam_local_pose.to_translation()
    quat = cam_local_pose.to_quaternion()
    pose_json = {
        "camera_local_to_bookshelf": {
            "x": loc.x, "y": loc.y, "z": loc.z,
            "qx": quat.x, "qy": quat.y, "qz": quat.z, "qw": quat.w,
        },
        "ortho_scale": cam_obj.data.ortho_scale,
        "resolution_x": bpy.context.scene.render.resolution_x,
        "resolution_y": bpy.context.scene.render.resolution_y,
    }
    pose_path = os.path.join(OUTPUT_DIR, "camera_pose.json")
    with open(pose_path, "w", encoding="utf-8") as f:
        json.dump(pose_json, f, indent=2)

    print(f"Render saved to: {out_png}")
    print(f"Camera pose saved to: {pose_path}")


if __name__ == "__main__":
    main()
