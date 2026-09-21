"""
Script for Blender 5.1.2 to render a top-down view of a "books on the table" scene for SAM/OCR.
Usage: open table_scene_retro.blend or table_scene_cover.blend (books placed,
physics settled) and run it in the Scripting tab. Writes <blend name>_photo.png
and <blend name>_camera_pose.json to render_output/ next to the .blend file.
"""
import json
import math
import os

import bpy      # import Blender Python API

CAMERA_NAME = "Table_Camera"
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
CAMERA_HEIGHT_MARGIN = 1.0   # meters above the highest point of the scene
COVER_ROUGHNESS = 0.9        # matte covers to avoid specular reflections
FRAME_MARGIN = 1.05          # 5% margin around table + books
RESOLUTION_LONG_EDGE = 2000  # px on the long edge of the image
# Books and wooden table share the same 3 lights: intermediate value that keeps the wood visible without overexposing the covers
TABLE_LIGHT_ENERGY = 17.0
# Same tilts as setup_render_camera.py, to break the vertical light/camera alignment
TABLE_LIGHT_TILT_CENTER = 45.0
TABLE_LIGHT_TILT_SIDE = 60.0

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


def output_stem():
    """
    Base name of the output files, taken from the open .blend file,
    so renders of different table scenes do not overwrite each other
    """
    return os.path.splitext(os.path.basename(bpy.data.filepath))[0]


def get_scene_objects():
    """
    All mesh objects of the scene (only table and books, no prefix filter needed)
    """
    objs = [obj for obj in bpy.data.objects if obj.type == "MESH"]
    if not objs:
        raise RuntimeError("No mesh found in the scene: run "
                            "create_table_scene_retro.py or create_table_scene_cover.py first.")
    return objs


def frame_bounds(objects):
    """
    World bounding box enclosing all the given objects
    """
    coords = []
    for obj in objects:
        coords += [obj.matrix_world @ v.co for v in obj.data.vertices]

    xs = [c.x for c in coords]
    ys = [c.y for c in coords]
    zs = [c.z for c in coords]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def setup_camera(scene_objects):
    """
    Create or move an orthographic camera looking straight down at the whole scene.
    The resolution follows the real X/Y aspect ratio of the scene, so ortho_scale
    and resolution_x/y match and the image is not cropped
    """
    (xmin, ymin, zmin), (xmax, ymax, zmax) = frame_bounds(scene_objects)
    width = xmax - xmin    # X, horizontal in the image
    depth = ymax - ymin    # Y, vertical in the image
    center_x = (xmin + xmax) / 2.0
    center_y = (ymin + ymax) / 2.0
    top_z = zmax + CAMERA_HEIGHT_MARGIN

    print(
        f"[setup_table_camera] scene bounding box: "
        f"width(X)={width:.3f}m depth(Y)={depth:.3f}m "
        f"center=({center_x:.3f}, {center_y:.3f})"
    )

    if depth >= width:
        res_y = RESOLUTION_LONG_EDGE
        res_x = max(1, round(RESOLUTION_LONG_EDGE * width / depth))
    else:
        res_x = RESOLUTION_LONG_EDGE
        res_y = max(1, round(RESOLUTION_LONG_EDGE * depth / width))
    bpy.context.scene.render.resolution_x = res_x
    bpy.context.scene.render.resolution_y = res_y

    cam_data = bpy.data.cameras.get(CAMERA_NAME) or bpy.data.cameras.new(CAMERA_NAME)
    cam_data.type = 'ORTHO'
    if depth >= width:
        cam_data.sensor_fit = 'VERTICAL'
        cam_data.ortho_scale = depth * FRAME_MARGIN
    else:
        cam_data.sensor_fit = 'HORIZONTAL'
        cam_data.ortho_scale = width * FRAME_MARGIN

    cam_obj = bpy.data.objects.get(CAMERA_NAME)
    if cam_obj is None:
        cam_obj = bpy.data.objects.new(CAMERA_NAME, cam_data)
        bpy.context.collection.objects.link(cam_obj)

    cam_obj.location = (center_x, center_y, top_z)
    # Identity rotation: the camera looks along its local -Z, i.e. straight down
    cam_obj.rotation_euler = (0.0, 0.0, 0.0)
    bpy.context.scene.camera = cam_obj

    cam_local_pose = cam_obj.matrix_world.copy()
    return cam_obj, cam_local_pose


def setup_lighting():
    """
    Three area lights above the table, tilted instead of vertical.
    With vertical lights over a vertical orthographic camera, every light satisfies
    the mirror condition on the whole table at once and overexposes it
    """
    for name, loc, tilt_deg in [
        ("Table_Light_Center", (0.0, 0.0, 1.8), TABLE_LIGHT_TILT_CENTER),
        ("Table_Light_Left",   (-0.5, 0.3, 1.5), TABLE_LIGHT_TILT_SIDE),
        ("Table_Light_Right",  (0.5, -0.3, 1.5), TABLE_LIGHT_TILT_SIDE),
    ]:
        light_data = bpy.data.lights.get(name) or bpy.data.lights.new(name, type='AREA')
        light_data.energy = TABLE_LIGHT_ENERGY
        light_data.size = 1.5

        light_obj = bpy.data.objects.get(name)
        if light_obj is None:
            light_obj = bpy.data.objects.new(name, light_data)
            bpy.context.collection.objects.link(light_obj)
        light_obj.location = loc
        light_obj.rotation_euler = (math.radians(tilt_deg), 0.0, 0.0)


def setup_world_background():
    """
    Light grey world background.
    In Cycles the world also lights the whole scene: darkening it makes the wooden
    table fade into the background, so book brightness is tuned with TABLE_LIGHT_ENERGY instead
    """
    world = bpy.context.scene.world or bpy.data.worlds.new("Table_World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = (0.85, 0.85, 0.85, 1.0)
        bg.inputs[1].default_value = 1.0


def reduce_cover_gloss():
    """
    Raise the Roughness of the *_cover/_retro/_spine materials to avoid reflections hiding the text.
    Applied again here because every scene has its own material instances
    """
    for mat in bpy.data.materials:
        if mat.name.endswith(("_cover", "_retro", "_spine")) and mat.use_nodes:
            bsdf = mat.node_tree.nodes.get("Principled BSDF")
            if bsdf:
                bsdf.inputs["Roughness"].default_value = COVER_ROUGHNESS


def setup_render_settings():
    """
    Cycles render settings with the 'AgX' view transform.
    'Standard' clips to pure white: AgX compresses the highlights, so the almost white
    ISBN stickers stay readable next to dark covers
    """
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 128
    scene.cycles.use_denoising = True
    scene.render.image_settings.file_format = 'PNG'
    scene.view_settings.view_transform = 'AgX'


def main():
    """
    Set up camera, lights and render settings, render the PNG and save the camera pose as JSON
    """
    scene_objects = get_scene_objects()
    cam_obj, cam_pose = setup_camera(scene_objects)
    setup_lighting()
    setup_world_background()
    reduce_cover_gloss()
    setup_render_settings()

    stem = output_stem()
    out_png = os.path.join(OUTPUT_DIR, f"{stem}_photo.png")
    bpy.context.scene.render.filepath = out_png
    bpy.ops.render.render(write_still=True)

    loc = cam_pose.to_translation()
    quat = cam_pose.to_quaternion()
    pose_json = {
        "camera_world": {
            "x": loc.x, "y": loc.y, "z": loc.z,
            "qx": quat.x, "qy": quat.y, "qz": quat.z, "qw": quat.w,
        },
        "ortho_scale": cam_obj.data.ortho_scale,
        "resolution_x": bpy.context.scene.render.resolution_x,
        "resolution_y": bpy.context.scene.render.resolution_y,
    }
    pose_path = os.path.join(OUTPUT_DIR, f"{stem}_camera_pose.json")
    with open(pose_path, "w", encoding="utf-8") as f:
        json.dump(pose_json, f, indent=2)

    print(f"Render saved to: {out_png}")
    print(f"Camera pose saved to: {pose_path}")


if __name__ == "__main__":
    main()
