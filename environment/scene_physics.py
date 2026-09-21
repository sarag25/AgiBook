"""
Shared rigid-body helpers for the scene builders: register objects as active
rigid bodies and bake the gravity-settled pose into the saved .blend.
Safe to import from any entry script (no bpy.context.space_data access at import time).
"""

import bpy      # import Blender Python API

SETTLE_FRAMES = 60   # ~2.5 s at 24 fps


def make_rigid_active(obj, mass, shape):
    """
    Register obj as an active rigid body so it reacts to gravity
    """
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.rigidbody.object_add(type="ACTIVE")
    obj.rigid_body.mass = mass
    obj.rigid_body.collision_shape = shape


def settle_physics(objects, frames=SETTLE_FRAMES):
    """
    Run the rigid body simulation for some frames, then bake the settled pose
    into each object's transform with visual_transform_apply().
    frame_set() alone only moves the evaluated (display) copy, so without the
    bake the saved .blend would not keep the settled pose on reload or export.
    """
    if not objects:
        return

    scene = bpy.context.scene
    for f in range(scene.frame_start, scene.frame_start + frames):
        scene.frame_set(f)

    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.visual_transform_apply()
        obj.select_set(False)

    scene.frame_set(scene.frame_start)
