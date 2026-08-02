"""
Generic Blender rigid-body helpers shared by the scene builders: mark an
object as an active rigid body so it reacts to gravity, and advance+bake
the simulation so gravity is actually respected in the saved .blend (not
just while scrubbing the timeline or pressing Play by hand).

Plain reusable module (no bpy.context.space_data access at import time),
same convention as table/table_scene_builder.py: safe to import from any
entry script once its own directory is on sys.path.

Note: table/table_scene_builder.py keeps its own copies of the same two
functions (predates this module, scoped to the table-scene subpackage) -
not merged here to avoid a cross-import between the library/ and table/
builder subpackages for two small generic functions.
"""

import bpy      # import Blender Python API

SETTLE_FRAMES = 60   # ~2.5s at 24fps, same value used by table_scene_builder.py


def make_rigid_active(obj, mass, shape):
    """
    Register obj in the physics world as an active rigid body, so it
    reacts to gravity when the simulation runs.
    """
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.rigidbody.object_add(type="ACTIVE")
    obj.rigid_body.mass = mass
    obj.rigid_body.collision_shape = shape


def settle_physics(objects, frames=SETTLE_FRAMES):
    """
    Step the rigid body simulation forward so gravity actually settles
    every object, then bake the resulting pose into each object's real
    transform with visual_transform_apply(): frame_set() alone only moves
    the *evaluated* depsgraph copy used for display, the object's own
    location/rotation stay at the un-settled spawn pose until baked -
    without this the saved .blend would only look settled while the
    timeline is scrubbed to a specific frame, not on reload or export.
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
