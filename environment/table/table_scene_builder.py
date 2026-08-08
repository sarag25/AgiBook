"""
Shared orchestration for the "books on table" scenes (retro-up and
cover-up): grid layout sizing, book placement at an arbitrary rotation,
and the gravity settle/bake pass. Reused by the two top-level entry
points (environment/create_table_scene_retro.py and
environment/create_table_scene_cover.py) so the two scenes - identical
except for which face ends up on top - don't duplicate ~60 lines of
layout/physics code.

Plain reusable module (no bpy.context.space_data access at import time,
unlike the top-level entry scripts): safe to import from either of them,
same convention as create_bookshelf.py/create_books.py being imported by
create_scene.py.
"""

import math

import bpy      # import Blender Python API

BOOK_GAP = 0.03        # gap between adjacent books on the grid
TABLE_MARGIN = 0.05    # border between the table edge and the outermost books
EPS = 0.001            # vertical play so the physics solver starts contact-free
BOOK_DENSITY = 600.0   # kg/m3, same value used by create_scene.py
N_COLS = 5              # 15 books -> 5x3 grid

SETTLE_FRAMES = 60     # ~2.5s at 24fps: enough for the EPS drop to settle


def grid_cell_size(book_list, gap=BOOK_GAP):
    """
    Cell size (X, Y) that fits the widest/tallest book lying flat: once a
    book is rotated flat (either RETRO_UP_ROTATION or COVER_UP_ROTATION,
    see the two entry scripts), its footprint on the table is
    (width=sx, height=sz) regardless of which face ends up on top - both
    rotations only flip which of +Z/-Z the Y faces point to.
    """
    max_w = max(b["size"][0] for b in book_list)
    max_h = max(b["size"][2] for b in book_list)
    return max_w + gap, max_h + gap


def table_footprint(book_list, n_cols=N_COLS, margin=TABLE_MARGIN):
    """
    (width, depth) the table top needs to fit every book of book_list in
    an n_cols grid, plus a margin border. Used before build_table() so the
    top is sized from the books, not the other way around.
    """
    cell_x, cell_y = grid_cell_size(book_list)
    n_rows = math.ceil(len(book_list) / n_cols)
    return n_cols * cell_x + 2 * margin, n_rows * cell_y + 2 * margin


def make_rigid_active(obj, mass, shape):
    """
    Register obj in the physics world as an active rigid body, so it
    reacts to gravity when the simulation runs (same helper as create_scene.py)
    """
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.rigidbody.object_add(type="ACTIVE")
    obj.rigid_body.mass = mass
    obj.rigid_body.collision_shape = shape


def place_books_on_table(books_module, images_dir, table_height, rotation,
                          n_cols=N_COLS, density=BOOK_DENSITY, eps=EPS):
    """
    Lay out every book in books_module.BOOKS on a grid on the table
    surface, oriented flat by `rotation` (an (rx, ry, rz) rotation_euler -
    RETRO_UP_ROTATION or COVER_UP_ROTATION, defined by the caller), resting
    on the table with `eps` clearance so the rigid body solver starts
    contact-free. Returns the created book objects (for settle_physics()).
    """
    book_list = books_module.BOOKS
    cell_x, cell_y = grid_cell_size(book_list)
    n_rows = math.ceil(len(book_list) / n_cols)
    grid_w = n_cols * cell_x
    grid_h = n_rows * cell_y
    x0 = -grid_w / 2.0 + cell_x / 2.0
    y0 = -grid_h / 2.0 + cell_y / 2.0

    created = []
    for i, book in enumerate(book_list):
        row, col = divmod(i, n_cols)
        sx, sy, sz = book["size"]
        obj = books_module.create_book(book, images_dir=images_dir)
        obj.rotation_euler = rotation
        obj.location = (
            x0 + col * cell_x,
            y0 + row * cell_y,
            table_height + sy / 2.0 + eps,
        )
        make_rigid_active(obj, sx * sy * sz * density, "BOX")
        created.append(obj)

    return created


def settle_physics(objects, frames=SETTLE_FRAMES):
    """
    Step the rigid body simulation forward so gravity actually settles
    every book, then bake the resulting pose into each object's real
    transform with visual_transform_apply(): frame_set() alone only moves
    the *evaluated* depsgraph copy used for display, the object's own
    location/rotation stay at the un-settled spawn pose until baked -
    without this the saved .blend would only look settled while the
    timeline is scrubbed to a specific frame, not on reload or export.
    """
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
