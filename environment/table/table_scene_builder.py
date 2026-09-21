"""
Shared helpers for the "books on table" scenes (create_table_scene_retro.py and create_table_scene_cover.py):
grid layout sizing, book placement at a given rotation and the gravity settle/bake pass.
No bpy.context.space_data access at import time, so it is safe to import.
"""

import math

import bpy      # import Blender Python API

BOOK_GAP = 0.03        # gap between adjacent books on the grid
TABLE_MARGIN = 0.05    # border between the table edge and the outermost books
EPS = 0.001            # vertical play so the physics solver starts contact-free
BOOK_DENSITY = 600.0   # kg/m3
N_COLS = 5              # 15 books -> 5x3 grid

SETTLE_FRAMES = 60     # ~2.5 s at 24 fps


def grid_cell_size(book_list, gap=BOOK_GAP):
    """
    Grid cell size (X, Y) that fits the widest/tallest book lying flat
    (footprint is (sx, sz) with either the retro-up or the cover-up rotation)
    """
    max_w = max(b["size"][0] for b in book_list)
    max_h = max(b["size"][2] for b in book_list)
    return max_w + gap, max_h + gap


def table_footprint(book_list, n_cols=N_COLS, margin=TABLE_MARGIN):
    """
    Table top (width, depth) needed to fit all the books in an n_cols grid plus a margin
    """
    cell_x, cell_y = grid_cell_size(book_list)
    n_rows = math.ceil(len(book_list) / n_cols)
    return n_cols * cell_x + 2 * margin, n_rows * cell_y + 2 * margin


def make_rigid_active(obj, mass, shape):
    """
    Register obj as an active rigid body, so it reacts to gravity
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
    Place every book of books_module.BOOKS on a grid on the table, oriented by `rotation`
    (rotation_euler), `eps` above the surface so the solver starts contact-free.
    Return the created book objects
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
    Run the rigid body simulation so gravity settles the books, then bake the pose.
    frame_set() only moves the evaluated (display) copy: visual_transform_apply()
    writes the settled pose into each object's transform, so it survives reload and export.
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
