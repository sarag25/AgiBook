"""
Script for Blender 5.1.2 to build the complete scene handed off to Gazebo:
the library (bookshelf + books + decorations, same layout system as
create_scene.py) plus an empty staging table at the robot's right, close
enough that the robot can reach it with a turn instead of walking, so the
arrangement can later be spawned in Gazebo with the robot already facing
the shelf, matching the "photo as if the robot took it" requirement (see
ROBOT_TO_SHELF_DISTANCE below).

Robot has a fixed base in Gazebo (world->pelvis fixed joint, see
x2_hand_gazebo.urdf) and never moves its legs - the request was for the
robot to reach both the shelf and the table without walking, so the whole
layout is built close enough for the arm alone to cover both (see
Gazebo.md for the reach numbers this was checked against).

Thin orchestrator: reuses library/library_scene_builder.py (bookshelf +
books + decorations) and table/create_table.py (empty table) rather than
duplicating either - same "thin entry point over a shared builder" pattern
as create_scene.py and the two create_table_scene_*.py scripts. Gravity is
respected for real: every book/decoration is baked to its settled resting
pose (scene_physics.settle_physics) before saving, not left for a manual
"Press Play".

Uso: apri questo script nello Scripting tab di Blender ed esegui (Run
Script / Alt+P). Salva full_scene.blend accanto a questo file.
"""

import importlib
import os
import sys

import bpy      # import Blender Python API

# bpy.path.abspath() (not os.path.abspath()): text.filepath can be a path
# relative to the current .blend ("//" prefix) once the .blend has already
# been saved before this script is opened from the file browser - see
# create_table_scene_retro.py for the bug this caused there.
SCRIPT_DIR = os.path.dirname(bpy.path.abspath(bpy.context.space_data.text.filepath))
for _sub in ("bookshelf", "books", "desk_decorations", "table", "library"):
    _dir = os.path.join(SCRIPT_DIR, _sub)
    if _dir not in sys.path:
        sys.path.append(_dir)
if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)

import create_bookshelf as bookshelf
import create_table as table
import library_scene_builder as library
import scene_physics as physics

for _mod in (bookshelf, table, library, physics):
    importlib.reload(_mod)

BOOKS_IMAGES_DIR = os.path.join(SCRIPT_DIR, "books")
BLEND_OUTPUT_PATH = os.path.join(SCRIPT_DIR, "full_scene.blend")

LIBRARY_CONFIG = "classic"   # same options as create_scene.ACTIVE_CONFIG
RANDOM_SEED = 1

# ---------------------------------------------------------------------------
# Layout relative to the robot.
#
# The bookshelf is always built at the scene origin by build_bookshelf()
# (front opening facing +Y, see create_bookshelf.py) - this script doesn't
# move it, same as create_scene.py. World placement in Gazebo is applied
# later at spawn time via shelf_x/shelf_y/shelf_yaw (manual_scene.launch.py),
# the same convention already used by the library-only pipeline
# (setup_render_camera.py / export_manual_layout.py) - see Gazebo.md.
#
# ROBOT_POSITION marks where the robot stands in this local frame:
# ROBOT_TO_SHELF_DISTANCE in front of the shelf along +Y. Unlike the 1.5 m
# used by setup_render_camera.CAMERA_DISTANCE (a framing distance for a
# clean orthographic photo, unrelated to the robot), this is the robot's
# actual stand-off, kept small on purpose (2026-08-10, see Gazebo.md
# "Raggiungibilita del braccio"): the robot has a fixed base in Gazebo
# (world->pelvis fixed joint in x2_hand_gazebo.urdf) and must reach both
# the shelf and the table without ever moving its legs, so it has to stand
# close enough for its ~0.50 m arm+hand span to cover them.
#
# TABLE_POSITION sits TABLE_LATERAL_DISTANCE to the robot's *right*
# instead of behind it, so reaching it only takes a turn, not a walk
# ("evitare troppi spostamenti"). The robot faces -Y here (towards the
# shelf); with +Z up, right-hand rule puts "right" on local -X (same
# convention as facing North/+Y -> right is +X, then turning 180 deg to
# face South/-Y flips it to -X). TABLE_WIDTH/TABLE_DEPTH match
# table.DEFAULT_WIDTH/DEFAULT_DEPTH (1.20 x 0.90 m) on purpose: that's the
# footprint already baked into the committed src/agibot_x2_pkg/meshes/table.glb
# (exported by table/create_table.py's standalone main()) - building the
# table at a different size here would make this scene disagree with the
# mesh Gazebo actually spawns, see Gazebo.md.
#
# The table's Y is pinned to the *shelf* (SHELF_FRONT_Y + TABLE_TO_SHELF_DISTANCE),
# not to the robot's Y. TABLE_LATERAL_DISTANCE/TABLE_TO_SHELF_DISTANCE were
# both tightened on 2026-08-10 together with ROBOT_TO_SHELF_DISTANCE (same
# "robot doesn't walk" requirement): close enough to the robot's right arm
# to reach the near edge, but still far enough from the shelf's own
# footprint that the two don't overlap (checked as axis-aligned bounding
# boxes in world frame - see Gazebo.md for the numbers).
# ---------------------------------------------------------------------------
ROBOT_TO_SHELF_DISTANCE = 0.25     # robot stand-off from the shelf front (was 1.5 m)
# 1.05, not 0.85 (2026-08-11): with 0.85 the table's near edge sat exactly
# inside the shelf's own Y-span in world frame, reading as "in front of the
# shelf" from most camera angles even though there was no real 3D overlap
# (checked as axis-aligned bounding boxes) - moved further right so the
# table's footprint fully clears the shelf's Y-span with margin. Costs some
# reach margin (checked: still within the ~0.50 m estimated arm+hand span,
# see Gazebo.md "Raggiungibilita del braccio") but not reach itself, since
# the nearest reachable point of the table stays at local x=0 regardless.
TABLE_LATERAL_DISTANCE = 1.05      # table center, to the robot's right (local -X) (was 1.0 m)
# 0.65, not the 0.55 first tried (2026-08-10): the reach distance to the
# table's near edge doesn't depend on this value (the nearest reachable
# point stays at local x=0 regardless, see Gazebo.md), so there was no
# reason not to widen the shelf/table gap once the first render showed it
# reading as "touching" on screen - 0.65 doubles the clearance (0.20 m
# instead of 0.10 m) for free.
TABLE_TO_SHELF_DISTANCE = 0.65     # table center, in front of the shelf (was 0.9 m)

TABLE_WIDTH = table.DEFAULT_WIDTH  # 1.20 m - matches the already-exported table.glb
TABLE_DEPTH = table.DEFAULT_DEPTH  # 0.90 m

# Own height override (2026-08-10), NOT table.TABLE_HEIGHT: that shared
# constant (0.50 m) is still used as-is by the book-photography table
# scenes (create_table_scene_retro.py/create_table_scene_cover.py) and
# stays untouched. This staging table needs to sit near shoulder height
# instead, since the robot has a fixed base right next to it and never
# bends its legs (see Gazebo.md "Raggiungibilita del braccio") - passed
# explicitly to table.build_table() in place_table() below.
GAZEBO_TABLE_HEIGHT = 0.75         # was table.TABLE_HEIGHT (0.50 m)

ROBOT_POSITION = (0.0, library.SHELF_FRONT_Y + ROBOT_TO_SHELF_DISTANCE, 0.0)
TABLE_POSITION = (
    ROBOT_POSITION[0] - TABLE_LATERAL_DISTANCE,
    library.SHELF_FRONT_Y + TABLE_TO_SHELF_DISTANCE,
    0.0,
)


def add_robot_reference(location):
    """
    Plain-axes Empty marking where the robot stands, as a visual/
    documentation anchor inside the .blend (no robot mesh exists in the
    Blender pipeline - see Blender.md). Named so it can never be picked up
    by export_bookshelf()/export_table(): both select objects by the
    "shelf_"/"table_" name prefix, which "robot_reference" doesn't match.
    """
    empty = bpy.data.objects.new("robot_reference", None)
    empty.empty_display_type = "PLAIN_AXES"
    empty.empty_display_size = 0.3
    empty.location = location
    bpy.context.collection.objects.link(empty)
    return empty


def place_table(location):
    """
    Build the empty staging table and move it into place. build_table()
    always creates its parts at the world origin (same convention as
    build_bookshelf()), so the whole group is translated afterwards; the
    table is a passive rigid body (see create_table.add_board), its
    resting pose does not depend on gravity settling, only on
    build_table() already placing the legs' bottom on the floor.
    """
    parts = table.build_table(width=TABLE_WIDTH, depth=TABLE_DEPTH, height=GAZEBO_TABLE_HEIGHT)
    for obj in parts:
        obj.location.x += location[0]
        obj.location.y += location[1]
        obj.location.z += location[2]
    return parts


def main():
    """
    Clear the scene, build the library (bookshelf + books + decorations)
    at the origin, add the empty table at the robot's right, mark the
    robot's own position, bake gravity for every book/decoration, then
    save full_scene.blend.
    """
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    bookshelf.clear_scene()

    label, library_objects = library.build_library(
        BOOKS_IMAGES_DIR, config_name=LIBRARY_CONFIG, seed=RANDOM_SEED)

    place_table(TABLE_POSITION)
    add_robot_reference(ROBOT_POSITION)

    physics.settle_physics(library_objects)

    print(f"\nScene assembled [{label}]: bookshelf + {len(library_objects)} "
          f"books/decorations + empty table at "
          f"({TABLE_POSITION[0]:.2f}, {TABLE_POSITION[1]:.2f}). Robot "
          f"reference at ({ROBOT_POSITION[0]:.2f}, {ROBOT_POSITION[1]:.2f}). "
          "Gravity baked.")

    bpy.ops.wm.save_as_mainfile(filepath=BLEND_OUTPUT_PATH)
    print(f"Blend file saved to: {BLEND_OUTPUT_PATH}")


if __name__ == "__main__":
    main()
