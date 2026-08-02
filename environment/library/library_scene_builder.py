"""
Shared orchestration for the library scene: shelf configurations, book/
decoration placement maths and config validation. Extracted from
create_scene.py so the layout logic can be reused by both create_scene.py
(library-only scene, feeds the existing photo/manual_layout pipeline) and
create_full_scene.py (library + empty table) without duplicating ~200
lines of placement code - same reasoning already applied to
table/table_scene_builder.py for the two table-scene entry points.

Unlike table_scene_builder.py (kept generic/parametrized since it serves
two sibling entry points that only differ by rotation), this module owns
its create_bookshelf/create_books/create_desk_decorations imports
directly: there is exactly one library layout system in this repo, and
threading three module parameters through every helper would add ceremony
without a second caller that needs different modules. Callers only need
their own subdirectories ("bookshelf", "books", "desk_decorations") on
sys.path before importing this module - same convention create_scene.py
already used.
"""

import math
import random

import bpy      # import Blender Python API

import create_bookshelf as bookshelf
import create_books as books
import create_desk_decorations as decorations
import scene_physics as physics

X_MARGIN = 0.02        # gap from the interior side panels
BOOK_GAP = 0.006       # gap between adjacent books
DECOR_GAP = 0.015      # gap between the book row and the beside decorations
EPS = 0.001            # vertical play so the physics solver starts contact-free
BOOK_DENSITY = 600.0   # kg/m3

X_MIN = -bookshelf.HALF_W + X_MARGIN
X_MAX = bookshelf.HALF_W - X_MARGIN
SHELF_BACK_Y = bookshelf.Y_OFFSET + bookshelf.BOARD_T       # interior back wall
SHELF_FRONT_Y = bookshelf.Y_OFFSET + bookshelf.CASE_DEPTH   # front edge

CONFIGURATIONS = {
    # Original layout: books grouped by height/collection.
    # Shelf 0 has no front decorations: the encyclopedias are too deep.
    "classic": {
        "books": {
            0: ["enciclopedia_animali_book", "enciclopedia_terra_vol1_book",
                "enciclopedia_terra_vol2_book"],
            1: ["it_book", "never_flinch_book", "black_widow_book",
                "fantasticos_4_book"],
            2: ["hunger_games_book", "ballata_usignolo_book",
                "alba_mietitura_book"],
            3: ["werther_book", "emma_book", "cats_cradle_book",
                "cane_stelle_book", "cane_stelle_racconti_book"],
        },
        "beside": {
            0: ["potted_plant"],
            1: ["desk_globe"],
            2: ["coffee_mug", "pen_holder"],
            3: ["pen_holder"],
        },
        "front": {
            1: ["coaster", "paperweight_ball"],
            2: ["paperweight_ball"],
            3: ["coaster", "paperweight_ball"],
        },
    },
    # "classic" turned upside down: small books at the bottom, encyclopedias on the top shelf
    "flipped": {
        "books": {
            0: ["werther_book", "emma_book", "cats_cradle_book",
                "cane_stelle_book", "cane_stelle_racconti_book"],
            1: ["hunger_games_book", "ballata_usignolo_book",
                "alba_mietitura_book"],
            2: ["it_book", "never_flinch_book", "black_widow_book",
                "fantasticos_4_book"],
            3: ["enciclopedia_animali_book", "enciclopedia_terra_vol1_book",
                "enciclopedia_terra_vol2_book"],
        },
        "beside": {
            0: ["pen_holder"],
            1: ["coffee_mug", "pen_holder"],
            2: ["desk_globe"],
            3: ["potted_plant"],
        },
        "front": {
            0: ["coaster", "paperweight_ball"],
            1: ["paperweight_ball"],
            2: ["coaster", "paperweight_ball"],
        },
    },
    # Tallest books at the bottom, shortest at the top
    "by_height": {
        "books": {
            0: ["enciclopedia_animali_book", "enciclopedia_terra_vol1_book",
                "enciclopedia_terra_vol2_book", "black_widow_book",
                "fantasticos_4_book"],
            1: ["it_book", "never_flinch_book", "ballata_usignolo_book",
                "alba_mietitura_book"],
            2: ["hunger_games_book", "cats_cradle_book",
                "cane_stelle_book", "cane_stelle_racconti_book"],
            3: ["werther_book", "emma_book"],
        },
        "beside": {
            0: ["pen_holder"],
            1: ["desk_globe"],
            2: ["coffee_mug"],
            3: ["potted_plant", "pen_holder"],
        },
        "front": {
            1: ["coaster", "paperweight_ball"],
            2: ["paperweight_ball"],
            3: ["coaster", "paperweight_ball"],
        },
    },
    # Collections split across different shelves
    "mixed": {
        "books": {
            0: ["it_book", "ballata_usignolo_book", "cane_stelle_book",
                "emma_book"],
            1: ["enciclopedia_animali_book", "enciclopedia_terra_vol1_book",
                "werther_book", "cats_cradle_book"],
            2: ["hunger_games_book", "black_widow_book",
                "cane_stelle_racconti_book"],
            3: ["enciclopedia_terra_vol2_book", "never_flinch_book",
                "fantasticos_4_book", "alba_mietitura_book"],
        },
        "beside": {
            0: ["pen_holder"],
            1: ["coffee_mug"],
            2: ["potted_plant"],
            3: ["desk_globe"],
        },
        "front": {
            0: ["coaster", "paperweight_ball"],
            2: ["paperweight_ball"],
        },
    },
}


BESIDE_POOL = ["potted_plant", "desk_globe", "coffee_mug", "pen_holder"]
FRONT_POOL = ["coaster", "paperweight_ball"]


DECOR_RADIUS = {
    "pen_holder": 0.028,
    "paperweight_ball": 0.032,
    "desk_globe": 0.040,
    "potted_plant": 0.050,
    "coffee_mug": 0.060,
    "coaster": 0.050,
}

BOOKS_BY_NAME = {b["name"]: b for b in books.BOOKS}
DECOR_BY_NAME = {d["name"]: d for d in decorations.DECORATIONS}


def spine_line(book_names):
    """
    Per-shelf Y of the spine alignment line: the deepest book of the row
    just clears the back wall
    """
    max_depth = max(BOOKS_BY_NAME[n]["size"][0] for n in book_names)
    return SHELF_BACK_Y + max_depth + 0.005


def config_problems(cfg):
    """
    Dry-run of the layout maths on a configuration, without touching the
    scene. Returns a list of human-readable problems: books taller than
    the shelf clearance, rows (books + beside decorations) wider than the
    shelf, front decorations that place_front would have to skip.
    An empty list means every object will be placed.
    """
    problems = []
    for shelf_idx, names in cfg["books"].items():
        clearance = bookshelf.SHELF_CLEARANCE_Z[shelf_idx]
        for n in names:
            if BOOKS_BY_NAME[n]["size"][2] >= clearance:
                problems.append(
                    f"shelf {shelf_idx}: {n} taller than clearance {clearance} m")

        row_x_end = X_MIN + sum(
            BOOKS_BY_NAME[n]["size"][1] + BOOK_GAP for n in names)
        beside = cfg["beside"].get(shelf_idx, [])
        beside_x_end = row_x_end + DECOR_GAP + sum(
            2 * DECOR_RADIUS[n] + DECOR_GAP for n in beside)
        if beside_x_end > X_MAX:
            problems.append(
                f"shelf {shelf_idx}: books + beside decorations exceed the "
                f"shelf width by {beside_x_end - X_MAX:.3f} m")
            continue

        strip = SHELF_FRONT_Y - spine_line(names)
        segments = [[X_MIN, row_x_end], [beside_x_end, X_MAX]]
        for name in cfg["front"].get(shelf_idx, []):
            r = DECOR_RADIUS[name]
            if 2 * r > strip:
                problems.append(
                    f"shelf {shelf_idx}: {name} deeper than the front strip "
                    f"({2 * r:.3f} > {strip:.3f} m)")
                continue
            for seg in segments:
                if seg[1] - seg[0] >= 2 * r:
                    seg[0] += 2 * r + DECOR_GAP
                    break
            else:
                problems.append(
                    f"shelf {shelf_idx}: no free front segment for {name}")
    return problems


def random_configuration(seed):
    """
    Generate a random but always valid configuration: books shuffled over
    the four shelves (at least two per shelf), 1-2 bookend decorations and
    0-2 front decorations per shelf. Only layouts with no config_problems
    are accepted, so every object is guaranteed a spot.
    """
    rng = random.Random(seed)
    book_names = sorted(BOOKS_BY_NAME)
    for _ in range(1000):
        order = book_names[:]
        rng.shuffle(order)
        shelves = {i: [order[2 * i], order[2 * i + 1]] for i in range(4)}
        for name in order[8:]:
            shelves[rng.randrange(4)].append(name)
        for row in shelves.values():
            rng.shuffle(row)

        cfg = {
            "books": shelves,
            "beside": {i: [rng.choice(BESIDE_POOL)
                           for _ in range(rng.randint(1, 2))]
                       for i in range(4)},
            "front": {i: [rng.choice(FRONT_POOL)
                          for _ in range(rng.randint(0, 2))]
                      for i in range(4)},
        }
        if not config_problems(cfg):
            return cfg
    raise RuntimeError(f"no valid random configuration found for seed {seed}")


def get_configuration(config_name, seed=1):
    """
    Resolve config_name ("classic", "flipped", "by_height", "mixed" or
    "random") to a configuration dict, printing any problem a predefined
    configuration would have (offending objects are then skipped by the
    placement functions). seed is only used for "random".
    """
    if config_name == "random":
        return random_configuration(seed)
    cfg = CONFIGURATIONS[config_name]
    for problem in config_problems(cfg):
        print(f"  config warning: {problem}")
    return cfg


def place_books(shelf_idx, names, images_dir):
    """
    Line up the shelf's books side by side starting from the left panel,
    standing on the shelf surface with the spine facing the front (+Y).
    Spines are aligned on a per-shelf line placed so that the deepest book
    clears the back wall; the space left towards the front edge is the
    strip used for the front decorations.
    Returns (x_end, spine_y, created): where the row ends, the spine
    line, and the list of created book objects.
    """
    z_surface = bookshelf.SHELF_SURFACES_Z[shelf_idx]
    spine_y = spine_line(names)

    x_cursor = X_MIN
    created = []
    for name in names:
        book = BOOKS_BY_NAME[name]
        sx, sy, sz = book["size"]
        obj = books.create_book(book, images_dir=images_dir)
        # assign_materials puts the spine on +X (-X for manga), so Rz(+90°)
        # turns it towards the shelf front (+Y); manga rotate the other way
        angle = -math.pi / 2.0 if book.get("manga") else math.pi / 2.0
        obj.rotation_euler = (0.0, 0.0, angle)
        obj.location = (
            x_cursor + sy / 2.0,
            spine_y - sx / 2.0,
            z_surface + sz / 2.0 + EPS,
        )
        physics.make_rigid_active(obj, sx * sy * sz * BOOK_DENSITY, "BOX")
        created.append(obj)
        x_cursor += sy + BOOK_GAP

    return x_cursor, spine_y, created


def place_beside(shelf_idx, names, x_start):
    """
    Place decorations after the book row, resting on the shelf surface
    and centered in the shelf depth.
    Returns (x_end, created): the x where the occupied span ends (last
    footprint + gap), and the list of created decoration objects.
    """
    z_surface = bookshelf.SHELF_SURFACES_Z[shelf_idx]
    y_center = (SHELF_BACK_Y + SHELF_FRONT_Y) / 2.0
    x_cursor = x_start + DECOR_GAP
    created = []
    for name in names:
        r = DECOR_RADIUS[name]
        obj = decorations.create_decoration(
            DECOR_BY_NAME[name],
            location=(x_cursor + r, y_center, z_surface + EPS),
        )
        physics.make_rigid_active(obj, DECOR_BY_NAME[name]["mass"], "CONVEX_HULL")
        created.append(obj)
        x_cursor += 2 * r + DECOR_GAP
    return x_cursor, created


def place_front(shelf_idx, names, row_x_end, spine_y, beside_x_end):
    """
    Place decorations in the strip between the spine line and the front
    edge. Their footprints (DECOR_RADIUS) are laid out left to right in
    the free x segments of the strip - before and after the span taken by
    the beside decorations - separated by DECOR_GAP, so no two objects
    overlap. Decorations too tall for the strip depth or with no segment
    wide enough are skipped with a warning. Returns the list of created
    decoration objects.
    """
    z_surface = bookshelf.SHELF_SURFACES_Z[shelf_idx]
    strip = SHELF_FRONT_Y - spine_y
    y_center = (spine_y + SHELF_FRONT_Y) / 2.0
    segments = [[X_MIN, row_x_end], [beside_x_end, X_MAX]]

    created = []
    for name in names:
        r = DECOR_RADIUS[name]
        if 2 * r > strip:
            print(f"  skip {name} on shelf {shelf_idx}: footprint {2 * r:.3f} m "
                  f"> front strip {strip:.3f} m")
            continue
        for seg in segments:
            if seg[1] - seg[0] >= 2 * r:
                obj = decorations.create_decoration(
                    DECOR_BY_NAME[name],
                    location=(seg[0] + r, y_center, z_surface + EPS),
                )
                physics.make_rigid_active(obj, DECOR_BY_NAME[name]["mass"], "CONVEX_HULL")
                created.append(obj)
                seg[0] += 2 * r + DECOR_GAP
                break
        else:
            print(f"  skip {name} on shelf {shelf_idx}: no free segment "
                  f"wide enough for footprint {2 * r:.3f} m")
    return created


def build_library(images_dir, config_name="classic", seed=1):
    """
    Build the bookshelf (at the scene origin, see create_bookshelf.py) and
    fill every shelf following config_name (or a seeded random layout when
    config_name == "random"): books standing spine-out plus bookend/front
    decorations. Does not clear the scene or bake physics - the caller
    decides when (create_full_scene.py builds the table first, so the
    scene is only cleared once) and does the gravity bake once every
    object of the full scene has been created.

    Returns (label, objects): label is a human-readable string for
    logging, objects is every book/decoration created (rigid bodies
    already registered as ACTIVE), for the caller to settle with
    scene_physics.settle_physics().
    """
    bookshelf.build_bookshelf()
    cfg = get_configuration(config_name, seed)
    label = config_name if config_name != "random" else f"random (seed {seed})"

    objects = []
    for shelf_idx, names in cfg["books"].items():
        row_x_end, spine_y, book_objs = place_books(shelf_idx, names, images_dir)
        beside_x_end, beside_objs = place_beside(
            shelf_idx, cfg["beside"].get(shelf_idx, []), row_x_end)
        front_objs = place_front(
            shelf_idx, cfg["front"].get(shelf_idx, []), row_x_end, spine_y, beside_x_end)
        objects += book_objs + beside_objs + front_objs

    return label, objects
