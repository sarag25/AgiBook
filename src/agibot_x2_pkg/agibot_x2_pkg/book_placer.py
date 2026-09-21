"""
Helpers for the scene objects: reproducible random book layout on the shelves (BookPlacer), book catalog,
scene URDFs for RViz and the physical test entities (grasp_test, ocr_test, full) shared by launch, bridge and pick nodes.
Usage: python book_placer.py --seed 42
       placements = BookPlacer(seed=42, shelf_x=1.0, shelf_y=0.0).generate()
"""

from __future__ import annotations

import math
import random
import textwrap
from dataclasses import dataclass, field
from typing import List

# book sizes (x cover width, y thickness, z height) in m = real .glb bounding boxes,
# same as environment/books/create_books.py BOOKS[*]["size"]; GLB centered, Z up
BOOK_CATALOG: dict[str, dict] = {
    "alba_mietitura_book":       {"size": (0.150, 0.038, 0.210), "mass": 0.45},
    "ballata_usignolo_book":     {"size": (0.150, 0.045, 0.210), "mass": 0.55},
    "black_widow_book":          {"size": (0.170, 0.023, 0.260), "mass": 0.20},
    "cane_stelle_book":          {"size": (0.148, 0.017, 0.210), "mass": 0.22},
    "cane_stelle_racconti_book": {"size": (0.148, 0.019, 0.210), "mass": 0.20},
    "cats_cradle_book":          {"size": (0.133, 0.015, 0.203), "mass": 0.25},
    "emma_book":                 {"size": (0.110, 0.019, 0.180), "mass": 0.35},
    "enciclopedia_animali_book": {"size": (0.230, 0.029, 0.280), "mass": 1.50},
    "enciclopedia_terra_vol1_book": {"size": (0.230, 0.027, 0.280), "mass": 1.50},
    "enciclopedia_terra_vol2_book": {"size": (0.230, 0.027, 0.280), "mass": 1.50},
    "fantasticos_4_book":        {"size": (0.170, 0.022, 0.260), "mass": 0.20},
    "hunger_games_book":         {"size": (0.200, 0.070, 0.205), "mass": 1.80},
    "it_book":                   {"size": (0.155, 0.055, 0.235), "mass": 1.20},
    "never_flinch_book":         {"size": (0.155, 0.039, 0.230), "mass": 0.28},
    "werther_book":              {"size": (0.120, 0.010, 0.190), "mass": 0.15},
}

# bookshelf geometry, LOCAL frame of the bookshelf (must match bookshelf.urdf)
from agibot_x2_pkg.scene_config import SHELF_SURFACES_Z, SHELF_HALF_INNER_WIDTH   # from bookshelf.urdf (scene_config)
SHELF_CLEARANCE_Z = [0.305, 0.300, 0.300, 0.335]  # usable height per compartment
SHELF_X_MIN =  -0.370   # inner left limit
SHELF_X_MAX =   0.370   # inner right limit
SHELF_Y_FRONT = 0.130   # book y at the open front, bookshelf frame

# safety margins
X_MARGIN      = 0.005   # side margin from the walls
BOOK_GAP_MIN  = 0.002   # min gap between books
BOOK_GAP_MAX  = 0.010   # max gap between books
YAW_MAX_DEG   = 3.0     # max random tilt (deg), "crooked book" effect


@dataclass
class BookPlacement:
    """
    One placed book, with world and bookshelf-local pose
    """
    name: str          # entity name in Gazebo
    book_key: str      # BOOK_CATALOG key
    urdf: str          # full URDF XML
    x: float           # world frame (book center)
    y: float
    z: float
    yaw: float         # rad
    local_x: float = 0.0   # bookshelf frame (book X center)
    local_y: float = 0.0   # bookshelf frame (book Y center)
    local_z: float = 0.0   # bookshelf frame (book Z center = surface + height/2)
    local_yaw: float = 0.0


def _make_urdf(entity_name: str, book_key: str) -> str:
    """
    Inline book URDF: mesh visual, box collision and box inertia
    """
    info = BOOK_CATALOG[book_key]
    sx, sy, sz = info["size"]
    m = info["mass"]
    ixx = m / 12.0 * (sy**2 + sz**2)
    iyy = m / 12.0 * (sx**2 + sz**2)
    izz = m / 12.0 * (sx**2 + sy**2)

    return textwrap.dedent(f"""\
        <?xml version="1.0" encoding="utf-8"?>
        <robot name="{entity_name}">
          <link name="base_link">
            <inertial>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <mass value="{m}"/>
              <inertia ixx="{ixx:.6f}" ixy="0" ixz="0"
                       iyy="{iyy:.6f}" iyz="0"
                       izz="{izz:.6f}"/>
            </inertial>
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <mesh filename="package://agibot_x2_pkg/meshes/books/{book_key}.glb"/>
              </geometry>
            </visual>
            <collision>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{sx} {sy} {sz}"/>
              </geometry>
            </collision>
          </link>
        </robot>
    """)


class BookPlacer:
    """
    Random but reproducible (seed) spine-out book layout for a bookshelf at (shelf_x, shelf_y, shelf_yaw [rad])
    books: subset of BOOK_CATALOG keys (None = all)
    """

    def __init__(
        self,
        seed: int = 42,
        shelf_x: float = 0.0,
        shelf_y: float = 0.0,
        shelf_yaw: float = 0.0,
        books: List[str] | None = None,
    ):
        """
        Store the seed, the bookshelf world pose and the book subset
        """
        self.rng = random.Random(seed)
        self.shelf_x = shelf_x
        self.shelf_y = shelf_y
        self.shelf_yaw = shelf_yaw
        self.books = list(books) if books else list(BOOK_CATALOG.keys())

    def generate(self) -> List[BookPlacement]:
        """
        BookPlacement list for every book that fits
        """

        book_keys = self.books[:]
        self.rng.shuffle(book_keys)

        # random but balanced split over the 4 shelves
        n_shelves = len(SHELF_SURFACES_Z)
        shelf_queues: list[list[str]] = [[] for _ in range(n_shelves)]
        for i, bk in enumerate(book_keys):
            shelf_queues[i % n_shelves].append(bk)
        for q in shelf_queues:
            self.rng.shuffle(q)

        placements: list[BookPlacement] = []
        counters: dict[str, int] = {}

        for shelf_idx, queue in enumerate(shelf_queues):
            z_surface = SHELF_SURFACES_Z[shelf_idx]
            clearance = SHELF_CLEARANCE_Z[shelf_idx]
            x_cursor = SHELF_X_MIN + X_MARGIN

            for book_key in queue:
                sx, sy, sz = BOOK_CATALOG[book_key]["size"]

                if sz > clearance - 0.005:
                    continue  # too tall for this compartment

                # spine-out: each book takes sy along X
                if x_cursor + sy + X_MARGIN > SHELF_X_MAX:
                    break  # shelf full

                # local frame: X side by side by thickness, Y depth = cover width sx, Z = book center
                local_x = x_cursor + sy / 2.0
                local_y = SHELF_Y_FRONT - sx / 2.0
                local_z = z_surface + sz / 2.0

                yaw_jitter = math.radians(
                    self.rng.uniform(-YAW_MAX_DEG, YAW_MAX_DEG)
                )

                # world frame: 2D rotation about Z
                cos_a = math.cos(self.shelf_yaw)
                sin_a = math.sin(self.shelf_yaw)
                world_x = self.shelf_x + cos_a * local_x - sin_a * local_y
                world_y = self.shelf_y + sin_a * local_x + cos_a * local_y
                world_z = local_z   # local_z is already the book center
                world_yaw = self.shelf_yaw + yaw_jitter

                counters[book_key] = counters.get(book_key, -1) + 1
                entity_name = f"{book_key}_{counters[book_key]}"

                placements.append(
                    BookPlacement(
                        name=entity_name,
                        book_key=book_key,
                        urdf=_make_urdf(entity_name, book_key),
                        x=world_x,
                        y=world_y,
                        z=world_z,
                        yaw=world_yaw,
                        local_x=local_x,
                        local_y=local_y,
                        local_z=local_z,
                        local_yaw=yaw_jitter,
                    )
                )

                gap = self.rng.uniform(BOOK_GAP_MIN, BOOK_GAP_MAX)
                x_cursor += sy + gap

        return placements


def generate_scene_urdf(
    robot_urdf_path: str,
    placements: List[BookPlacement],
    shelf_x: float = 1.5,
    shelf_y: float = 0.0,
    shelf_yaw: float = math.pi / 2,
    robot_z: float = 0.93,
    book_face_yaw: float = math.pi / 2,  # 90 deg -> spine towards the robot
) -> str:
    """
    Single RViz URDF with robot + bookshelf + books
    Robot at (0, 0, robot_z) facing +X; bookshelf at yaw 90 deg so its open side (local +Y)
    faces the robot and the spines are visible looking along +X.
    Books are children of bookshelf_link in LOCAL coordinates, so they follow its rotation.
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(robot_urdf_path)
    robot_root = tree.getroot()

    # robot root link
    child_links = {
        j.find('child').get('link')
        for j in robot_root.findall('joint')
        if j.find('child') is not None
    }
    all_links = {lnk.get('name') for lnk in robot_root.findall('link')}
    robot_root_link = next(iter(all_links - child_links))

    lines = ['<?xml version="1.0" encoding="utf-8"?>', '<robot name="scene">']

    # world frame
    lines.append('  <link name="world"/>')

    # robot
    lines.append('  <joint name="world_to_robot" type="fixed">')
    lines.append('    <parent link="world"/>')
    lines.append(f'    <child link="{robot_root_link}"/>')
    lines.append(f'    <origin xyz="0 0 {robot_z}" rpy="0 0 0"/>')
    lines.append('  </joint>')
    for child in robot_root:
        if child.tag in ('link', 'joint'):
            lines.append(ET.tostring(child, encoding='unicode'))

    # bookshelf
    lines.append('  <link name="bookshelf_link">')
    lines.append('    <visual>')
    lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
    lines.append('      <geometry>')
    lines.append('        <mesh filename="package://agibot_x2_pkg/meshes/bookshelf.glb"/>')
    lines.append('      </geometry>')
    lines.append('    </visual>')
    lines.append('  </link>')
    lines.append('  <joint name="world_to_bookshelf" type="fixed">')
    lines.append('    <parent link="world"/>')
    lines.append('    <child link="bookshelf_link"/>')
    lines.append(f'    <origin xyz="{shelf_x:.4f} {shelf_y:.4f} 0.0" rpy="0 0 {shelf_yaw:.6f}"/>')
    lines.append('  </joint>')

    # books (children of bookshelf_link; local_z = book center, GLB origin at the bbox center)
    for p in placements:
        lname = f"{p.name}_link"
        sx, sy, sz = BOOK_CATALOG[p.book_key]["size"]
        lines.append(f'  <link name="{lname}">')
        lines.append('    <visual>')
        # Rz(face_yaw)*Rx(90): GLB Y -> Z (upright), GLB +X -> link +Y (spine towards the robot)
        lines.append(f'      <origin xyz="0 0 0" rpy="1.5707963 0 {book_face_yaw:.6f}"/>')
        lines.append('      <geometry>')
        lines.append(f'        <mesh filename="package://agibot_x2_pkg/meshes/books/{p.book_key}.glb"/>')
        lines.append('      </geometry>')
        lines.append('    </visual>')
        lines.append(f'    <collision>')
        lines.append(f'      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'      <geometry><box size="{sx} {sy} {sz}"/></geometry>')
        lines.append(f'    </collision>')
        lines.append('  </link>')
        lines.append(f'  <joint name="shelf_to_{p.name}" type="fixed">')
        lines.append('    <parent link="bookshelf_link"/>')
        lines.append(f'    <child link="{lname}"/>')
        lines.append(f'    <origin xyz="{p.local_x:.4f} {p.local_y:.4f} {p.local_z:.4f}" rpy="0 0 {p.local_yaw:.4f}"/>')
        lines.append('  </joint>')

    lines.append('</robot>')
    return '\n'.join(lines)


def generate_objects_urdf(
    placements: List[BookPlacement],
    shelf_x: float = 1.5,
    shelf_y: float = 0.0,
    shelf_yaw: float = math.pi / 2,
    book_face_yaw: float = 0.0,
) -> str:
    """
    URDF with only bookshelf + books (no robot), for a second robot_state_publisher on /scene_description
    """
    lines = ['<?xml version="1.0" encoding="utf-8"?>', '<robot name="scene_objects">']

    lines.append('  <link name="world"/>')

    # bookshelf
    lines.append('  <link name="bookshelf_link">')
    lines.append('    <visual>')
    lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
    lines.append('      <geometry>')
    lines.append('        <mesh filename="package://agibot_x2_pkg/meshes/bookshelf.glb"/>')
    lines.append('      </geometry>')
    lines.append('    </visual>')
    lines.append('  </link>')
    lines.append('  <joint name="world_to_bookshelf" type="fixed">')
    lines.append('    <parent link="world"/>')
    lines.append('    <child link="bookshelf_link"/>')
    lines.append(f'    <origin xyz="{shelf_x:.4f} {shelf_y:.4f} 0.0" rpy="0 0 {shelf_yaw:.6f}"/>')
    lines.append('  </joint>')

    # books
    for p in placements:
        lname = f"{p.name}_link"
        sx, sy, sz = BOOK_CATALOG[p.book_key]["size"]
        lines.append(f'  <link name="{lname}">')
        lines.append('    <visual>')
        lines.append(f'      <origin xyz="0 0 0" rpy="1.5707963 0 {book_face_yaw:.6f}"/>')
        lines.append('      <geometry>')
        lines.append(f'        <mesh filename="package://agibot_x2_pkg/meshes/books/{p.book_key}.glb"/>')
        lines.append('      </geometry>')
        lines.append('    </visual>')
        lines.append('  </link>')
        lines.append(f'  <joint name="shelf_to_{p.name}" type="fixed">')
        lines.append('    <parent link="bookshelf_link"/>')
        lines.append(f'    <child link="{lname}"/>')
        lines.append(f'    <origin xyz="{p.local_x:.4f} {p.local_y:.4f} {p.local_z:.4f}" rpy="0 0 {p.local_yaw:.4f}"/>')
        lines.append('  </joint>')

    lines.append('</robot>')
    return '\n'.join(lines)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Show the book layout")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--shelf-x", type=float, default=0.0)
    parser.add_argument("--shelf-y", type=float, default=0.0)
    parser.add_argument("--shelf-yaw", type=float, default=0.0,
                        help="bookshelf rotation in degrees")
    args = parser.parse_args()

    placer = BookPlacer(
        seed=args.seed,
        shelf_x=args.shelf_x,
        shelf_y=args.shelf_y,
        shelf_yaw=math.radians(args.shelf_yaw),
    )
    results = placer.generate()

    print(f"\nSeed {args.seed} → {len(results)} books placed\n")
    print(f"{'Entità':<35} {'Scaffale':>8} {'x':>7} {'y':>7} {'z':>7} {'yaw°':>6}")
    print("-" * 75)
    for p in results:
        # shelf from z
        for i, zs in enumerate(SHELF_SURFACES_Z):
            if p.z < zs + SHELF_CLEARANCE_Z[i]:
                shelf_num = i + 1
                break
        print(
            f"{p.name:<35} {shelf_num:>8} "
            f"{p.x:>7.3f} {p.y:>7.3f} {p.z:>7.3f} "
            f"{math.degrees(p.yaw):>6.1f}"
        )


# physical test books and robot spawn, shared by rviz_gaz_control.launch.py, pick_place_teleop.py and pick_test_book.py
ROBOT_SPAWN_X = -0.10   # clean side grasp only with the spine ~0.30 m in front of the robot (at 0.25 only from above)

# robot spawns WALK_DISTANCE behind ROBOT_SPAWN_X and walks there (walk_to_shelf drives base_x_joint to it);
# arm kinematics (robot_x = ROBOT_SPAWN_X) hold only after the walk; walk_distance:=0 spawns at the shelf
WALK_DISTANCE = 1.5

# (Gazebo entity name, BOOK_CATALOG key, world x, world y); x 0.28 and y -0.20/-0.30: in front of the right
# shoulder, the only area where the right arm works well (shoulder roll limited to +0.061 rad)
TEST_BOOKS = [
    ("picktest_it",   "it_book",   0.28, -0.20),
    ("picktest_emma", "emma_book", 0.28, -0.30),
]


# "grasp_test" scene (launch default, scene:=grasp_test | full): bookshelf + table only, and 6 graspable entities on the
# top shelf (the only one reachable by arm and head camera): 4 books + 2 objects, (name, catalog key, kind, world x, world y).
# All books are thicker than the minimum finger closure (3 cm, arm_kinematics.GRIPPER_MIN_GAP).
# Book spines are aligned on x = BOOKS_FRONT_X (2 cm inside the shelf edge at 0.25): center = front + width/2,
# so the pick_test_book grasp point has the same x for every book.
# Desk globe instead of the mug: heaviest decoration (280 g; the 200 g mug was catapulted by the DetachableJoint
# constraints at startup), non-cylindrical, 8 cm diameter fits the gripper (max opening 8.4 cm).
BOOKS_FRONT_X = 0.27


def _book_x(key: str) -> float:
    """
    World x of a book center with its spine on BOOKS_FRONT_X
    """
    return round(BOOKS_FRONT_X + BOOK_CATALOG[key]["size"][0] / 2.0, 4)


# y positions: the free 41.2 cm (75.6 cm inner - 34.4 cm of objects) split into 7 equal 5.89 cm gaps, room for the
# gripper palm (right_gripper_base_link, 10 cm along the gap axis) that touches neighbors when the waist turns a lot,
# not only for the fingers (1 cm). Pen holder at x=0.40 (grasp and straight exit verified); globe at the front like the books.
GRASP_TEST_ENTITIES = [
    ("gt_hunger",  "hunger_games_book",     "book",       _book_x("hunger_games_book"),     -0.2841),
    ("gt_it",      "it_book",               "book",       _book_x("it_book"),               -0.1628),
    ("gt_ballata", "ballata_usignolo_book", "book",       _book_x("ballata_usignolo_book"), -0.0539),
    ("gt_alba",    "alba_mietitura_book",   "book",       _book_x("alba_mietitura_book"),    0.0464),
    ("gt_pen",     "pen_holder",            "decoration", 0.40,  0.1523),
    ("gt_globe",   "desk_globe",            "decoration", round(BOOKS_FRONT_X + 0.04, 4),    0.2791),
]

def free_space_sides(entity_name: str, scene: str = "grasp_test") -> tuple[float, float]:
    """
    Free space (m) between the entity's mesh sides and the nearest neighbor or wall, as (+y, -y)
    pick_test_book uses it to decide how far to open the fingers.
    """
    ents = test_entities(scene)
    me = next(e for e in ents if e[0] == entity_name)
    _n, key, kind, _x, y = me
    half = catalog_entry(kind, key)["size"][1] / 2.0
    lo, hi = y - half, y + half
    plus = [SHELF_HALF_INNER_WIDTH - hi]
    minus = [lo + SHELF_HALF_INNER_WIDTH]
    for n, k, kd, _x2, y2 in ents:
        if n == entity_name:
            continue
        h2 = catalog_entry(kd, k)["size"][1] / 2.0
        if y2 > y:
            plus.append((y2 - h2) - hi)
        else:
            minus.append(lo - (y2 + h2))
    return (min(plus), min(minus))


def entity_topics(name: str) -> dict:
    """
    DetachableJoint topics of entity `name`: single source for the launch <plugin> block,
    bridge_config and GraspManagerNode
    """
    return {"attach": f"/{name}/attach",
            "detach": f"/{name}/detach",
            "state":  f"/{name}/state",
            # second DetachableJoint, to the left finger
            "attach_left": f"/{name}/attach_left",
            "detach_left": f"/{name}/detach_left",
            "state_left":  f"/{name}/state_left"}

# book collision thinner than the mesh along the thickness only (books only; decorations keep the full box):
# fingers fit between books whose meshes almost touch, and closing jaws sink slightly into the mesh instead of
# stopping flush with unstable contacts; 3 mm per side keeps Alba (3.8 cm) at 3.2 cm, above the 3 cm finger closure
BOOK_COLLISION_SIDE_MARGIN = 0.003   # m per side, along the thickness
BOOK_COLLISION_MIN_THICKNESS = 0.012  # m, lower bound for stability


# "ocr_test" scene: all 15 catalog books on the top shelf, fronts on BOOKS_FRONT_X, 19 mm apart (the depth detector
# separates from 12 mm), 8 mm from the walls; for head-camera OCR tests, not for grasping (fingers need 2 cm per side)
OCR_TEST_ENTITIES = [
    ("ot_alba_mietitura", "alba_mietitura_book", "book", _book_x("alba_mietitura_book"), +0.3510),
    ("ot_ballata_usignolo", "ballata_usignolo_book", "book", _book_x("ballata_usignolo_book"), +0.2905),
    ("ot_black_widow", "black_widow_book", "book", _book_x("black_widow_book"), +0.2375),
    ("ot_cane_stelle", "cane_stelle_book", "book", _book_x("cane_stelle_book"), +0.1985),
    ("ot_cane_stelle_racconti", "cane_stelle_racconti_book", "book", _book_x("cane_stelle_racconti_book"), +0.1615),
    ("ot_cats_cradle", "cats_cradle_book", "book", _book_x("cats_cradle_book"), +0.1255),
    ("ot_emma", "emma_book", "book", _book_x("emma_book"), +0.0895),
    ("ot_enciclopedia_animali", "enciclopedia_animali_book", "book", _book_x("enciclopedia_animali_book"), +0.0465),
    ("ot_enciclopedia_terra_vol1", "enciclopedia_terra_vol1_book", "book", _book_x("enciclopedia_terra_vol1_book"), -0.0005),
    ("ot_enciclopedia_terra_vol2", "enciclopedia_terra_vol2_book", "book", _book_x("enciclopedia_terra_vol2_book"), -0.0465),
    ("ot_fantasticos_4", "fantasticos_4_book", "book", _book_x("fantasticos_4_book"), -0.0900),
    ("ot_hunger_games", "hunger_games_book", "book", _book_x("hunger_games_book"), -0.1550),
    ("ot_it", "it_book", "book", _book_x("it_book"), -0.2365),
    ("ot_never_flinch", "never_flinch_book", "book", _book_x("never_flinch_book"), -0.3025),
    ("ot_werther", "werther_book", "book", _book_x("werther_book"), -0.3460),
]


def test_entities(scene: str = "full"):
    """
    Physical test entities of a scene as (name, catalog key, kind, world x, world y)
    """
    if scene == "grasp_test":
        return list(GRASP_TEST_ENTITIES)
    if scene == "ocr_test":
        return list(OCR_TEST_ENTITIES)
    return [(n, k, "book", x, y) for n, k, x, y in TEST_BOOKS]


def catalog_entry(kind: str, key: str) -> dict:
    """
    {"size": (x, y, z), "mass"} from BOOK_CATALOG or DECOR_CATALOG
    (lazy import: manual_scene_placer imports this module)
    """
    if kind == "book":
        return BOOK_CATALOG[key]
    from agibot_x2_pkg.manual_scene_placer import DECOR_CATALOG
    return DECOR_CATALOG[key]


def collision_size(kind: str, key: str) -> tuple[float, float, float]:
    """
    (x, y, z) of the Gazebo collision box: catalog size for decorations, reduced thickness for books
    This, not the mesh, is the thickness the fingers close on (teleop 'b', pick_test_book).
    """
    sx, sy, sz = catalog_entry(kind, key)["size"]
    if kind != "book":
        return (sx, sy, sz)
    sy_c = max(BOOK_COLLISION_MIN_THICKNESS, sy - 2.0 * BOOK_COLLISION_SIDE_MARGIN)
    return (sx, round(sy_c, 4), sz)


MESH_DIR = {"book": "books", "decoration": "desk_decorations"}
