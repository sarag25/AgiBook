"""
book_placer.py
==============
Genera il posizionamento casuale (ma riproducibile) dei libri sugli scaffali.

Uso diretto:
    python book_placer.py --seed 42

Uso da launch file (OpaqueFunction):
    from agibot_x2_pkg.book_placer import BookPlacer
    placer = BookPlacer(seed=42, shelf_x=1.0, shelf_y=0.0)
    placements = placer.generate()

Ogni placement è un dict con:
    name      : str   – nome univoco entità Gazebo  (es. "it_book_0")
    urdf      : str   – contenuto XML del URDF
    x, y, z   : float – posa nel world frame
    yaw       : float – rotazione attorno Z (radianti)
"""

from __future__ import annotations

import math
import random
import textwrap
from dataclasses import dataclass, field
from typing import List

# ─────────────────────────────────────────────────────────────────────────────
# Catalogo libri
# Dimensioni in metri: (larghezza_x, spessore_y, altezza_z)
# La mesh GLB è centrata nell'origine con l'asse Z verso l'alto.
# ─────────────────────────────────────────────────────────────────────────────
BOOK_CATALOG: dict[str, dict] = {
    # size allineate a environment/books/create_books.py BOOKS[*]["size"]
    # = bounding box reale dei .glb (2026-08-31: 11 voci su 15 erano
    # stantie, es. ballata 2.5 cm invece di 4.5, emma 3.5 invece di 1.9 -
    # collision e prese calcolate su misure inesistenti). Se rigeneri le
    # mesh cambiando le size la', aggiorna anche qui.
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

# ─────────────────────────────────────────────────────────────────────────────
# Geometria della libreria (deve coincidere con bookshelf.urdf)
# Tutte le quote sono nel frame LOCAL della libreria (origine = base_link).
# ─────────────────────────────────────────────────────────────────────────────
SHELF_SURFACES_Z = [0.022, 0.349, 0.671, 0.993]   # z del piano di appoggio
SHELF_CLEARANCE_Z = [0.305, 0.300, 0.300, 0.335]  # altezza utile per scomparto
SHELF_X_MIN =  -0.370   # limite sinistro interno
SHELF_X_MAX =   0.370   # limite destro interno
SHELF_Y_FRONT = 0.130   # y centro libro (fronte aperto), locale alla libreria

# Margini di sicurezza
X_MARGIN      = 0.005   # margine laterale da parete
BOOK_GAP_MIN  = 0.002   # spazio minimo tra libri
BOOK_GAP_MAX  = 0.010   # spazio massimo tra libri
YAW_MAX_DEG   = 3.0     # inclinazione massima (°) – effetto "libro storto"


# ─────────────────────────────────────────────────────────────────────────────
# Dataclass risultato
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class BookPlacement:
    name: str          # entity name in Gazebo
    book_key: str      # chiave nel catalogo
    urdf: str          # XML completo del robot
    x: float           # world frame (centro libro)
    y: float
    z: float
    yaw: float         # radianti
    local_x: float = 0.0   # frame locale della libreria (centro X libro)
    local_y: float = 0.0   # frame locale (centro Y libro)
    local_z: float = 0.0   # frame locale (BASE del libro = superficie ripiano)
    local_yaw: float = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Generatore URDF inline (per libri senza file .urdf nel package)
# ─────────────────────────────────────────────────────────────────────────────
def _make_urdf(entity_name: str, book_key: str) -> str:
    info = BOOK_CATALOG[book_key]
    sx, sy, sz = info["size"]
    m = info["mass"]
    # inertia box: ixx = m/12*(sy²+sz²), iyy = m/12*(sx²+sz²), izz = m/12*(sx²+sy²)
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


# ─────────────────────────────────────────────────────────────────────────────
# BookPlacer
# ─────────────────────────────────────────────────────────────────────────────
class BookPlacer:
    """
    Parametri
    ---------
    seed      : int   – seme per la riproducibilità (default 42)
    shelf_x   : float – posizione X della libreria nel world frame
    shelf_y   : float – posizione Y della libreria nel world frame
    shelf_yaw : float – rotazione della libreria attorno Z (radianti)
    books     : list  – sottoinsieme di chiavi da BOOK_CATALOG (None = tutti)
    """

    def __init__(
        self,
        seed: int = 42,
        shelf_x: float = 0.0,
        shelf_y: float = 0.0,
        shelf_yaw: float = 0.0,
        books: List[str] | None = None,
    ):
        self.rng = random.Random(seed)
        self.shelf_x = shelf_x
        self.shelf_y = shelf_y
        self.shelf_yaw = shelf_yaw
        self.books = list(books) if books else list(BOOK_CATALOG.keys())

    # ── posizionamento ────────────────────────────────────────────────────────
    def generate(self) -> List[BookPlacement]:
        """Ritorna la lista di BookPlacement per tutti i libri che entrano."""

        book_keys = self.books[:]
        self.rng.shuffle(book_keys)

        # Distribuisce i libri sui 4 scaffali in modo casuale ma bilanciato
        n_shelves = len(SHELF_SURFACES_Z)
        shelf_queues: list[list[str]] = [[] for _ in range(n_shelves)]
        for i, bk in enumerate(book_keys):
            shelf_queues[i % n_shelves].append(bk)
        # rimescola ulteriormente l'ordine dentro ogni scaffale
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

                # Il libro entra in altezza?
                if sz > clearance - 0.005:
                    continue  # troppo alto per questo scomparto, saltato

                # Il libro entra in larghezza? (spine-out: ogni libro occupa sy in X)
                if x_cursor + sy + X_MARGIN > SHELF_X_MAX:
                    break  # scaffale pieno

                # Posizione locale (frame libreria) – modalità spine-out:
                #   X locale: libri affiancati per spessore (sy)
                #   Y locale: profondità del libro verso il robot = sx (larghezza copertina)
                #   Z locale: centro del libro = superficie + metà altezza
                local_x = x_cursor + sy / 2.0
                local_y = SHELF_Y_FRONT - sx / 2.0
                local_z = z_surface + sz / 2.0

                # Piccola inclinazione casuale
                yaw_jitter = math.radians(
                    self.rng.uniform(-YAW_MAX_DEG, YAW_MAX_DEG)
                )

                # Trasforma in world frame (rotazione 2D attorno all'asse Z)
                cos_a = math.cos(self.shelf_yaw)
                sin_a = math.sin(self.shelf_yaw)
                world_x = self.shelf_x + cos_a * local_x - sin_a * local_y
                world_y = self.shelf_y + sin_a * local_x + cos_a * local_y
                world_z = local_z   # local_z è già il centro del libro (z_surface + sz/2)
                world_yaw = self.shelf_yaw + yaw_jitter

                # Nome univoco entità
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


# ─────────────────────────────────────────────────────────────────────────────
# Generatore URDF scena completa (robot + libreria + libri) per RViz
# ─────────────────────────────────────────────────────────────────────────────
def generate_scene_urdf(
    robot_urdf_path: str,
    placements: List[BookPlacement],
    shelf_x: float = 1.5,
    shelf_y: float = 0.0,
    shelf_yaw: float = math.pi / 2,
    robot_z: float = 0.93,
    book_face_yaw: float = math.pi / 2,  # 90° → spine verso robot
) -> str:
    """
    Genera un URDF unico per RViz: robot + libreria + libri.

    Layout di default:
      - Robot a (0, 0, robot_z) che guarda +X
      - Libreria a (shelf_x, 0, 0) con yaw=90°
        → il lato aperto della libreria (local +Y) punta verso -X (verso il robot)
        → le spine dei libri (±Y locale) puntano verso ±X mondo
        → il robot vede le spine guardando in +X

    I libri sono figli di bookshelf_link con coordinate LOCALI, quindi
    seguono automaticamente la rotazione della libreria senza calcoli aggiuntivi.

    Convenzione z libri: local_z = superficie ripiano (origine mesh GLB alla base).
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(robot_urdf_path)
    robot_root = tree.getroot()

    # Trova il link radice del robot
    child_links = {
        j.find('child').get('link')
        for j in robot_root.findall('joint')
        if j.find('child') is not None
    }
    all_links = {lnk.get('name') for lnk in robot_root.findall('link')}
    robot_root_link = next(iter(all_links - child_links))

    lines = ['<?xml version="1.0" encoding="utf-8"?>', '<robot name="scene">']

    # ── world frame ──────────────────────────────────────────────────────────
    lines.append('  <link name="world"/>')

    # ── robot ─────────────────────────────────────────────────────────────────
    lines.append('  <joint name="world_to_robot" type="fixed">')
    lines.append('    <parent link="world"/>')
    lines.append(f'    <child link="{robot_root_link}"/>')
    lines.append(f'    <origin xyz="0 0 {robot_z}" rpy="0 0 0"/>')
    lines.append('  </joint>')
    for child in robot_root:
        if child.tag in ('link', 'joint'):
            lines.append(ET.tostring(child, encoding='unicode'))

    # ── libreria ──────────────────────────────────────────────────────────────
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

    # ── libri (figli di bookshelf_link, coordinate locali) ───────────────────
    # local_x : centro libro lungo larghezza scaffale
    # local_y : centro libro lungo profondità scaffale
    # local_z : centro del libro in Z (= z_surface + sz/2)
    #           i GLB hanno origine al centro del bounding box
    for p in placements:
        lname = f"{p.name}_link"
        sx, sy, sz = BOOK_CATALOG[p.book_key]["size"]
        lines.append(f'  <link name="{lname}">')
        lines.append('    <visual>')
        # Rz(face_yaw)*Rx(90°): GLB Y→Z (libro in piedi), GLB +X→link +Y (dorso verso robot)
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
    URDF con SOLO libreria + libri (senza robot).
    Pubblicato su /scene_description da un secondo robot_state_publisher.
    """
    lines = ['<?xml version="1.0" encoding="utf-8"?>', '<robot name="scene_objects">']

    lines.append('  <link name="world"/>')

    # libreria
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

    # libri
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


# ─────────────────────────────────────────────────────────────────────────────
# CLI rapida per debug / ispezione layout
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Mostra il layout dei libri")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--shelf-x", type=float, default=0.0)
    parser.add_argument("--shelf-y", type=float, default=0.0)
    parser.add_argument("--shelf-yaw", type=float, default=0.0,
                        help="rotazione libreria in gradi")
    args = parser.parse_args()

    placer = BookPlacer(
        seed=args.seed,
        shelf_x=args.shelf_x,
        shelf_y=args.shelf_y,
        shelf_yaw=math.radians(args.shelf_yaw),
    )
    results = placer.generate()

    print(f"\nSeed {args.seed} → {len(results)} libri posizionati\n")
    print(f"{'Entità':<35} {'Scaffale':>8} {'x':>7} {'y':>7} {'z':>7} {'yaw°':>6}")
    print("-" * 75)
    for p in results:
        # ricava scaffale dall'altezza z
        for i, zs in enumerate(SHELF_SURFACES_Z):
            if p.z < zs + SHELF_CLEARANCE_Z[i]:
                shelf_num = i + 1
                break
        print(
            f"{p.name:<35} {shelf_num:>8} "
            f"{p.x:>7.3f} {p.y:>7.3f} {p.z:>7.3f} "
            f"{math.degrees(p.yaw):>6.1f}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Libri fisici di test + posizione di spawn del robot (2026-08-30)
# Fonte unica per rviz_gaz_control.launch.py (spawn), pick_place_teleop.py
# (tasti 1/2) e pick_test_book.py (presa automatica).
#
# ROBOT_SPAWN_X = -0.10 (era 0.0): con la cinematica ricavata dall'URDF
# (agibot_x2_pkg_py/arm_kinematics.py) il braccio destro riesce a fare una
# presa laterale pulita (dita lungo Y, avvicinamento quasi orizzontale)
# solo se il dorso del libro sta a ~0.30 m davanti al robot: a 0.25 m la
# spalla e' troppo vicina e alta e il gripper arriva solo da sopra.
# I libri stanno a x=0.28 (6 cm davanti alla fila dipinta a 0.40, con
# ~2 cm di sovrapposizione visiva sul retro, accettata) e a y=-0.20/-0.30:
# la zona davanti alla spalla destra, l'unica in cui il braccio destro
# lavora bene - il roll della spalla e' limitato a +0.061 rad e non puo'
# portare il braccio verso il centro del corpo.
# ─────────────────────────────────────────────────────────────────────────────
ROBOT_SPAWN_X = -0.10

# Camminata (2026-09-13, vedi Gazebo.md "Camminata"): il robot NASCE
# WALK_DISTANCE metri piu' indietro (spawn x = ROBOT_SPAWN_X - WALK_DISTANCE,
# default di rviz_gaz_control.launch.py walk_distance) e raggiunge la posa di
# lavoro (pelvis a ROBOT_SPAWN_X) camminando: walk_to_shelf porta il giunto
# virtuale base_x_joint esattamente a WALK_DISTANCE. Tutta la cinematica del
# braccio (robot_x = ROBOT_SPAWN_X) vale solo A CAMMINATA FINITA.
# walk_distance:=0 al launch = vecchio comportamento (nasce gia' davanti
# allo scaffale, walk_to_shelf non fa nulla).
WALK_DISTANCE = 1.5

# (nome entita' Gazebo, chiave BOOK_CATALOG, world x, world y)
TEST_BOOKS = [
    ("picktest_it",   "it_book",   0.28, -0.20),
    ("picktest_emma", "emma_book", 0.28, -0.30),
]


# ─────────────────────────────────────────────────────────────────────────────
# Scena "grasp_test" (2026-08-30, rivista 2026-08-31 = SCENA DI DEFAULT del
# launch): SOLO libreria + tavolo (modelli separati bookshelf.urdf/table.urdf,
# niente libri dipinti) e 6 entita' fisiche afferrabili sul primo scaffale
# dall'alto (ripiano alto, l'unico raggiungibile da braccio e camera di testa),
# lato destro: 4 libri + 2 oggetti.
# Libri scelti per la pipeline definitiva (TODO "Primo scaffale con 4 libri e
# 2 oggetti"): IT + trilogia Hunger Games + i due prequel (Ballata
# dell'usignolo 4.5 cm, Alba sulla mietitura 3.8 cm - misure delle mesh).
# Tutti sopra la chiusura minima delle dita (3 cm, vedi
# arm_kinematics.GRIPPER_MIN_GAP), quindi stringibili oltre che agganciabili.
# Libri a x=0.34 (interamente sul ripiano, che va da x=0.25 a 0.55), dorso
# verso il robot, 2 cm fra una mesh e l'altra (le collision sono piu'
# strette delle mesh, vedi collision_size: spazio reale per le dita
# = 2 cm + 2*BOOK_COLLISION_SIDE_MARGIN); oggetti al centro della
# profondita' (x=0.40), 2 cm dopo l'ultimo libro. Da destra (parete interna
# a y=-0.37) verso il centro: la fascia y<-0.15 e' quella delle prese pulite
# del braccio destro (Gazebo.md "Presa automatica"), la tazza a y>0 e' dal
# lato del braccio sinistro.
# Selezione: rviz_gaz_control.launch.py scene:=grasp_test (default) | full.
# (nome entita', chiave catalogo, kind, world x, world y)
# ─────────────────────────────────────────────────────────────────────────────
# Fronti ALLINEATI (2026-09-06, era x=0.34 per tutti = centri allineati):
# i libri hanno larghezze diverse (Hunger Games 20 cm contro i 15-15.5 degli
# altri) e con i centri sullo stesso asse HG sporgeva di ~2.5 cm. Ora tutti
# i dorsi stanno sul piano x = BOOKS_FRONT_X (2 cm dentro il bordo del
# ripiano, che parte a 0.25): centro = fronte + larghezza/2. Bonus: il punto
# di presa di pick_test_book (dorso + grasp_depth) ha ora la stessa x per
# tutti i libri. Mappamondo al posto della tazza (2026-09-06, "se la tazza
# da problemi cambia con un altro oggetto non cilindrico"): e' la
# decorazione piu' pesante (280 g, la tazza da 200 g veniva catapultata in
# cima al mobile dai vincoli DetachableJoint all'avvio), ha forma diversa
# da libri e portapenne (sfera su piedistallo) e con 8 cm di diametro entra
# ancora nella pinza (apertura max 8.4 cm). A y=+0.02 sta accanto al
# portapenne (che finisce a y=-0.027): ~7 mm di aria.
BOOKS_FRONT_X = 0.27


def _book_x(key: str) -> float:
    return round(BOOKS_FRONT_X + BOOK_CATALOG[key]["size"][0] / 2.0, 4)


# Posizioni y riviste il 2026-09-06 per lasciare SPAZIO ALLE DITA: la dita
# (1 cm di spessore) devono entrare ai lati dell'oggetto senza toccare i
# vicini, quindi ogni gap fra due oggetti (o fra oggetto e parete interna,
# a |y|=0.378) deve essere >= 2 cm. Prima Hunger Games stava a 8 mm dalla
# parete e portapenne/mappamondo a 7 mm l'uno dall'altro: nessuna apertura
# delle dita poteva entrarci (vedi approach_opening in arm_kinematics).
# Le decorazioni stanno a y >= -0.04, dove il braccio destro arriva solo
# ruotando la vita (pick_test_book ritenta l'IK con yaw libero). Il
# portapenne resta a x=0.40 (verificato: presa e uscita rettilinea a
# 0.5 mm); il mappamondo invece sta al FRONTE (x=0.31 = 0.27 + 0.04) come i
# libri: a x=0.40 e y=+0.05 la presa riusciva (yaw 0.86 rad) ma l'uscita
# rettilinea di 10-14 cm no (errore IK 27-57 mm), al fronte 1 mm.
GRASP_TEST_ENTITIES = [
    ("gt_hunger",  "hunger_games_book",     "book",       _book_x("hunger_games_book"),     -0.320),
    ("gt_it",      "it_book",               "book",       _book_x("it_book"),               -0.238),
    ("gt_ballata", "ballata_usignolo_book", "book",       _book_x("ballata_usignolo_book"), -0.168),
    ("gt_alba",    "alba_mietitura_book",   "book",       _book_x("alba_mietitura_book"),   -0.107),
    ("gt_pen",     "pen_holder",            "decoration", 0.40, -0.040),
    ("gt_globe",   "desk_globe",            "decoration", round(BOOKS_FRONT_X + 0.04, 4), 0.050),
]

# Meta' larghezza interna della libreria lungo y (bookshelf.urdf: interno
# 0.756 m): pareti a y = +-SHELF_HALF_INNER_WIDTH nel frame world con la
# libreria a yaw 90 gradi. Serve per il calcolo dello spazio libero ai
# lati di un oggetto (free_space_sides).
SHELF_HALF_INNER_WIDTH = 0.378


def free_space_sides(entity_name: str, scene: str = "grasp_test") -> tuple[float, float]:
    """Spazio libero (m) fra le facce laterali (mesh) dell'entita' e il
    vicino piu' prossimo (o la parete) verso +y e verso -y. Su questo
    pick_test_book decide quanto aprire le dita per entrare senza urtare."""
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
    """Topic del DetachableJoint dell'entita' `name`: fonte unica per il
    blocco <plugin> del launch (_test_book_urdf), per bridge_config e per
    GraspManagerNode. Cambiare qui = cambia ovunque."""
    return {"attach": f"/{name}/attach",
            "detach": f"/{name}/detach",
            "state":  f"/{name}/state"}

# Collision dei libri piu' STRETTA della mesh lungo lo spessore (TODO
# 2026-08-31: "box interna con collision, di larghezza minore della mesh,
# lunghezza ok"): larghezza copertina (sx) e altezza (sz) restano quelle
# della mesh (il libro appoggia alla quota giusta e sporge quanto si vede),
# lo spessore (sy) perde BOOK_COLLISION_SIDE_MARGIN per lato. Cosi' le dita
# (1 cm) entrano fra due libri anche se le mesh quasi si toccano, e chiudendo
# sulla collision le ganasce "affondano" un po' nella mesh invece di
# fermarsi a filo con contatti instabili. 3 mm per lato: Mietitura (3.8 cm)
# resta a 3.2 cm di collision, sopra la chiusura minima delle dita (3 cm).
# Solo libri: le decorazioni tengono il box pieno del catalogo.
# ─────────────────────────────────────────────────────────────────────────────
BOOK_COLLISION_SIDE_MARGIN = 0.003   # m per lato, lungo lo spessore
BOOK_COLLISION_MIN_THICKNESS = 0.012  # m, non scendere sotto (stabilita')


def test_entities(scene: str = "full"):
    """Entita' fisiche di test per la scena data, come lista di
    (nome, chiave catalogo, kind, world x, world y)."""
    if scene == "grasp_test":
        return list(GRASP_TEST_ENTITIES)
    return [(n, k, "book", x, y) for n, k, x, y in TEST_BOOKS]


def catalog_entry(kind: str, key: str) -> dict:
    """{"size": (x,y,z), "mass"} per libri (BOOK_CATALOG) o decorazioni
    (DECOR_CATALOG di manual_scene_placer, import pigro: quel modulo importa
    questo)."""
    if kind == "book":
        return BOOK_CATALOG[key]
    from agibot_x2_pkg.manual_scene_placer import DECOR_CATALOG
    return DECOR_CATALOG[key]


def collision_size(kind: str, key: str) -> tuple[float, float, float]:
    """Dimensioni (x, y, z) del box di collision spawnato in Gazebo per
    l'entita': = catalogo per le decorazioni, spessore ridotto per i libri
    (vedi BOOK_COLLISION_SIDE_MARGIN). E' lo spessore su cui chiudere le
    dita (teleop 'b', pick_test_book), NON quello della mesh."""
    sx, sy, sz = catalog_entry(kind, key)["size"]
    if kind != "book":
        return (sx, sy, sz)
    sy_c = max(BOOK_COLLISION_MIN_THICKNESS, sy - 2.0 * BOOK_COLLISION_SIDE_MARGIN)
    return (sx, round(sy_c, 4), sz)


MESH_DIR = {"book": "books", "decoration": "desk_decorations"}
