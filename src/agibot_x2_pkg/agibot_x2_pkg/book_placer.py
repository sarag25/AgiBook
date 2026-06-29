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
    "alba_mietitura_book":       {"size": (0.130, 0.022, 0.200), "mass": 0.25},
    "ballata_usignolo_book":     {"size": (0.135, 0.025, 0.205), "mass": 0.30},
    "black_widow_book":          {"size": (0.170, 0.012, 0.260), "mass": 0.20},
    "cane_stelle_book":          {"size": (0.130, 0.020, 0.195), "mass": 0.22},
    "cane_stelle_racconti_book": {"size": (0.130, 0.018, 0.195), "mass": 0.20},
    "cats_cradle_book":          {"size": (0.130, 0.022, 0.195), "mass": 0.25},
    "emma_book":                 {"size": (0.125, 0.035, 0.185), "mass": 0.35},
    "enciclopedia_animali_book": {"size": (0.250, 0.040, 0.295), "mass": 1.50},
    "enciclopedia_terra_vol1_book": {"size": (0.250, 0.040, 0.295), "mass": 1.50},
    "enciclopedia_terra_vol2_book": {"size": (0.250, 0.040, 0.295), "mass": 1.50},
    "fantasticos_4_book":        {"size": (0.170, 0.012, 0.260), "mass": 0.20},
    "hunger_games_book":         {"size": (0.200, 0.070, 0.205), "mass": 1.80},
    "it_book":                   {"size": (0.155, 0.055, 0.235), "mass": 1.20},
    "never_flinch_book":         {"size": (0.135, 0.022, 0.205), "mass": 0.28},
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
    x: float           # world frame
    y: float
    z: float
    yaw: float         # radianti


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
          <gazebo reference="base_link">
            <visual>
              <material>
                <diffuse>1 1 1 1</diffuse>
                <specular>0.1 0.1 0.1 1</specular>
              </material>
            </visual>
          </gazebo>
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

                # Il libro entra in larghezza?
                if x_cursor + sx + X_MARGIN > SHELF_X_MAX:
                    break  # scaffale pieno

                # Posizione locale (frame libreria)
                local_x = x_cursor + sx / 2.0
                local_y = SHELF_Y_FRONT - sy / 2.0
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
                world_z = local_z
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
                    )
                )

                gap = self.rng.uniform(BOOK_GAP_MIN, BOOK_GAP_MAX)
                x_cursor += sx + gap

        return placements


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
