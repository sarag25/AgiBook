"""
manual_scene_placer.py
=======================
Legge manual_layout.json (esportato da environment/export_manual_layout.py)
e genera i placement world-frame per libri e decorazioni, così la scena
spawnata in Gazebo corrisponde esattamente a quella fotografata in Blender
da environment/setup_render_camera.py — stessa disposizione, stesse
coordinate locali rispetto alla libreria.

Alternativa "layout fisso" a BookPlacer (layout casuale): usa le posizioni
REALI degli oggetti in Blender (dopo l'eventuale assestamento fisico)
invece di ricalcolare un layout da zero.

Uso da launch file:
    from agibot_x2_pkg.manual_scene_placer import ManualScenePlacer
    placer = ManualScenePlacer(
        "manual_layout.json", shelf_x=1.5, shelf_y=0.0, shelf_yaw=math.pi / 2,
    )
    placements = placer.generate()
"""

from __future__ import annotations

import json
import math
import textwrap
from dataclasses import dataclass
from typing import List

from agibot_x2_pkg.book_placer import BOOK_CATALOG

# ─────────────────────────────────────────────────────────────────────────────
# Catalogo decorazioni da scrivania.
# "size" è un bounding box approssimato (larghezza_x, profondità_y, altezza_z)
# in metri, derivato a mano dai parametri delle primitive in
# environment/desk_decorations/create_desk_decorations.py (stesso pattern di
# BOOK_CATALOG in book_placer.py: nessuna fonte unica di verità tra Blender e
# ROS2 in questo repo, vedi Architecture.md). Usato solo per la collision box
# nell'URDF: la visual usa sempre il .glb reale.
# ─────────────────────────────────────────────────────────────────────────────
DECOR_CATALOG: dict[str, dict] = {
    "pen_holder":       {"size": (0.056, 0.056, 0.095), "mass": 0.12},
    "paperweight_ball": {"size": (0.064, 0.064, 0.064), "mass": 0.15},
    "desk_globe":       {"size": (0.080, 0.080, 0.122), "mass": 0.28},
    "potted_plant":     {"size": (0.100, 0.100, 0.140), "mass": 0.22},
    "coffee_mug":       {"size": (0.090, 0.075, 0.090), "mass": 0.20},
    "coaster":          {"size": (0.100, 0.100, 0.008), "mass": 0.05},
}

_MESH_DIR = {"book": "books", "decoration": "desk_decorations"}


def _catalog_for(kind: str) -> dict:
    return BOOK_CATALOG if kind == "book" else DECOR_CATALOG


def read_table_pose(layout_path: str):
    """
    Look for a "table" entry in manual_layout.json (written by
    environment/export_manual_layout.py when the scene has an empty
    staging table - e.g. environment/create_full_scene.py). Returns
    (local_x, local_y, local_yaw) relative to SHELF_ORIGIN, or None if the
    layout has no table (e.g. one exported from library_scene.blend,
    library-only - not an error, just nothing to spawn).
    """
    with open(layout_path, "r", encoding="utf-8") as f:
        entries = json.load(f)
    for e in entries:
        if e["kind"] == "table":
            return e["local_x"], e["local_y"], e["local_yaw"]
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Dataclass risultato
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class ScenePlacement:
    name: str          # nome univoco entità Gazebo (es. "it_book_0")
    kind: str          # "book" | "decoration"
    object_key: str    # chiave nel catalogo (BOOK_CATALOG o DECOR_CATALOG)
    urdf: str          # XML completo del robot
    x: float           # world frame (centro oggetto)
    y: float
    z: float
    yaw: float         # radianti
    local_x: float      # frame locale della libreria, letto da Blender
    local_y: float
    local_z: float
    local_yaw: float


# ─────────────────────────────────────────────────────────────────────────────
# Generatore URDF inline (stesso schema di book_placer._make_urdf, esteso
# alle decorazioni: visual = mesh reale, collision = box approssimato)
#
# <static>true</static> (2026-08-10): libri e decorazioni erano spawnati
# come rigid body dinamici con una collision box solo approssimata
# (BOOK_CATALOG/DECOR_CATALOG). La loro posa però è già quella di riposo
# calcolata dalla fisica *reale* di Blender (mesh esatte, non box) - appena
# la simulazione Gazebo partiva, le box approssimate (impacchettate a
# millimetri l'una dall'altra per costruzione) si trovavano leggermente
# compenetrate tra loro e con lo scaffale, e il solver le respingeva con un
# impulso violento: libri sparsi a terra lontano dallo scaffale, decorazioni
# volanti. Stessa soluzione già usata per libreria/tavolo (`bookshelf.urdf`/
# `table.urdf`, entrambi `<static>true</static>`): dato che la scena deve
# corrispondere esattamente a quella fotografata/costruita in Blender (vedi
# Gazebo.md), non c'è motivo di ri-simulare da zero un equilibrio che Blender
# ha già risolto con precisione maggiore.
# ─────────────────────────────────────────────────────────────────────────────
def _make_urdf(entity_name: str, kind: str, object_key: str) -> str:
    info = _catalog_for(kind)[object_key]
    sx, sy, sz = info["size"]
    m = info["mass"]
    ixx = m / 12.0 * (sy**2 + sz**2)
    iyy = m / 12.0 * (sx**2 + sz**2)
    izz = m / 12.0 * (sx**2 + sy**2)
    mesh_dir = _MESH_DIR[kind]

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
                <mesh filename="package://agibot_x2_pkg/meshes/{mesh_dir}/{object_key}.glb"/>
              </geometry>
            </visual>
            <collision>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{sx} {sy} {sz}"/>
              </geometry>
            </collision>
          </link>
          <gazebo>
            <static>true</static>
          </gazebo>
        </robot>
    """)


# ─────────────────────────────────────────────────────────────────────────────
# ManualScenePlacer
# ─────────────────────────────────────────────────────────────────────────────
class ManualScenePlacer:
    """
    Parametri
    ---------
    layout_path : str   – percorso di manual_layout.json
                           (environment/export_manual_layout.py)
    shelf_x   : float – posizione X della libreria nel world frame
    shelf_y   : float – posizione Y della libreria nel world frame
    shelf_yaw : float – rotazione della libreria attorno Z (radianti)

    shelf_x/y/yaw devono essere gli stessi valori passati allo spawn della
    libreria (bookshelf.urdf) e, per coerenza fotografica, gli stessi usati
    per calcolare la posa camera in setup_render_camera.py rispetto a
    SHELF_ORIGIN.
    """

    def __init__(
        self,
        layout_path: str,
        shelf_x: float = 0.0,
        shelf_y: float = 0.0,
        shelf_yaw: float = 0.0,
    ):
        self.layout_path = layout_path
        self.shelf_x = shelf_x
        self.shelf_y = shelf_y
        self.shelf_yaw = shelf_yaw

    def generate(self) -> List[ScenePlacement]:
        """Ritorna la lista di ScenePlacement per ogni oggetto in manual_layout.json."""

        with open(self.layout_path, "r", encoding="utf-8") as f:
            entries = json.load(f)

        cos_a = math.cos(self.shelf_yaw)
        sin_a = math.sin(self.shelf_yaw)
        counters: dict[str, int] = {}
        placements: List[ScenePlacement] = []

        for e in entries:
            kind = e["kind"]
            object_key = e["object_key"]

            if kind == "table":
                # Not a book/decoration: spawned separately as urdf/table.urdf
                # by manual_scene.launch.py via table_world_pose() below, not
                # through the generic per-object catalog + inline URDF path
                # (the table already has a real multi-part URDF on disk, no
                # need to synthesize one from a size/mass catalog entry).
                continue

            if object_key not in _catalog_for(kind):
                print(
                    f"  skip {e['source_object']}: '{object_key}' non è nel "
                    f"catalogo {kind} (BOOK_CATALOG/DECOR_CATALOG)"
                )
                continue

            local_x, local_y, local_z = e["local_x"], e["local_y"], e["local_z"]
            local_yaw = e["local_yaw"]

            # Trasforma in world frame (rotazione 2D attorno all'asse Z),
            # stessa formula di BookPlacer.generate().
            world_x = self.shelf_x + cos_a * local_x - sin_a * local_y
            world_y = self.shelf_y + sin_a * local_x + cos_a * local_y
            world_z = local_z
            world_yaw = self.shelf_yaw + local_yaw

            counters[object_key] = counters.get(object_key, -1) + 1
            entity_name = f"{object_key}_{counters[object_key]}"

            placements.append(
                ScenePlacement(
                    name=entity_name,
                    kind=kind,
                    object_key=object_key,
                    urdf=_make_urdf(entity_name, kind, object_key),
                    x=world_x,
                    y=world_y,
                    z=world_z,
                    yaw=world_yaw,
                    local_x=local_x,
                    local_y=local_y,
                    local_z=local_z,
                    local_yaw=local_yaw,
                )
            )

        return placements

    def table_world_pose(self):
        """
        World-frame (x, y, yaw) for the empty staging table, read from the
        same manual_layout.json this placer parses for books/decorations
        (kind == "table") and transformed with the same rotation formula
        used in generate(). Returns None if the layout has no table entry.
        """
        pose = read_table_pose(self.layout_path)
        if pose is None:
            return None

        local_x, local_y, local_yaw = pose
        cos_a = math.cos(self.shelf_yaw)
        sin_a = math.sin(self.shelf_yaw)
        world_x = self.shelf_x + cos_a * local_x - sin_a * local_y
        world_y = self.shelf_y + sin_a * local_x + cos_a * local_y
        world_yaw = self.shelf_yaw + local_yaw
        return world_x, world_y, world_yaw


# ─────────────────────────────────────────────────────────────────────────────
# CLI rapida per debug / ispezione layout
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Mostra il layout manuale caricato da JSON")
    parser.add_argument("layout_json", help="Percorso a manual_layout.json")
    parser.add_argument("--shelf-x", type=float, default=0.0)
    parser.add_argument("--shelf-y", type=float, default=0.0)
    parser.add_argument("--shelf-yaw", type=float, default=0.0,
                        help="rotazione libreria in gradi")
    args = parser.parse_args()

    placer = ManualScenePlacer(
        args.layout_json,
        shelf_x=args.shelf_x,
        shelf_y=args.shelf_y,
        shelf_yaw=math.radians(args.shelf_yaw),
    )
    results = placer.generate()

    print(f"\n{len(results)} oggetti caricati da {args.layout_json}\n")
    print(f"{'Entità':<25} {'Tipo':<12} {'x':>7} {'y':>7} {'z':>7} {'yaw°':>6}")
    print("-" * 65)
    for p in results:
        print(
            f"{p.name:<25} {p.kind:<12} "
            f"{p.x:>7.3f} {p.y:>7.3f} {p.z:>7.3f} "
            f"{math.degrees(p.yaw):>6.1f}"
        )
