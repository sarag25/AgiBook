"""
manual_scene.launch.py
=======================
Spawna in Gazebo la libreria con libri e decorazioni nella disposizione
ESATTA fotografata in Blender da environment/setup_render_camera.py,
leggendo manual_layout.json (prodotto da environment/export_manual_layout.py).

Alternativa a spawn_books.launch.py (layout casuale di BookPlacer): qui il
layout non è generato, è letto dalle posizioni reali degli oggetti in
Blender, così la scena in Gazebo corrisponde punto per punto alla foto già
scattata (che resta l'unica foto "pulita" della scena, senza distorsione e
con le texture dei libri: il render Gazebo del robot le perderebbe).

Dal 2026-08-02 spawna anche il tavolo di staging vuoto (urdf/table.urdf),
se presente nel layout - vedi environment/create_full_scene.py e
Gazebo.md. Se il world caricato è quello di default (worlds/bookshelf.world,
che contiene già una libreria/tavolo placeholder) la libreria/tavolo reali
spawnati qui si sovrappongono al placeholder: lancia gazebo.launch.py con
world:=empty.world per evitarlo.

Uso:
    # Gazebo già avviato (es. tramite gazebo.launch.py)
    ros2 launch agibot_x2_pkg manual_scene.launch.py \\
        layout_json:=/percorso/a/manual_layout.json \\
        shelf_x:=1.5 shelf_y:=0.0 shelf_yaw_deg:=90.0

Nota: shelf_x/shelf_y/shelf_yaw_deg devono combaciare con quelli usati per
calcolare la posa camera in setup_render_camera.py (stesso SHELF_ORIGIN),
altrimenti la scena spawnata è ruotata/traslata rispetto alla foto.

Precondizioni:
    - Gazebo già avviato
    - Il package agibot_x2_pkg è nel workspace compilato
    - manual_layout.json generato dalla STESSA disposizione fotografata
      (rilancia export_manual_layout.py se la scena Blender cambia)
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agibot_x2_pkg.manual_scene_placer import ManualScenePlacer


# ─────────────────────────────────────────────────────────────────────────────
# Helper: nodo spawn_entity per un singolo modello
# ─────────────────────────────────────────────────────────────────────────────
def _spawn_node(entity_name: str, urdf_path: str,
                 x: float, y: float, z: float, yaw: float) -> Node:
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{entity_name}",
        arguments=[
            "-world", "bookshelf_world",
            # ^ 2026-08-11: senza -world, "create" prova ad AUTO-rilevare il
            # mondo interrogando la lista dei mondi attivi ("Requesting list
            # of world names" in log) - quella query non torna mai risposta
            # in alcuni ambienti Docker/WSL2 (discovery GZ Transport rotta,
            # tipicamente multicast UDP bloccato), quindi il nodo resta
            # bloccato per sempre PRIMA ancora di provare lo spawn vero.
            # "bookshelf_world" e' il nome dichiarato sia in bookshelf.world
            # sia in empty.world (vedi il commento in empty.world - lasciato
            # identico apposta, gia' usato hardcoded nei topic del bridge in
            # gazebo.launch.py), quindi e' sicuro darlo per scontato invece
            # di scoprirlo a runtime.
            "-name",  entity_name,
            "-file",  urdf_path,
            "-x",     str(round(x,   4)),
            "-y",     str(round(y,   4)),
            "-z",     str(round(z,   4)),
            "-Y",     str(round(yaw, 4)),
        ],
        output="screen",
    )


# ─────────────────────────────────────────────────────────────────────────────
# OpaqueFunction: viene eseguita a runtime, con accesso ai LaunchConfig
# ─────────────────────────────────────────────────────────────────────────────
def _spawn_all(context, *args, **kwargs):
    pkg = get_package_share_directory("agibot_x2_pkg")

    layout_json = LaunchConfiguration("layout_json").perform(context)
    if not layout_json:
        raise RuntimeError(
            "layout_json non specificato. Esempio:\n"
            "  ros2 launch agibot_x2_pkg manual_scene.launch.py "
            "layout_json:=/percorso/a/manual_layout.json"
        )

    shelf_x   = float(LaunchConfiguration("shelf_x").perform(context))
    shelf_y   = float(LaunchConfiguration("shelf_y").perform(context))
    shelf_yaw = math.radians(
        float(LaunchConfiguration("shelf_yaw_deg").perform(context))
    )

    actions = []

    # ── 1. Spawna la libreria ────────────────────────────────────────────────
    bookshelf_urdf = os.path.join(pkg, "urdf", "bookshelf.urdf")
    actions.append(
        _spawn_node(
            entity_name="bookshelf",
            urdf_path=bookshelf_urdf,
            x=shelf_x,
            y=shelf_y,
            z=0.0,
            yaw=shelf_yaw,
        )
    )

    # ── 2. Carica il layout manuale (libri + decorazioni) ───────────────────
    placer = ManualScenePlacer(
        layout_json,
        shelf_x=shelf_x,
        shelf_y=shelf_y,
        shelf_yaw=shelf_yaw,
    )
    placements = placer.generate()

    # ── 2b. Tavolo di staging vuoto, se presente nel layout ──────────────────
    # A differenza di libri/decorazioni ha una sola posa (non un catalogo
    # mesh/massa per-oggetto) e un URDF reale già pronto su disco, quindi si
    # spawna direttamente come la libreria al passo 1, non tramite
    # ManualScenePlacer.generate() (che lo salta apposta, vedi
    # manual_scene_placer.py). Assente nei manual_layout.json più vecchi
    # (esportati da library_scene.blend, solo libreria): in quel caso
    # table_world_pose() ritorna None e semplicemente non si spawna nulla.
    table_pose = placer.table_world_pose()
    if table_pose is not None:
        table_x, table_y, table_yaw = table_pose
        table_urdf = os.path.join(pkg, "urdf", "table.urdf")
        actions.append(
            _spawn_node(
                entity_name="table",
                urdf_path=table_urdf,
                x=table_x,
                y=table_y,
                z=0.0,
                yaw=table_yaw,
            )
        )

    # ── 3. Spawna ogni oggetto ───────────────────────────────────────────────
    tmp_dir = tempfile.mkdtemp(prefix="manual_scene_spawn_")

    for p in placements:
        tmp_path = os.path.join(tmp_dir, f"{p.name}.urdf")
        with open(tmp_path, "w", encoding="utf-8") as f:
            f.write(p.urdf)

        actions.append(
            _spawn_node(
                entity_name=p.name,
                urdf_path=tmp_path,
                x=p.x,
                y=p.y,
                z=p.z,
                yaw=p.yaw,
            )
        )

    n_books = sum(1 for p in placements if p.kind == "book")
    n_decor = sum(1 for p in placements if p.kind == "decoration")
    table_info = f"tavolo=({table_pose[0]:.2f},{table_pose[1]:.2f})" if table_pose else "tavolo=assente"
    print(
        f"\n[manual_scene] libreria=({shelf_x:.2f},{shelf_y:.2f})  "
        f"libri={n_books}  decorazioni={n_decor}  {table_info}  layout={layout_json}\n"
    )

    return actions


# ─────────────────────────────────────────────────────────────────────────────
# LaunchDescription
# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "layout_json",
            default_value="",
            description=(
                "Percorso assoluto a manual_layout.json, salvato accanto al "
                "file .blend da environment/export_manual_layout.py"
            ),
        ),
        DeclareLaunchArgument(
            "shelf_x",
            default_value="0.0",
            description="Posizione X della libreria nel world frame (metri)",
        ),
        DeclareLaunchArgument(
            "shelf_y",
            default_value="0.0",
            description="Posizione Y della libreria nel world frame (metri)",
        ),
        DeclareLaunchArgument(
            "shelf_yaw_deg",
            default_value="0.0",
            description="Rotazione della libreria attorno Z (gradi)",
        ),

        OpaqueFunction(function=_spawn_all),
    ])
