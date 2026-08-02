"""
Script Blender per preparare ed eseguire il render "fotografico" della
libreria da dare in pasto a SAM. Crea/riposiziona una camera ortografica
allineata al fronte della libreria, imposta illuminazione diffusa e
parametri di render adatti a detection/OCR, poi salva il PNG e la posa
della camera relativa alla libreria (utile in futuro per proiettare gli
slot noti nell'immagine).

Uso: apri il .blend con i libri già posizionati (dopo aver eventualmente
girato export_manual_layout.py), incolla/apri questo script nello
Scripting tab di Blender ed esegui (Run Script / Alt+P).
Output in render_output/ accanto al file .blend.
"""
import json
import math
import os

import bpy
from mathutils import Vector

# ── configurazione: adatta ai nomi reali della tua scena ─────────────────
# La libreria è composta da più parti senza genitore comune (tutte a
# rotazione 0°): elenco i prefissi dei nomi invece di un singolo oggetto.
SHELF_PART_PREFIXES = (
    "shelf_back", "shelf_board_", "shelf_side_left", "shelf_side_right",
)
# Deve coincidere con SHELF_ORIGIN in export_manual_layout.py
SHELF_ORIGIN = (0.0, 0.0, 0.0)
CAMERA_NAME = "SAM_Camera"
if not bpy.data.filepath:
    raise RuntimeError(
        "Il file .blend non è ancora stato salvato: OUTPUT_DIR non può "
        "essere calcolato relativo al file (bpy.data.filepath è vuoto) e "
        "finirebbe nella working directory del processo Blender, spesso "
        "non scrivibile. Salva il .blend (Ctrl+S) prima di eseguire lo script."
    )
OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(bpy.data.filepath)), "render_output"
)
CAMERA_DISTANCE = 1.5        # metri davanti al fronte scaffale
COVER_ROUGHNESS = 0.9        # opacizza le copertine per evitare riflessi speculari
FRAME_MARGIN = 1.05          # 5% di margine attorno alla libreria
RESOLUTION_LONG_EDGE = 2000  # px sul lato lungo dell'immagine

try:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
except PermissionError as exc:
    raise PermissionError(
        f"Permessi insufficienti per creare/scrivere in {OUTPUT_DIR}: "
        "verifica che la cartella del file .blend sia scrivibile "
        "dall'utente che esegue Blender (su Docker/WSL può servire un "
        "chown/chmod sul mount, o salvare il .blend in un percorso nativo "
        "invece che su un mount Windows)."
    ) from exc


def get_shelf_parts():
    parts = [
        obj for obj in bpy.data.objects
        if obj.type == 'MESH' and obj.name.startswith(SHELF_PART_PREFIXES)
    ]
    if not parts:
        raise RuntimeError(
            "Nessuna parte della libreria trovata con i prefissi "
            f"{SHELF_PART_PREFIXES}. Modifica SHELF_PART_PREFIXES in cima allo script."
        )
    return parts


def frame_bounds(parts):
    """Bounding box mondo unione di tutte le parti della libreria."""
    coords = []
    for obj in parts:
        coords += [obj.matrix_world @ v.co for v in obj.data.vertices]

    xs = [c.x for c in coords]
    ys = [c.y for c in coords]
    zs = [c.z for c in coords]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def setup_camera(shelf_parts):
    (xmin, ymin, zmin), (xmax, ymax, zmax) = frame_bounds(shelf_parts)
    width = xmax - xmin
    height = zmax - zmin
    center_x = (xmin + xmax) / 2.0
    center_z = (zmin + zmax) / 2.0
    front_y = ymax + CAMERA_DISTANCE  # +Y = lato aperto/copertine, da create_books.py

    print(
        f"[setup_render_camera] bounding box libreria: "
        f"width(X)={width:.3f}m height(Z)={height:.3f}m "
        f"center=({center_x:.3f}, {center_z:.3f})"
    )

    # Risoluzione proporzionata all'aspect ratio reale della libreria: evita
    # il disallineamento tra ortho_scale e resolution_x/y che tagliava
    # l'immagine (libreria portrait vs risoluzione landscape fissa).
    if height >= width:
        res_y = RESOLUTION_LONG_EDGE
        res_x = max(1, round(RESOLUTION_LONG_EDGE * width / height))
    else:
        res_x = RESOLUTION_LONG_EDGE
        res_y = max(1, round(RESOLUTION_LONG_EDGE * height / width))
    bpy.context.scene.render.resolution_x = res_x
    bpy.context.scene.render.resolution_y = res_y

    cam_data = bpy.data.cameras.get(CAMERA_NAME) or bpy.data.cameras.new(CAMERA_NAME)
    cam_data.type = 'ORTHO'
    # sensor_fit fissato sull'asse dominante: con resolution_x/y proporzionati
    # a width/height, l'altro asse si adatta automaticamente senza crop.
    if height >= width:
        cam_data.sensor_fit = 'VERTICAL'
        cam_data.ortho_scale = height * FRAME_MARGIN
    else:
        cam_data.sensor_fit = 'HORIZONTAL'
        cam_data.ortho_scale = width * FRAME_MARGIN

    cam_obj = bpy.data.objects.get(CAMERA_NAME)
    if cam_obj is None:
        cam_obj = bpy.data.objects.new(CAMERA_NAME, cam_data)
        bpy.context.collection.objects.link(cam_obj)

    cam_obj.location = (center_x, front_y, center_z)
    # guarda in -Y (verso la libreria), nessun roll/tilt
    cam_obj.rotation_euler = (math.radians(90.0), 0.0, math.radians(180.0))
    bpy.context.scene.camera = cam_obj

    # rotazione libreria = 0°, quindi il frame locale è il frame mondo
    # traslato di SHELF_ORIGIN (stessa convenzione di export_manual_layout.py)
    cam_local_pose = cam_obj.matrix_world.copy()
    cam_local_pose.translation -= Vector(SHELF_ORIGIN)
    return cam_obj, cam_local_pose


def setup_lighting():
    for name, loc, rot_x in [
        ("SAM_Light_Center", (0.0, 1.0, 1.0), 45.0),
        ("SAM_Light_Left",   (-1.5, 0.8, 0.8), 60.0),
        ("SAM_Light_Right",  (1.5, 0.8, 0.8), 60.0),
    ]:
        light_data = bpy.data.lights.get(name) or bpy.data.lights.new(name, type='AREA')
        light_data.energy = 300.0
        light_data.size = 1.5

        light_obj = bpy.data.objects.get(name)
        if light_obj is None:
            light_obj = bpy.data.objects.new(name, light_data)
            bpy.context.collection.objects.link(light_obj)
        light_obj.location = loc
        light_obj.rotation_euler = (math.radians(rot_x), 0.0, 0.0)


def setup_world_background():
    world = bpy.context.scene.world or bpy.data.worlds.new("SAM_World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = (0.85, 0.85, 0.85, 1.0)
        bg.inputs[1].default_value = 1.0


def reduce_cover_gloss():
    """Alza la Roughness sulle copertine (materiali *_cover/_retro/_spine
    creati da create_books.py) per evitare riflessi che nascondono testo."""
    for mat in bpy.data.materials:
        if mat.name.endswith(("_cover", "_retro", "_spine")) and mat.use_nodes:
            bsdf = mat.node_tree.nodes.get("Principled BSDF")
            if bsdf:
                bsdf.inputs["Roughness"].default_value = COVER_ROUGHNESS


def setup_render_settings():
    # NB: resolution_x/y sono già impostate da setup_camera() in proporzione
    # all'aspect ratio della libreria - non toccarle qui altrimenti si
    # torna al crop causato da un aspect ratio fisso non coerente.
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 128
    scene.cycles.use_denoising = True
    scene.render.image_settings.file_format = 'PNG'
    scene.view_settings.view_transform = 'Standard'


def main():
    shelf_parts = get_shelf_parts()
    cam_obj, cam_local_pose = setup_camera(shelf_parts)
    setup_lighting()
    setup_world_background()
    reduce_cover_gloss()
    setup_render_settings()

    out_png = os.path.join(OUTPUT_DIR, "library_photo.png")
    bpy.context.scene.render.filepath = out_png
    bpy.ops.render.render(write_still=True)

    loc = cam_local_pose.to_translation()
    quat = cam_local_pose.to_quaternion()
    pose_json = {
        "camera_local_to_bookshelf": {
            "x": loc.x, "y": loc.y, "z": loc.z,
            "qx": quat.x, "qy": quat.y, "qz": quat.z, "qw": quat.w,
        },
        "ortho_scale": cam_obj.data.ortho_scale,
        "resolution_x": bpy.context.scene.render.resolution_x,
        "resolution_y": bpy.context.scene.render.resolution_y,
    }
    pose_path = os.path.join(OUTPUT_DIR, "camera_pose.json")
    with open(pose_path, "w", encoding="utf-8") as f:
        json.dump(pose_json, f, indent=2)

    print(f"Render salvato in: {out_png}")
    print(f"Posa camera salvata in: {pose_path}")


if __name__ == "__main__":
    main()
