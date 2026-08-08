"""
Script Blender per preparare ed eseguire il render dall'alto di una scena
"libri sul tavolo" (create_table_scene_retro.py / create_table_scene_cover.py),
da dare in pasto a SAM/OCR come si fa con la libreria (setup_render_camera.py).

A differenza dello scaffale (vista frontale, oggetti in piedi), qui la
camera guarda dritta verso il basso: inquadra tutta la scena (tavolo +
libri, non c'è altro) invece di filtrare per prefisso come
SHELF_PART_PREFIXES nello script della libreria.

Generico per le due scene tavolo: non conosce quale delle due (retro-up o
cover-up) sia aperta, il nome di output deriva dal file .blend corrente
(vedi output_stem()), così lo stesso script funziona su entrambe senza
modifiche e senza sovrascrivere l'output dell'altra.

Uso: apri il .blend della scena tavolo (table_scene_retro.blend o
table_scene_cover.blend, già con libri posizionati e fisica assestata),
incolla/apri questo script nello Scripting tab di Blender ed esegui (Run
Script / Alt+P). Output in render_output/ accanto al file .blend.
"""
import json
import math
import os

import bpy

CAMERA_NAME = "Table_Camera"
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
CAMERA_HEIGHT_MARGIN = 1.0   # metri sopra il punto più alto della scena
COVER_ROUGHNESS = 0.9        # opacizza le copertine per evitare riflessi speculari
FRAME_MARGIN = 1.05          # 5% di margine attorno a tavolo+libri
RESOLUTION_LONG_EDGE = 2000  # px sul lato lungo dell'immagine
# Due problemi distinti, due fix distinti (vedi Bugs.md per la cronologia
# completa dei tentativi):
# 1) Con luci verticali (rotation_euler=(0,0,0)) sopra una camera
#    ortografica anch'essa verticale, la direzione riflessa speculare su
#    una superficie orizzontale è (0,0,1) per QUALUNQUE punto del tavolo:
#    ogni luce soddisfa la condizione di specchio sull'intera inquadratura
#    insieme, non in un punto isolato come con una superficie inclinata o
#    una prospettiva - un surplus di luminosità in più rispetto alla sola
#    incidenza diffusa. Fix: angolarle come in setup_render_camera.py
#    (TABLE_LIGHT_TILT_* sotto), che rompe l'allineamento.
# 2) Angolare le luci NON basta per gli adesivi ISBN: sono quasi bianco
#    puro (albedo carta ~0.85-0.9) contro l'albedo ~0.05-0.15 di una
#    copertina scura come "Emma". Con superfici perfettamente diffuse
#    (Lambertiane) il rapporto di luminosità riflessa tra due materiali è
#    il rapporto dei loro albedo, punto - NON dipende da energia o angolo
#    della luce, perché entrambi i materiali ricevono la stessa
#    irradianza nello stesso punto. Nessuna energia "giusta" può quindi
#    esporre bene sia la copertina scura sia l'adesivo bianco con una
#    view transform che clippa. Fix: view_transform 'AgX' in
#    setup_render_settings(), che comprime le alte luci invece di
#    clipparle.
# Round 5 (2026-08-02): il round 4 aveva provato a scurire con
# un'esposizione post-AgX (-1 stop) e uno world più scuro, lasciando
# invariata l'energia — risultato verificato con un render reale: il
# tavolo (legno, albedo medio-basso, illuminato quasi solo dall'ambiente)
# è stato spinto nella parte "toe" della curva AgX, che desatura le zone
# scure verso il grigio: si è fuso visivamente con lo sfondo world (reso
# anch'esso più scuro) e la venatura del legno è sparita. Le copertine dei
# libri, già nella parte alta/piatta della curva AgX (shoulder, che
# comprime le alte luci), sono rimaste quasi identiche: un taglio uniforme
# post-tonemap sposta poco le zone già chiare e molto le zone scure -
# esattamente l'effetto opposto a quello voluto. Esposizione post-AgX e
# taglio dello world tornati ai valori pre-round4 (vedi
# setup_world_background sotto); l'unica leva che sposta davvero le zone
# chiare (i libri, illuminati direttamente dalle 3 area light) lungo la
# curva PRIMA che AgX le comprima è l'energia della luce diretta stessa.
# Sceso a 10.0, verificato con un render reale: le copertine sono scese a
# un livello ragionevole (media ~77-98/255, non più clippate né troppo
# chiare), ma il tavolo di legno (albedo più basso delle copertine, quindi
# sempre più scuro a parità di luce) è diventato quasi invisibile (media
# ~52/255 nei vani tra i libri, più scuro perfino del margine di sfondo).
# Legno e copertine condividono le stesse 3 luci: non si possono esporre
# in modo indipendente solo con l'energia. Round 6: valore intermedio, per
# tenere il legno visibile senza tornare alla sovraesposizione dei libri.
TABLE_LIGHT_ENERGY = 17.0
# Stessi angoli di setup_render_camera.py (45° al centro, 60° ai lati):
# principio identico ("non puntare le luci lungo l'asse della camera"),
# qui necessario esplicitamente per rompere l'allineamento verticale del
# punto 1 sopra.
TABLE_LIGHT_TILT_CENTER = 45.0
TABLE_LIGHT_TILT_SIDE = 60.0

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


def output_stem():
    """Nome base dei file di output, derivato dal .blend aperto (es.
    'table_scene_retro' o 'table_scene_cover'): così lo stesso script,
    eseguito su scene diverse, non sovrascrive l'output dell'altra."""
    return os.path.splitext(os.path.basename(bpy.data.filepath))[0]


def get_scene_objects():
    """Tutta la scena "libri sul tavolo" è tavolo + libri, nessun altro
    oggetto: a differenza dello scaffale non serve filtrare per prefisso."""
    objs = [obj for obj in bpy.data.objects if obj.type == "MESH"]
    if not objs:
        raise RuntimeError("Nessuna mesh trovata nella scena: esegui prima "
                            "create_table_scene_retro.py o create_table_scene_cover.py.")
    return objs


def frame_bounds(objects):
    """Bounding box mondo unione di tutti gli oggetti passati."""
    coords = []
    for obj in objects:
        coords += [obj.matrix_world @ v.co for v in obj.data.vertices]

    xs = [c.x for c in coords]
    ys = [c.y for c in coords]
    zs = [c.z for c in coords]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def setup_camera(scene_objects):
    (xmin, ymin, zmin), (xmax, ymax, zmax) = frame_bounds(scene_objects)
    width = xmax - xmin    # X, inquadratura orizzontale
    depth = ymax - ymin    # Y, inquadratura verticale nell'immagine
    center_x = (xmin + xmax) / 2.0
    center_y = (ymin + ymax) / 2.0
    top_z = zmax + CAMERA_HEIGHT_MARGIN

    print(
        f"[setup_table_camera] bounding box scena: "
        f"width(X)={width:.3f}m depth(Y)={depth:.3f}m "
        f"center=({center_x:.3f}, {center_y:.3f})"
    )

    # Stessa tecnica di setup_render_camera.py: risoluzione proporzionata
    # all'aspect ratio reale (qui X/Y del tavolo) per evitare crop tra
    # ortho_scale e resolution_x/y.
    if depth >= width:
        res_y = RESOLUTION_LONG_EDGE
        res_x = max(1, round(RESOLUTION_LONG_EDGE * width / depth))
    else:
        res_x = RESOLUTION_LONG_EDGE
        res_y = max(1, round(RESOLUTION_LONG_EDGE * depth / width))
    bpy.context.scene.render.resolution_x = res_x
    bpy.context.scene.render.resolution_y = res_y

    cam_data = bpy.data.cameras.get(CAMERA_NAME) or bpy.data.cameras.new(CAMERA_NAME)
    cam_data.type = 'ORTHO'
    if depth >= width:
        cam_data.sensor_fit = 'VERTICAL'
        cam_data.ortho_scale = depth * FRAME_MARGIN
    else:
        cam_data.sensor_fit = 'HORIZONTAL'
        cam_data.ortho_scale = width * FRAME_MARGIN

    cam_obj = bpy.data.objects.get(CAMERA_NAME)
    if cam_obj is None:
        cam_obj = bpy.data.objects.new(CAMERA_NAME, cam_data)
        bpy.context.collection.objects.link(cam_obj)

    cam_obj.location = (center_x, center_y, top_z)
    # rotazione identità: la camera guarda già lungo il proprio -Z locale,
    # che con rotazione zero coincide col -Z del mondo (dritta verso il
    # basso) - a differenza della libreria (vista frontale) non serve
    # ruotare Rx/Rz per puntarla.
    cam_obj.rotation_euler = (0.0, 0.0, 0.0)
    bpy.context.scene.camera = cam_obj

    cam_local_pose = cam_obj.matrix_world.copy()
    return cam_obj, cam_local_pose


def setup_lighting():
    """Tre luci ad area sopra il tavolo, angolate come in
    setup_render_camera.py (45° al centro, 60° ai lati) invece che
    verticali: con luci verticali sopra una camera ortografica anch'essa
    verticale, la direzione riflessa speculare su una superficie
    orizzontale è (0,0,1) ovunque sul tavolo, quindi ogni luce soddisfa la
    condizione di specchio sull'intera inquadratura simultaneamente (non
    in un punto isolato) - contribuiva alla sovraesposizione oltre alla
    sola incidenza diffusa (vedi Bugs.md). L'angolo rompe questo
    allineamento, come già fa la libreria con le sue luci frontali - da
    solo però non basta per il contrasto copertina scura/adesivo ISBN
    chiaro, vedi TABLE_LIGHT_ENERGY e view_transform in
    setup_render_settings()."""
    for name, loc, tilt_deg in [
        ("Table_Light_Center", (0.0, 0.0, 1.8), TABLE_LIGHT_TILT_CENTER),
        ("Table_Light_Left",   (-0.5, 0.3, 1.5), TABLE_LIGHT_TILT_SIDE),
        ("Table_Light_Right",  (0.5, -0.3, 1.5), TABLE_LIGHT_TILT_SIDE),
    ]:
        light_data = bpy.data.lights.get(name) or bpy.data.lights.new(name, type='AREA')
        light_data.energy = TABLE_LIGHT_ENERGY
        light_data.size = 1.5

        light_obj = bpy.data.objects.get(name)
        if light_obj is None:
            light_obj = bpy.data.objects.new(name, light_data)
            bpy.context.collection.objects.link(light_obj)
        light_obj.location = loc
        light_obj.rotation_euler = (math.radians(tilt_deg), 0.0, 0.0)


def setup_world_background():
    """Il world non è solo lo sfondo visibile attorno al tavolo: in Cycles
    è anche una sorgente di luce ambientale che illumina l'intera scena da
    ogni direzione, sommandosi alle 3 area light. Il round 4 lo aveva
    scurito insieme a un'esposizione post-AgX per abbassare la luminosità
    complessiva, ma verificato con un render reale l'effetto è stato
    quello di desaturare il legno del tavolo (poco illuminato, spinto nel
    "toe" della curva AgX) fino a fondersi col grigio di sfondo, mentre le
    copertine dei libri (già nella parte alta/piatta della curva) non ne
    hanno risentito. Tornato ai valori originali: il tavolo deve restare
    ben distinguibile dallo sfondo, la luminosità dei libri si corregge
    con l'energia della luce diretta (TABLE_LIGHT_ENERGY sopra), non
    tagliando l'ambiente."""
    world = bpy.context.scene.world or bpy.data.worlds.new("Table_World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = (0.85, 0.85, 0.85, 1.0)
        bg.inputs[1].default_value = 1.0


def reduce_cover_gloss():
    """Alza la Roughness su copertine/retro/coste (materiali *_cover/_retro/
    _spine creati da create_books.py) per evitare riflessi che nascondono
    testo - stessa funzione di setup_render_camera.py, riapplicata qui
    perché ogni scena ha le proprie istanze di materiale."""
    for mat in bpy.data.materials:
        if mat.name.endswith(("_cover", "_retro", "_spine")) and mat.use_nodes:
            bsdf = mat.node_tree.nodes.get("Principled BSDF")
            if bsdf:
                bsdf.inputs["Roughness"].default_value = COVER_ROUGHNESS


def setup_render_settings():
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 128
    scene.cycles.use_denoising = True
    scene.render.image_settings.file_format = 'PNG'
    # A differenza di setup_render_camera.py: 'Standard' clippa a bianco
    # puro, e il rapporto di albedo tra copertina scura (~0.05-0.15) e
    # adesivo ISBN quasi bianco (~0.85-0.9) è troppo ampio perché
    # un'unica esposizione lineare li esponga bene entrambi senza
    # clippare l'adesivo (vedi TABLE_LIGHT_ENERGY sopra). 'AgX' comprime
    # le alte luci invece di clipparle, mantenendo leggibile il barcode
    # senza schiacciare il testo sulla copertina scura.
    scene.view_settings.view_transform = 'AgX'


def main():
    scene_objects = get_scene_objects()
    cam_obj, cam_pose = setup_camera(scene_objects)
    setup_lighting()
    setup_world_background()
    reduce_cover_gloss()
    setup_render_settings()

    stem = output_stem()
    out_png = os.path.join(OUTPUT_DIR, f"{stem}_photo.png")
    bpy.context.scene.render.filepath = out_png
    bpy.ops.render.render(write_still=True)

    loc = cam_pose.to_translation()
    quat = cam_pose.to_quaternion()
    pose_json = {
        "camera_world": {
            "x": loc.x, "y": loc.y, "z": loc.z,
            "qx": quat.x, "qy": quat.y, "qz": quat.z, "qw": quat.w,
        },
        "ortho_scale": cam_obj.data.ortho_scale,
        "resolution_x": bpy.context.scene.render.resolution_x,
        "resolution_y": bpy.context.scene.render.resolution_y,
    }
    pose_path = os.path.join(OUTPUT_DIR, f"{stem}_camera_pose.json")
    with open(pose_path, "w", encoding="utf-8") as f:
        json.dump(pose_json, f, indent=2)

    print(f"Render salvato in: {out_png}")
    print(f"Posa camera salvata in: {pose_path}")


if __name__ == "__main__":
    main()
