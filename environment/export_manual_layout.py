"""
Script Blender per esportare in JSON la posizione manuale di libri e
decorazioni nella libreria, da usare al posto del layout casuale di
BookPlacer quando si carica la scena in Gazebo (vedi
src/agibot_x2_pkg/agibot_x2_pkg/manual_scene_placer.py e il launch file
manual_scene.launch.py).

Serve a far corrispondere la scena spawnata in Gazebo a quella fotografata
in Blender da setup_render_camera.py: la foto ortografica non ha distorsione
ed è quella da usare per SAM/detection (non si può far scattare la foto al
robot in Gazebo perché le texture JPG delle copertine dei libri non
sopravvivono alla pipeline di rendering di Gazebo).

Uso: apri il .blend con libri e decorazioni già posizionati (dopo
create_scene.py, eventualmente con la fisica assestata premendo Play),
incolla/apri questo script nello Scripting tab di Blender ed esegui (Run
Script / Alt+P). Il JSON viene salvato accanto al file .blend, in
manual_layout.json.

Dal 2026-08-02 esporta anche la posa del tavolo di staging vuoto, se la
scena aperta ne contiene uno (environment/create_full_scene.py): una sola
entry con kind="table", object_key="table", posizione presa dall'oggetto
"table_top" (il gruppo piano+4 gambe è sempre rigido e traslato in blocco
da create_full_scene.place_table(), quindi la posizione del piano
rappresenta l'intero tavolo - stessa convenzione a singola posa già usata
per la libreria in bookshelf.urdf). Se lo script gira su library_scene.blend
(nessun tavolo nella scena) l'entry "table" semplicemente non compare,
nessun errore.

Assunzioni:
  - Lo script gira nella STESSA sessione Blender in cui gli oggetti sono
    ancora nel frame nativo Z-up di Blender (prima/senza il round-trip di
    export glTF, che introduce una correzione di assi separata usata altrove
    nella pipeline ROS).
  - Ogni libro/decorazione è un oggetto separato (non joinato con altri) il
    cui nome inizia con una delle chiavi in BOOK_KEYS/DECOR_KEYS (es.
    "it_book" oppure "it_book.002" se duplicato).
  - La libreria NON è un singolo oggetto ma più parti (shelf_back,
    shelf_board_0..N, shelf_side_left/right, ...) senza genitore comune,
    tutte a rotazione 0°. In questo caso il frame "locale libreria" è
    semplicemente il frame mondo di Blender, traslato di SHELF_ORIGIN
    (di default (0,0,0): se le coordinate esportate non coincidono con la
    convenzione di book_placer.py, es. SHELF_X_MIN/MAX centrato su 0,
    regola SHELF_ORIGIN qui sotto invece di spostare oggetti in Blender).
    Deve coincidere con lo SHELF_ORIGIN usato in setup_render_camera.py:
    nessun modulo condiviso tra Blender e ROS2 in questo repo, i due valori
    vanno tenuti allineati a mano per convenzione (come già per le quote
    dello scaffale, vedi Blender.md).
"""
import json
import os

import bpy

# ── configurazione: adatta ai nomi reali della tua scena ─────────────────
# Punto del frame mondo di Blender che deve corrispondere a (0,0,0) locale
# nella convenzione di book_placer.py. Lascia (0,0,0) se coincidono già.
SHELF_ORIGIN = (0.0, 0.0, 0.0)

OUTPUT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(bpy.data.filepath)), "manual_layout.json"
)

# Deve coincidere con le chiavi di BOOK_CATALOG in book_placer.py
BOOK_KEYS = [
    "alba_mietitura_book", "ballata_usignolo_book", "black_widow_book",
    "cane_stelle_book", "cane_stelle_racconti_book", "cats_cradle_book",
    "emma_book", "enciclopedia_animali_book", "enciclopedia_terra_vol1_book",
    "enciclopedia_terra_vol2_book", "fantasticos_4_book", "hunger_games_book",
    "it_book", "never_flinch_book", "werther_book",
]

# Deve coincidere con i nomi in DECORATIONS di create_desk_decorations.py
# e con DECOR_CATALOG in manual_scene_placer.py
DECOR_KEYS = [
    "pen_holder", "paperweight_ball", "desk_globe",
    "potted_plant", "coffee_mug", "coaster",
]

# Oggetto di riferimento per l'intero tavolo (piano+4 gambe, traslati in
# blocco da create_full_scene.place_table()): una sola entry, non una per
# parte, come per il singolo bookshelf.urdf.
TABLE_TOP_NAME = "table_top"

# Nome -> (object_key, kind). Per libri/decorazioni object_key coincide col
# nome della chiave stessa (usata poi per il lookup in BOOK_CATALOG/
# DECOR_CATALOG lato ROS2); per il tavolo object_key è "table" a prescindere
# dal nome dell'oggetto Blender, perché è quello che manual_scene_placer.py
# userà per trovare urdf/table.urdf, non un catalogo mesh/massa.
OBJECT_KEY_KIND = {
    **{k: (k, "book") for k in BOOK_KEYS},
    **{k: (k, "decoration") for k in DECOR_KEYS},
    TABLE_TOP_NAME: ("table", "table"),
}


def match_object_key(obj_name: str):
    """Ritorna (object_key, kind) per il primo prefisso che combacia, oppure (None, None)."""
    for match_name, (object_key, kind) in OBJECT_KEY_KIND.items():
        if obj_name == match_name or obj_name.startswith(match_name + "."):
            return object_key, kind
    return None, None


def main():
    ox, oy, oz = SHELF_ORIGIN
    entries = []

    for obj in bpy.data.objects:
        object_key, kind = match_object_key(obj.name)
        if object_key is None:
            continue

        # rotazione libreria = 0°, quindi il frame locale è il frame mondo
        # traslato di SHELF_ORIGIN (nessuna rotazione da compensare)
        loc = obj.matrix_world.translation
        yaw = obj.matrix_world.to_euler('XYZ').z

        entries.append({
            "object_key": object_key,
            "kind": kind,
            "source_object": obj.name,
            "local_x": round(loc.x - ox, 5),
            "local_y": round(loc.y - oy, 5),
            "local_z": round(loc.z - oz, 5),
            "local_yaw": round(yaw, 5),
        })

    if not entries:
        print("Nessun libro/decorazione trovato: controlla BOOK_KEYS/DECOR_KEYS e i nomi degli oggetti.")
        return

    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump(entries, f, indent=2, ensure_ascii=False)

    n_books = sum(1 for e in entries if e["kind"] == "book")
    n_decor = sum(1 for e in entries if e["kind"] == "decoration")
    n_table = sum(1 for e in entries if e["kind"] == "table")
    print(f"Esportati {len(entries)} oggetti ({n_books} libri, {n_decor} decorazioni, "
          f"{n_table} tavolo) in: {OUTPUT_PATH}")
    for e in entries:
        print(
            f"  {e['source_object']:<30} -> {e['object_key']:<25} [{e['kind']:<10}] "
            f"x={e['local_x']:.3f} y={e['local_y']:.3f} z={e['local_z']:.3f} "
            f"yaw={e['local_yaw']:.3f}"
        )


if __name__ == "__main__":
    main()
