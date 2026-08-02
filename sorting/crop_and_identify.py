"""
crop_and_identify.py
=====================
Prende le detection rilevate da detect_sam3.py su una foto — libri e altri
oggetti, vedi detect_sam3.CONCEPTS — ritaglia ciascuna e, solo per i libri,
ne estrae titolo/autore. Gli oggetti non-libro (decorazioni da scrivania,
ecc.) vengono ritagliati e basta: cercare titolo/autore su un portapenne
non avrebbe senso.

Due modalità, selezionate da `view` (di default "spine", pensata per la
foto frontale dello scaffale, spine-out, vedi Blender.md):

- **"spine"** (default): il ritaglio è la costa del libro. Se il testo non
  basta a identificarlo, identify_book (costa -> copertina -> retro)
  segnala che serve una foto successiva del libro ruotato — questo script
  si limita a riportare quel segnale (status "rotate"), non simula la
  rotazione: quella richiede una nuova foto della stessa area, salvata con
  lo stesso prefisso ("_cover"/"_retro") perché venga ripresa.
- **"cover"/"retro"**: il ritaglio mostra già la faccia indicata (es. la
  scena tavolo di setup_table_camera.py, dove tutti i libri sono già
  retro-up): non ha senso passare dallo stato costa/copertina di
  identify_book (non esiste una costa da leggere in quella foto), quindi
  per "retro" si chiama direttamente `search_from_retro` (barcode ISBN,
  poi OCR via extract_isbn.py) sul ritaglio.

Uso da detect_sam3.py:
    from crop_and_identify import identify_all
    records = identify_all(image_path, detections, view="retro")
    # detections: [{"kind": "book"|"object", "bbox": (x1,y1,x2,y2), ...}, ...]

Uso diretto (bbox in pixel, x1,y1,x2,y2, trattati come libri):
    python crop_and_identify.py library_photo.png "100,50,180,420" "200,60,270,410"
"""

import json
import sys
from pathlib import Path

from PIL import Image

from identify_book import identify_book, search_from_retro

CROPS_DIR = Path(__file__).parent / "crops"
RESULTS_PATH = Path(__file__).parent / "identified_books.json"

# Chiavi metadati presenti (vuote) su OGNI record, anche quando non
# identificato: così il JSON ha uno schema fisso, comodo da consumare a
# valle senza controllare se la chiave esiste.
_EMPTY_METADATA = {"title": "", "author": "", "publisher": "", "year": "", "language": "", "isbn13": ""}


def crop_book(image: Image.Image, box: tuple, book_id: int, view: str = "spine") -> Path:
    """
    Ritaglia il bbox (x1, y1, x2, y2) e lo salva come <id>_<view>.png: la
    faccia mostrata nel ritaglio dipende da quale foto è stata data in
    input (scaffale spine-out di default, oppure la scena tavolo
    cover/retro-up di setup_table_camera.py).
    """
    CROPS_DIR.mkdir(exist_ok=True)
    crop = image.crop(tuple(round(v) for v in box))
    out_path = CROPS_DIR / f"book_{book_id}_{view}.png"
    crop.save(out_path)
    return out_path


def crop_object(image: Image.Image, box: tuple, object_id: int) -> Path:
    """
    Ritaglia un oggetto non-libro e lo salva come <id>.png: nessuna
    convenzione _spine/_cover/_retro (non passa da identify_book, non c'è
    "faccia" da cercare per un oggetto qualunque).
    """
    CROPS_DIR.mkdir(exist_ok=True)
    crop = image.crop(tuple(round(v) for v in box))
    out_path = CROPS_DIR / f"object_{object_id}.png"
    crop.save(out_path)
    return out_path


def identify_all(image_path: str, detections: list, view: str = "spine") -> list:
    """
    Due fasi separate apposta:

    1. Ritaglia e salva SUBITO ogni detection (libro o oggetto), prima di
       tentare qualunque identificazione. Così i crop restano su disco anche
       se l'identificazione di un libro fallisce a metà lista (rete, API
       esterne down) — non si perdono i ritagli dei libri successivi.
    2. Identifica solo i libri. Un errore imprevisto sul singolo libro (es.
       eccezione non gestita più a monte) viene catturato e registrato come
       record "error": non interrompe il ciclo sugli altri libri.

       `view="spine"` (default, foto scaffale): costa -> copertina -> retro
       via identify_book, con eventuale segnale "rotate" se serve una foto
       successiva.
       `view="cover"/"retro"` (foto scena tavolo, già nella faccia giusta):
       nessuno stato costa/copertina da provare, si va dritti a
       `search_from_retro` (barcode ISBN, poi OCR) sul ritaglio.

    Salva tutti i record in RESULTS_PATH, così sort_books.py può riordinare
    senza rifare detection/OCR/ricerche web ogni volta.
    """
    image = Image.open(image_path).convert("RGB")

    crops = []
    for i, det in enumerate(detections):
        kind = det.get("kind", "book")
        box = det["bbox"]
        crop_path = crop_object(image, box, i) if kind == "object" else crop_book(image, box, i, view=view)
        crops.append({"id": i, "kind": kind, "box": box, "crop_path": crop_path})
    print(f"\n{len(crops)} ritagli salvati in {CROPS_DIR}")

    records = []
    for c in crops:
        i, kind, box, crop_path = c["id"], c["kind"], c["box"], c["crop_path"]

        if kind == "object":
            print(f"\n=== Oggetto {i} (bbox={tuple(round(v) for v in box)}) — non un libro, nessuna identificazione ===")
            records.append({
                "id": i, "kind": "object", "bbox": list(box), "crop": str(crop_path),
                "status": "object", "needs_rotation": False, **_EMPTY_METADATA,
            })
            continue

        print(f"\n=== Libro {i} (bbox={tuple(round(v) for v in box)}) ===")
        record = {"id": i, "kind": "book", "bbox": list(box), "crop": str(crop_path)}

        try:
            if view == "retro":
                result = search_from_retro(crop_path)
            else:
                prefix = crop_path.parent / crop_path.stem.replace(f"_{view}", "")
                result = identify_book(prefix)
        except Exception as exc:
            print(f"  Identificazione fallita per il libro {i}: {exc}")
            record.update({"status": "error", "needs_rotation": False, **_EMPTY_METADATA})
            records.append(record)
            continue

        if result["status"] == "found":
            info = result["info"]
            record.update({
                "status": "found",
                "source": result["source"],
                "needs_rotation": False,
                "title": info.get("Titolo", "N/A"),
                "author": info.get("Autori", "N/A"),
                "publisher": info.get("Editore", "N/A"),
                "year": info.get("Anno", "N/A"),
                "language": info.get("Lingua", "N/A"),
                "isbn13": info.get("ISBN-13", "N/A"),
            })
        elif result["status"] == "rotate":
            # needs_rotation=True è il segnale esplicito da controllare a
            # valle (es. dal planner del robot): "next" dice quale vista
            # serve ("cover"/"retro"), ma un semplice booleano evita di dover
            # confrontare status == "rotate" ovunque venga consumato il JSON.
            record.update({
                "status": "rotate",
                "next": result["next"],
                "needs_rotation": True,
                **_EMPTY_METADATA,
            })
        else:
            # "not_found": già provate costa, copertina e retro senza esito.
            # Nessuna rotazione ulteriore ha senso, il libro è stato visto
            # da tutti i lati previsti.
            record.update({"status": "not_found", "needs_rotation": False, **_EMPTY_METADATA})
        records.append(record)

    RESULTS_PATH.write_text(json.dumps(records, indent=2, ensure_ascii=False), encoding="utf-8")

    books = [r for r in records if r["kind"] == "book"]
    n_found = sum(1 for r in books if r["status"] == "found")
    n_objects = len(records) - len(books)
    print(f"\n{n_found}/{len(books)} libri identificati, {n_objects} oggetti (non libri) rilevati. "
          f"Risultati salvati in: {RESULTS_PATH}")
    return records


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python crop_and_identify.py <image_path> [x1,y1,x2,y2 ...]")
        sys.exit(1)

    img_path = sys.argv[1]
    if not Path(img_path).exists():
        print(f"Errore: immagine '{img_path}' non trovata.")
        sys.exit(1)

    if len(sys.argv) > 2:
        detections = [
            {"kind": "book", "bbox": tuple(float(v) for v in b.split(","))}
            for b in sys.argv[2:]
        ]
    else:
        print("Nessun bbox passato: uso l'intera immagine come singolo libro (solo per test manuale).")
        with Image.open(img_path) as im:
            detections = [{"kind": "book", "bbox": (0, 0, im.width, im.height)}]

    identify_all(img_path, detections)
