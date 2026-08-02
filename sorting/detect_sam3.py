"""
detect_sam3.py
==============
Rileva libri e altri oggetti su uno scaffale usando SAM3 (Meta), che fa
"Promptable Concept Segmentation": bastano i prompt testuali "book" e
"object" per segmentare ogni istanza, anche libri stretti e affiancati — a
differenza del rilevatore YOLO/COCO generico usato in test_yolo.py, che
tende a fondere libri vicini in un unico blob o a mancare i dorsi sottili.

Due passate separate sulla stessa immagine (stesso modello caricato una
sola volta): una col prompt "book", una col prompt "object" per le
decorazioni da scrivania e altro non-libro. I risultati vengono passati a
crop_and_identify.identify_all() già etichettati per tipo: solo i "book"
vengono avviati all'identificazione (costa/copertina/retro) — cercare
titolo/autore su un portapenne non avrebbe senso.

SAM3 è un modello ad accesso condizionato (gated) su Hugging Face:
  1. Crea un account su https://huggingface.com
  2. Richiedi l'accesso su https://huggingface.co/facebook/sam3
  3. Genera un token: https://huggingface.co/settings/tokens
  4. Aggiungi in .env:  HF_TOKEN=hf_xxx...

Dipendenze (pesanti, non installate di default nel progetto):
    pip install torch transformers

Uso:
    python books/detect_sam3.py books/library.png
    python books/detect_sam3.py books/library.png --threshold 0.4

Sulla foto scaffale (default, spine-out) i libri vanno identificati
costa -> copertina -> retro (vedi crop_and_identify.py). Sulla scena
tavolo (setup_table_camera.py), dove i libri sono già ripresi in una
faccia nota, passa `--view retro` (o `cover`) per saltare quello stato e
leggere l'ISBN direttamente dal ritaglio:
    python sorting/detect_sam3.py environment/render_output/table_scene_retro_photo.png --view retro
"""

import argparse
import os
import sys
from pathlib import Path

import torch
from transformers import Sam3Model, Sam3Processor

from dotenv import load_dotenv

sys.path.insert(0, str(Path(__file__).parent))
from crop_and_identify import identify_all

load_dotenv(Path(__file__).parent.parent / ".env")

# Concetti cercati da SAM3 in due passate separate sulla stessa immagine.
# "object" cattura le decorazioni da scrivania (e altro non-libro): vengono
# ritagliate ma non mandate a identify_book (vedi crop_and_identify.py).
CONCEPTS = ("book", "object")

# Un libro è comunque un "object": il prompt "object" lo ritrova quasi
# sempre di nuovo (bbox pressoché identico, vedi _dedup_objects). IoU minimo
# per considerare una detection "object" un duplicato di un libro già
# trovato e scartarla, invece di trattare un libro vero come non-libro.
DEDUP_IOU = 0.5


def _iou(box_a: tuple, box_b: tuple) -> float:
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter == 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _dedup_objects(books: list[dict], objects: list[dict]) -> list[dict]:
    """Scarta le detection 'object' che si sovrappongono a un libro già
    rilevato (stesso oggetto fisico, ritrovato due volte da due prompt
    diversi), tenendo solo gli oggetti non-libro distinti (decorazioni)."""
    kept = [obj for obj in objects
            if not any(_iou(obj["bbox"], b["bbox"]) >= DEDUP_IOU for b in books)]
    n_dropped = len(objects) - len(kept)
    if n_dropped:
        print(f"Scartate {n_dropped} detection 'object' che coincidono con libri già rilevati "
              f"(IoU >= {DEDUP_IOU}).")
    return kept


def _detect_concept(model, processor, image, text: str, threshold: float) -> list[dict]:
    inputs = processor(images=image, text=text, return_tensors="pt").to(model.device)

    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_instance_segmentation(
        outputs,
        threshold=threshold,
        mask_threshold=0.5,
        target_sizes=inputs.get("original_sizes").tolist(),
    )[0]

    boxes = results["boxes"].tolist()
    scores = results["scores"].tolist()

    print(f"Rilevati {len(boxes)} '{text}' (soglia={threshold}):")
    for box, score in zip(boxes, scores):
        print(f"  bbox={tuple(round(v) for v in box)}  conf={score:.2f}")

    return [{"bbox": tuple(box), "score": score} for box, score in zip(boxes, scores)]


def detect_all(image_path: str, threshold: float = 0.5) -> list[dict]:
    """
    Rileva libri e oggetti nell'immagine con una passata SAM3 per ciascun
    concetto in CONCEPTS (stesso modello, un solo caricamento). Le detection
    "object" che coincidono con un libro già trovato (un libro è comunque un
    "object" per SAM3) vengono scartate, vedi _dedup_objects. Ritorna una
    lista di detection: {"kind": "book"|"object", "bbox": (x1,y1,x2,y2), "score": float}
    """
    from PIL import Image

    hf_token = os.environ.get("HF_TOKEN") or None

    print("Carico SAM3 (facebook/sam3)...")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = Sam3Model.from_pretrained("facebook/sam3", token=hf_token).to(device)
    processor = Sam3Processor.from_pretrained("facebook/sam3", token=hf_token)

    image = Image.open(image_path).convert("RGB")

    by_concept = {
        concept: _detect_concept(model, processor, image, concept, threshold)
        for concept in CONCEPTS
    }

    books = [{"kind": "book", **det} for det in by_concept.get("book", [])]
    objects = [{"kind": "object", **det} for det in by_concept.get("object", [])]
    objects = _dedup_objects(books, objects)

    return books + objects


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rileva libri e oggetti su uno scaffale con SAM3")
    parser.add_argument("image", help="Percorso screenshot/foto dello scaffale")
    parser.add_argument("--threshold", type=float, default=0.5, help="Soglia di confidenza (default 0.5)")
    parser.add_argument("--view", choices=["spine", "cover", "retro"], default="spine",
                         help="Faccia mostrata dai libri nella foto: 'spine' per lo scaffale "
                              "(default, costa->copertina->retro), 'cover'/'retro' per la scena "
                              "tavolo (già nella faccia giusta, ISBN letto direttamente)")
    args = parser.parse_args()

    if not Path(args.image).exists():
        print(f"Errore: immagine '{args.image}' non trovata.")
        sys.exit(1)

    detections = detect_all(args.image, args.threshold)
    if not detections:
        print("Nessun libro/oggetto rilevato. Prova ad abbassare --threshold.")
        sys.exit(0)

    identify_all(args.image, detections, view=args.view)
