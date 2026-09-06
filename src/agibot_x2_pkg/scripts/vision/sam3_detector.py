"""
Detector alternativo a YOLO basato su SAM3 (Meta, Promptable Concept
Segmentation) - stessa interfaccia di BookDetector, selezionabile in
library_manager_node con il parametro ROS `detector:=sam3`.

Replica la logica di sorting/detect_sam3.py (la pipeline standalone
no-ROS del repo, vedi Sorting.md), adattata all'interfaccia della
pipeline ROS: due passate sulla stessa immagine (prompt "book" e
"object"), dedup delle detection "object" che coincidono con un libro
gia' trovato (per SAM3 un libro E' anche un object), output come lista
di DetectedObject.

Perche' esiste: YOLO/COCO (yolov8n) tende a fondere libri affiancati o a
mancare i dorsi sottili - al primo run reale (2026-08-29) ha trovato 4
libri su ~15 nella foto scaffale. SAM3 con prompt testuale e' molto piu'
adatto a questo soggetto (gia' verificato concettualmente dalla pipeline
standalone).

Costi/prerequisiti (gli stessi di sorting/detect_sam3.py):
  - modello gated su Hugging Face: serve HF_TOKEN (in .env alla radice
    del repo o come variabile d'ambiente) e accesso concesso su
    https://huggingface.co/facebook/sam3
  - primo avvio: scarica il modello (grande, richiede rete e spazio)
  - inferenza su CPU in questo container: lenta (decine di secondi per
    passata) - accettabile perche' gira una volta per foto trigger, non
    in continuo.
"""

from __future__ import annotations
import os
import logging

import numpy as np

from vision.book_detector import BookDetector, DetectedObject

log = logging.getLogger(__name__)

# Stessi concetti e soglia IoU di sorting/detect_sam3.py - tenere allineati.
CONCEPTS = ("book", "object")
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


class Sam3BookDetector:
    """
    Drop-in replacement di BookDetector basato su SAM3.

    Uso (identico a BookDetector):
        detector = Sam3BookDetector()
        objects = detector.detect(image_bgr)   # -> list[DetectedObject]
    """

    # Riusa il disegno debug di BookDetector (non tocca stato dell'istanza)
    draw_detections = BookDetector.draw_detections

    def __init__(self, conf_threshold: float = 0.5):
        try:
            import torch
            from transformers import Sam3Model, Sam3Processor
        except ImportError as e:
            raise ImportError(
                f"SAM3 richiede torch+transformers (gia' in requirements.txt): {e}"
            )

        hf_token = self._find_hf_token()
        if not hf_token:
            log.warning(
                "HF_TOKEN non trovato (ne' in ambiente ne' in .env): il "
                "download di facebook/sam3 fallira' se l'accesso al repo "
                "gated non e' gia' in cache."
            )

        self.conf_threshold = conf_threshold
        self._torch = torch
        device = "cuda" if torch.cuda.is_available() else "cpu"
        log.info(f"Carico SAM3 (facebook/sam3) su {device}...")
        self.model = Sam3Model.from_pretrained(
            "facebook/sam3", token=hf_token).to(device)
        self.processor = Sam3Processor.from_pretrained(
            "facebook/sam3", token=hf_token)
        self._next_id = 0
        log.info("SAM3 caricato")

    @staticmethod
    def _find_hf_token() -> str | None:
        """HF_TOKEN dall'ambiente, o da un .env cercato da cwd verso l'alto
        (stessa fonte di sorting/detect_sam3.py, che pero' lo cerca con un
        path relativo al proprio file - qui il file installato sta in
        install/, il path relativo non funzionerebbe)."""
        token = os.environ.get("HF_TOKEN")
        if token:
            return token
        try:
            from dotenv import load_dotenv, find_dotenv
            load_dotenv(find_dotenv(usecwd=True))
            return os.environ.get("HF_TOKEN")
        except ImportError:
            return None

    def _detect_concept(self, image_pil, text: str) -> list[dict]:
        import time
        t0 = time.perf_counter()
        inputs = self.processor(
            images=image_pil, text=text, return_tensors="pt"
        ).to(self.model.device)

        with self._torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_instance_segmentation(
            outputs,
            threshold=self.conf_threshold,
            mask_threshold=0.5,
            target_sizes=inputs.get("original_sizes").tolist(),
        )[0]

        boxes = results["boxes"].tolist()
        scores = results["scores"].tolist()
        log.info(f"SAM3: {len(boxes)} '{text}' (soglia={self.conf_threshold}) "
                 f"in {time.perf_counter() - t0:.0f} s")
        return [{"bbox": tuple(b), "score": s} for b, s in zip(boxes, scores)]

    def detect(self, image_bgr: np.ndarray) -> list[DetectedObject]:
        from PIL import Image
        rgb = image_bgr[:, :, ::-1]
        image_pil = Image.fromarray(np.ascontiguousarray(rgb))

        by_concept = {
            c: self._detect_concept(image_pil, c) for c in CONCEPTS
        }
        books = by_concept.get("book", [])
        # Dedup: un "object" che coincide con un libro e' il libro stesso
        objects = [
            det for det in by_concept.get("object", [])
            if all(_iou(det["bbox"], b["bbox"]) < DEDUP_IOU for b in books)
        ]

        detected = []
        for kind, dets in (("book", books), ("object", objects)):
            for det in dets:
                x1, y1, x2, y2 = (int(round(v)) for v in det["bbox"])
                detected.append(DetectedObject(
                    obj_id=self._next_id,
                    class_name=kind,
                    is_book=(kind == "book"),
                    bbox=(x1, y1, x2, y2),
                    confidence=float(det["score"]),
                    center=((x1 + x2) // 2, (y1 + y2) // 2),
                    width_px=x2 - x1,
                    height_px=y2 - y1,
                ))
                self._next_id += 1

        log.info(f"Rilevati: {sum(o.is_book for o in detected)} libri, "
                 f"{sum(o.is_obstacle for o in detected)} oggetti")
        return detected
