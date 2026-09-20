"""
Rileva libri e oggetti generici in un'immagine di libreria usando YOLOv8.
Output: lista di DetectedObject con bbox, classe, confidence.
"""

from __future__ import annotations
import cv2
import numpy as np
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
import logging

log = logging.getLogger(__name__)

# Classi COCO che consideriamo "ostacoli" (non libri)
OBSTACLE_CLASSES = {
    "bottle", "cup", "vase", "clock", "potted plant", "bowl",
    "remote", "cell phone", "toy", "figurine", "scissors",
    "teddy bear", "mouse", "keyboard", "laptop"
}


@dataclass
class DetectedObject:
    obj_id: int
    class_name: str          # "book" | "bottle" | ...
    is_book: bool
    bbox: tuple              # (x1, y1, x2, y2) in pixel
    confidence: float
    center: tuple            # (cx, cy)
    width_px: int
    height_px: int
    # Campi riempiti da moduli successivi
    color_name: str = ""
    color_rgb: tuple = field(default_factory=tuple)
    ocr_text: str = ""
    title: str = ""
    author: str = ""
    orientation: str = "unknown"   # upright | sideways_left | sideways_right | inverted
    depth_m: float = 0.0           # distanza stimata in metri
    shelf_row: int = -1
    shelf_slot: int = -1
    world_xyz: tuple = field(default_factory=tuple)
    isbn: str = ""                 # dal codice a barre sul retro (tavolo), 2026-09-06
    year: str = ""                 # anno di prima pubblicazione (metadati ISBN)
    # Geometria 3D dalla depth della shelf_camera (vision/shelf_geometry.py,
    # 2026-09-13): cio' che serve a pick_test_book per una presa automatica
    # senza pose/misure note a priori. 0 = non misurato.
    world_x: float = 0.0           # x mondo della faccia frontale (dorso verso il robot)
    world_y: float = 0.0           # y mondo del centro del dorso
    z_bottom: float = 0.0
    z_top: float = 0.0
    thickness_m: float = 0.0       # spessore (laterale)
    height_m: float = 0.0
    length_m: float = 0.0          # profondita' dorso->taglio (0 = non visibile)
    free_plus_m: float = 0.0       # spazio libero verso +y mondo (vicino o parete)
    free_minus_m: float = 0.0      # spazio libero verso -y mondo
    width_profile: list = field(default_factory=list)   # [(z, larghezza)] per fasce di 1 cm (2026-09-18)

    @property
    def is_obstacle(self) -> bool:
        return not self.is_book

    def to_dict(self) -> dict:
        return {
            "id": self.obj_id,
            "class": self.class_name,
            "is_book": self.is_book,
            "bbox": list(self.bbox),
            "confidence": round(self.confidence, 3),
            "color": self.color_name,
            "title": self.title,
            "author": self.author,
            "orientation": self.orientation,
            "depth_m": round(self.depth_m, 3),
            "shelf_row": self.shelf_row,
            "shelf_slot": self.shelf_slot,
            "isbn": self.isbn,
            "year": self.year,
            "world_x": round(self.world_x, 4),
            "world_y": round(self.world_y, 4),
            "z_bottom": round(self.z_bottom, 4),
            "z_top": round(self.z_top, 4),
            "thickness_m": round(self.thickness_m, 4),
            "height_m": round(self.height_m, 4),
            "length_m": round(self.length_m, 4),
            "free_plus_m": round(self.free_plus_m, 4),
            "free_minus_m": round(self.free_minus_m, 4),
            "width_profile": [[round(float(z), 4), round(float(w), 4)] for z, w in (self.width_profile or [])],
            "ocr_text": self.ocr_text or "",
        }


class BookDetector:
    """
    Wrapper YOLOv8 per rilevamento libri e ostacoli su scaffale.

    Uso:
        detector = BookDetector()
        objects = detector.detect(image_bgr)
    """

    def __init__(self, model_path: str = "yolov8n.pt", conf_threshold: float = 0.35):
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            log.info(f"YOLOv8 caricato: {model_path}")
        except ImportError:
            raise ImportError("Installa ultralytics: pip install ultralytics")

        self.conf_threshold = conf_threshold
        self._next_id = 0

    def detect(self, image_bgr: np.ndarray) -> list[DetectedObject]:
        """
        Lancia la detection sull'immagine e restituisce la lista di oggetti.
        """
        results = self.model(image_bgr, conf=self.conf_threshold, verbose=False)
        detected = []

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id].lower()

                is_book = (cls_name == "book")
                obj = DetectedObject(
                    obj_id=self._next_id,
                    class_name=cls_name,
                    is_book=is_book,
                    bbox=(x1, y1, x2, y2),
                    confidence=conf,
                    center=((x1 + x2) // 2, (y1 + y2) // 2),
                    width_px=x2 - x1,
                    height_px=y2 - y1,
                )
                detected.append(obj)
                self._next_id += 1

        log.info(f"Rilevati: {sum(o.is_book for o in detected)} libri, "
                 f"{sum(o.is_obstacle for o in detected)} ostacoli")
        return detected

    def draw_detections(self, image_bgr: np.ndarray,
                        objects: list[DetectedObject]) -> np.ndarray:
        """Disegna i bounding box sull'immagine per debug/visualizzazione."""
        out = image_bgr.copy()
        for obj in objects:
            x1, y1, x2, y2 = obj.bbox
            color = (0, 200, 80) if obj.is_book else (0, 80, 220)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

            label = f"[{obj.obj_id}] {obj.class_name} {obj.confidence:.2f}"
            if obj.title:
                label += f" | {obj.title[:20]}"
            if obj.color_name:
                label += f" | {obj.color_name}"

            cv2.putText(out, label, (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        return out
