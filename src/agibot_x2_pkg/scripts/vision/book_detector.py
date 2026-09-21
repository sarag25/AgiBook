"""
Helpers to detect books and generic objects in a bookshelf image with YOLOv8.
Output: list of DetectedObject with bbox, class and confidence.
"""

from __future__ import annotations
import cv2
import numpy as np
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
import logging

log = logging.getLogger(__name__)

# COCO classes treated as obstacles (not books)
OBSTACLE_CLASSES = {
    "bottle", "cup", "vase", "clock", "potted plant", "bowl",
    "remote", "cell phone", "toy", "figurine", "scissors",
    "teddy bear", "mouse", "keyboard", "laptop"
}


@dataclass
class DetectedObject:
    """
    One detected object; fields after height_px are filled by later modules
    """
    obj_id: int
    class_name: str          # "book" | "bottle" | ...
    is_book: bool
    bbox: tuple              # (x1, y1, x2, y2) in pixels
    confidence: float
    center: tuple            # (cx, cy)
    width_px: int
    height_px: int
    color_name: str = ""
    color_rgb: tuple = field(default_factory=tuple)
    ocr_text: str = ""
    title: str = ""
    author: str = ""
    orientation: str = "unknown"   # upright | sideways_left | sideways_right | inverted
    depth_m: float = 0.0           # estimated distance in m
    shelf_row: int = -1
    shelf_slot: int = -1
    world_xyz: tuple = field(default_factory=tuple)
    isbn: str = ""                 # from the barcode on the back cover (table)
    year: str = ""                 # first publication year (ISBN metadata)
    # 3D geometry from the shelf_camera depth (shelf_geometry.py), used by pick_test_book; 0 = not measured
    world_x: float = 0.0           # world x of the front face (spine towards the robot)
    world_y: float = 0.0           # world y of the spine center
    z_bottom: float = 0.0
    z_top: float = 0.0
    thickness_m: float = 0.0       # lateral thickness
    height_m: float = 0.0
    length_m: float = 0.0          # spine-to-fore-edge depth (0 = not visible)
    free_plus_m: float = 0.0       # free space towards world +y (neighbour or wall)
    free_minus_m: float = 0.0      # free space towards world -y
    width_profile: list = field(default_factory=list)   # [(z, width)] per 1 cm band

    @property
    def is_obstacle(self) -> bool:
        """
        True for anything that is not a book
        """
        return not self.is_book

    def to_dict(self) -> dict:
        """
        JSON-friendly dict of the object (lengths rounded to 0.1 mm)
        """
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
    YOLOv8 wrapper that detects books and obstacles on a shelf.
    Usage: objects = BookDetector().detect(image_bgr)
    """

    def __init__(self, model_path: str = "yolov8n.pt", conf_threshold: float = 0.35):
        """
        Load the YOLOv8 model
        """
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            log.info(f"YOLOv8 loaded: {model_path}")
        except ImportError:
            raise ImportError("Install ultralytics: pip install ultralytics")

        self.conf_threshold = conf_threshold
        self._next_id = 0

    def detect(self, image_bgr: np.ndarray) -> list[DetectedObject]:
        """
        Run detection on the image and return the list of objects
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

        log.info(f"Detected: {sum(o.is_book for o in detected)} books, "
                 f"{sum(o.is_obstacle for o in detected)} obstacles")
        return detected

    def draw_detections(self, image_bgr: np.ndarray,
                        objects: list[DetectedObject]) -> np.ndarray:
        """
        Draw the bounding boxes on a copy of the image for debugging
        """
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
