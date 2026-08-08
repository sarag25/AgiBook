"""
Analizza la struttura della libreria: rileva ripiani, slot, dimensioni.
Assegna ogni oggetto rilevato al ripiano e slot corrispondenti.
"""

from __future__ import annotations
import cv2
import numpy as np
import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from scripts.vision.book_detector import DetectedObject

log = logging.getLogger(__name__)


@dataclass
class ShelfRow:
    row_id: int
    y_top: int
    y_bottom: int
    y_center: int
    slots: list = field(default_factory=list)   # lista slot x ordinati
    objects: list = field(default_factory=list)  # DetectedObject assegnati


@dataclass
class ShelfSlot:
    slot_id: int
    row_id: int
    x_center: int
    occupied: bool = False
    obj_id: int = -1


class ShelfParser:
    """
    Identifica i ripiani della libreria nell'immagine e assegna
    ogni libro/oggetto al ripiano e slot corretto.

    Strategia:
      1. Rileva le linee orizzontali forti (bordi ripiani) con HoughLines
      2. Raggruppa le linee vicine → ogni gruppo = un ripiano
      3. Ordina gli oggetti per (riga, x_center)
      4. Assegna shelf_row e shelf_slot a ogni DetectedObject
    """

    def __init__(self, min_row_gap_px: int = 40):
        self.min_row_gap_px = min_row_gap_px

    def parse(self, image_bgr: np.ndarray,
              objects: list["DetectedObject"]) -> list[ShelfRow]:
        """
        Analizza l'immagine, rileva i ripiani, assegna posizioni agli oggetti.
        Modifica in-place shelf_row e shelf_slot di ogni DetectedObject.
        """
        rows = self._detect_shelf_rows(image_bgr)
        if not rows:
            # Fallback: inferisci le righe dai centri Y degli oggetti
            rows = self._infer_rows_from_objects(objects, image_bgr.shape[0])

        self._assign_objects_to_rows(objects, rows)
        return rows

    def _detect_shelf_rows(self, image_bgr: np.ndarray) -> list[ShelfRow]:
        """Usa Canny + HoughLinesP per trovare i bordi orizzontali dei ripiani."""
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi/180,
            threshold=80, minLineLength=image_bgr.shape[1] // 3,
            maxLineGap=30
        )

        if lines is None:
            return []

        # Tieni solo linee quasi orizzontali (|dy| < 10px)
        h_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if abs(y2 - y1) < 10:
                h_lines.append((y1 + y2) // 2)

        if not h_lines:
            return []

        # Raggruppa linee vicine → un ripiano per cluster
        h_lines.sort()
        groups = []
        current = [h_lines[0]]
        for y in h_lines[1:]:
            if y - current[-1] < self.min_row_gap_px:
                current.append(y)
            else:
                groups.append(current)
                current = [y]
        groups.append(current)

        rows = []
        prev_y = 0
        for i, group in enumerate(groups):
            y_center = int(np.mean(group))
            row = ShelfRow(
                row_id=i,
                y_top=prev_y,
                y_bottom=y_center,
                y_center=(prev_y + y_center) // 2,
            )
            rows.append(row)
            prev_y = y_center

        # Ultima riga fino al fondo immagine
        if rows:
            last = rows[-1]
            rows.append(ShelfRow(
                row_id=len(rows),
                y_top=last.y_bottom,
                y_bottom=image_bgr.shape[0],
                y_center=(last.y_bottom + image_bgr.shape[0]) // 2,
            ))

        log.info(f"Ripiani rilevati: {len(rows)}")
        return rows

    def _infer_rows_from_objects(self, objects: list["DetectedObject"],
                                 img_height: int) -> list[ShelfRow]:
        """
        Fallback: raggruppa gli oggetti per y_center usando clustering.
        """
        if not objects:
            return [ShelfRow(0, 0, img_height, img_height // 2)]

        from sklearn.cluster import AgglomerativeClustering

        centers_y = np.array([[o.center[1]] for o in objects])
        n_clusters = max(1, min(5, len(objects) // 2))
        cl = AgglomerativeClustering(n_clusters=n_clusters)
        labels = cl.fit_predict(centers_y)

        row_ys = {}
        for obj, label in zip(objects, labels):
            row_ys.setdefault(label, []).append(obj.center[1])

        rows = []
        for i, (label, ys) in enumerate(sorted(row_ys.items(),
                                                key=lambda x: np.mean(x[1]))):
            y_center = int(np.mean(ys))
            rows.append(ShelfRow(
                row_id=i,
                y_top=max(0, y_center - 80),
                y_bottom=min(img_height, y_center + 80),
                y_center=y_center,
            ))
        return rows

    def _assign_objects_to_rows(self, objects: list["DetectedObject"],
                                rows: list[ShelfRow]):
        """Assegna shelf_row e shelf_slot a ogni oggetto."""
        for obj in objects:
            cy = obj.center[1]
            # Trova la riga con y_top/y_bottom che contiene cy
            best_row = min(rows, key=lambda r: abs(r.y_center - cy))
            obj.shelf_row = best_row.row_id
            best_row.objects.append(obj)

        # Ordina gli oggetti per x_center → assegna shelf_slot
        for row in rows:
            row.objects.sort(key=lambda o: o.center[0])
            for slot_idx, obj in enumerate(row.objects):
                obj.shelf_slot = slot_idx

        log.debug("Assegnazione righe/slot completata")

    def visualize(self, image_bgr: np.ndarray,
                  rows: list[ShelfRow]) -> np.ndarray:
        """Disegna le righe rilevate sull'immagine per debug."""
        out = image_bgr.copy()
        for row in rows:
            cv2.line(out, (0, row.y_top), (out.shape[1], row.y_top),
                     (255, 200, 0), 1, cv2.LINE_AA)
            cv2.putText(out, f"R{row.row_id}", (4, row.y_center),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
        return out
