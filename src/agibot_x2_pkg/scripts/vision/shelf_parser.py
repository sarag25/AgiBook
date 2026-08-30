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
    x_left: int = 0
    x_right: int = 0
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
      5. Riempie row.slots con TUTTI gli slot (occupati + vuoti): i vuoti
         sono gap fra libri consecutivi (o fra un bordo riga e il primo/
         ultimo libro) più larghi di un libro medio - vedi _build_slots.

    Nota: gli slot vuoti sono utili per decidere IN QUALE RIGA c'è posto
    (mappa direttamente sulle 3 configurazioni braccio esistenti reach_
    shelf_low/mid/high in action_sequencer.py); la posizione X del gap
    non guida il braccio verso un punto preciso - questo progetto non ha
    ancora una IK continua, solo joint config fissi per riga.
    """

    # Fallback quando una riga non ha nessun libro rilevato da cui stimare
    # una larghezza media (riga vuota o rilevazione fallita): valore di
    # comodo mai calibrato su una vera distanza focale/profondità - se le
    # righe hanno quasi sempre almeno un libro, questo valore conta poco.
    DEFAULT_BOOK_WIDTH_PX = 45

    def __init__(self, min_row_gap_px: int = 40, min_gap_factor: float = 0.8):
        self.min_row_gap_px = min_row_gap_px
        # Un gap fra due libri (o fra bordo riga e libro) deve essere almeno
        # min_gap_factor × larghezza media di un libro in quella riga per
        # contare come slot vuoto - evita di scambiare la normale spaziatura
        # fra dorsi per un buco libero.
        self.min_gap_factor = min_gap_factor

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
        for row in rows:
            self._build_slots(row, image_bgr.shape[1])
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

        # Tieni solo linee quasi orizzontali (|dy| < 10px).
        # reshape(-1, 4): OpenCV <5 restituisce shape (N,1,4), OpenCV 5
        # (nel venv via opencv-python-headless, dipendenza di easyocr)
        # restituisce (N,4) - senza normalizzare, con OpenCV 5 "line[0]"
        # e' uno scalare e l'unpacking esplode (TypeError: cannot unpack
        # non-iterable numpy.int32, visto al primo run reale 2026-08-29).
        h_lines = []
        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
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

    def _build_slots(self, row: ShelfRow, img_width: int):
        """
        Popola row.slots con TUTTI gli slot della riga, occupati e vuoti.

        row.objects è già ordinato per x (fatto in _assign_objects_to_rows).
        Cammina lungo l'asse X aggiungendo uno ShelfSlot occupato per ogni
        libro rilevato, e uno slot vuoto per ogni gap (prima del primo
        libro, fra due libri, dopo l'ultimo) abbastanza largo da contenere
        un libro medio - un gap enorme diventa più slot vuoti consecutivi,
        non uno solo, così find_empty_slots() ne conta il numero reale.
        """
        row.slots = []

        widths = [o.width_px for o in row.objects if o.width_px > 0]
        ref_width = int(np.median(widths)) if widths else self.DEFAULT_BOOK_WIDTH_PX
        min_gap = max(1, int(ref_width * self.min_gap_factor))

        def emit_empty_run(x_left: int, x_right: int):
            span = x_right - x_left
            if span < min_gap:
                return
            n = max(1, round(span / ref_width))
            step = span / n
            for i in range(n):
                sl = x_left + int(i * step)
                sr = x_left + int((i + 1) * step)
                row.slots.append(ShelfSlot(
                    slot_id=len(row.slots), row_id=row.row_id,
                    x_center=(sl + sr) // 2, x_left=sl, x_right=sr,
                    occupied=False,
                ))

        cursor = 0
        for obj in row.objects:
            x1, _, x2, _ = obj.bbox
            emit_empty_run(cursor, x1)
            row.slots.append(ShelfSlot(
                slot_id=len(row.slots), row_id=row.row_id,
                x_center=(x1 + x2) // 2, x_left=x1, x_right=x2,
                occupied=True, obj_id=obj.obj_id,
            ))
            cursor = x2
        emit_empty_run(cursor, img_width)

    def find_empty_slots(self, rows: list[ShelfRow]) -> list[ShelfSlot]:
        """Tutti gli slot vuoti su tutte le righe, ordinati per riga poi x."""
        return [s for r in rows for s in r.slots if not s.occupied]

    def occupancy_summary(self, rows: list[ShelfRow]) -> dict:
        """
        Riassunto "quanto è piena/vuota" la libreria, per riga e totale.
        Non giudica se è "in ordine" (dipende dal criterio di
        ordinamento scelto dall'utente, vedi sort_planner.SortPlanner) -
        solo occupazione geometrica.
        """
        per_row = {}
        total_slots = total_occupied = 0
        for row in rows:
            occupied = sum(1 for s in row.slots if s.occupied)
            total = len(row.slots)
            per_row[row.row_id] = {
                "occupied": occupied, "total": total,
                "empty": total - occupied,
            }
            total_slots += total
            total_occupied += occupied
        return {
            "per_row": per_row,
            "total": total_slots,
            "occupied": total_occupied,
            "empty": total_slots - total_occupied,
            "occupancy_ratio": (total_occupied / total_slots) if total_slots else 0.0,
        }

    def visualize(self, image_bgr: np.ndarray,
                  rows: list[ShelfRow]) -> np.ndarray:
        """Disegna le righe rilevate + gli slot vuoti sull'immagine per debug."""
        out = image_bgr.copy()
        for row in rows:
            cv2.line(out, (0, row.y_top), (out.shape[1], row.y_top),
                     (255, 200, 0), 1, cv2.LINE_AA)
            cv2.putText(out, f"R{row.row_id}", (4, row.y_center),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
            for slot in row.slots:
                if slot.occupied:
                    continue
                cv2.rectangle(out, (slot.x_left, row.y_top),
                              (slot.x_right, row.y_bottom), (0, 255, 0), 1)
                cv2.putText(out, "vuoto", (slot.x_left + 2, row.y_center),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        return out
