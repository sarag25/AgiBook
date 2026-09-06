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

    Strategia (rivista 2026-09-06):
      1. I ripiani si ricavano dagli OGGETTI: ogni cosa rilevata poggia su
         un piano, quindi il FONDO delle bbox (y2) si addensa a un'altezza
         per ripiano. Gruppi di fondi separati da meno di row_gap px = un
         ripiano. Prima si usavano le linee di Hough sull'immagine intera:
         a 960x720 venature del legno e bordi dei libri davano 4 "ripiani"
         su una foto che ne inquadra uno (prima, a 320x240, funzionava per
         caso). Hough resta solo come fallback per immagini senza oggetti.
         Limite noto: un ripiano completamente vuoto non genera una riga.
      2. Ogni oggetto va al ripiano con la linea più vicina al suo fondo
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

    def __init__(self, min_row_gap_px: int = 40, min_gap_factor: float = 0.8,
                 row_gap_frac: float = 0.15):
        self.min_row_gap_px = min_row_gap_px
        # Distanza verticale minima fra i fondi di due oggetti per stare su
        # ripiani diversi: max(min_row_gap_px, row_gap_frac * altezza img).
        # 0.15 = 108 px a 720p: gli oggetti sullo stesso ripiano ma a
        # profondità diversa (decorazioni 13 cm dietro il fronte dei libri,
        # camera inclinata) differiscono di ~70 px, i ripiani di 250+.
        self.row_gap_frac = row_gap_frac
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
        h, w = image_bgr.shape[:2]
        rows = self._rows_from_objects(objects, h)
        source = "fondo bbox oggetti"
        if not rows:
            rows = self._detect_shelf_rows(image_bgr)
            source = "linee Hough (nessun oggetto)"
        if not rows:
            rows = [ShelfRow(0, 0, h, h // 2)]
            source = "riga unica di default"
        log.info(f"Ripiani rilevati: {len(rows)} ({source})")

        self._assign_objects_to_rows(objects, rows, h)
        for row in rows:
            self._build_slots(row, w)
        return rows

    def _rows_from_objects(self, objects: list["DetectedObject"],
                           img_height: int) -> list[ShelfRow]:
        """Un ripiano per ogni gruppo di oggetti col fondo bbox alla stessa
        altezza (vedi docstring classe). Linea del ripiano = mediana dei
        fondi; la riga si estende dal ripiano precedente (o dal bordo
        superiore dell'oggetto più alto) fino alla linea."""
        if not objects:
            return []
        # Solo i LIBRI definiscono i ripiani, se ce ne sono: la passata
        # "object" di SAM3 e' generica e puo' prendere la testa del robot in
        # basso nell'inquadratura, il tavolo o la parete - ognuno col fondo a
        # un'altezza diversa = un ripiano fantasma. Le decorazioni vengono
        # poi assegnate al ripiano piu' vicino (o scartate se lontane da
        # tutti, vedi _assign_objects_to_rows).
        anchors = [o for o in objects if o.is_book] or list(objects)
        row_gap = max(self.min_row_gap_px, int(self.row_gap_frac * img_height))
        ordered = sorted(anchors, key=lambda o: o.bbox[3])
        groups = [[ordered[0]]]
        for o in ordered[1:]:
            if o.bbox[3] - groups[-1][-1].bbox[3] < row_gap:
                groups[-1].append(o)
            else:
                groups.append([o])

        rows = []
        prev_line = 0
        for i, g in enumerate(groups):
            line = int(np.median([o.bbox[3] for o in g]))
            y_top = max(prev_line, min(o.bbox[1] for o in g) - 5)
            rows.append(ShelfRow(row_id=i, y_top=y_top, y_bottom=line,
                                 y_center=(y_top + line) // 2))
            prev_line = line
        return rows

    def _detect_shelf_rows(self, image_bgr: np.ndarray) -> list[ShelfRow]:
        """Fallback per immagini SENZA oggetti rilevati: Canny + HoughLinesP
        sui bordi orizzontali. Fragile ad alta risoluzione (venature del
        legno), per questo non è più la strada principale."""
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

        return rows

    def _assign_objects_to_rows(self, objects: list["DetectedObject"],
                                rows: list[ShelfRow], img_height: int = 0):
        """Assegna shelf_row e shelf_slot a ogni oggetto."""
        if not img_height:
            img_height = max(r.y_bottom for r in rows)
        row_gap = max(self.min_row_gap_px, int(self.row_gap_frac * img_height))
        for obj in objects:
            # La riga il cui piano (y_bottom) è più vicino al fondo della
            # bbox: è lì che l'oggetto poggia, indipendentemente da quanto
            # è alto (un centro geometrico penalizzava i libri alti).
            best_row = min(rows, key=lambda r: abs(r.y_bottom - obj.bbox[3]))
            cut_by_frame = obj.bbox[3] >= img_height - 2   # bbox tagliata dal bordo basso
            if cut_by_frame or abs(best_row.y_bottom - obj.bbox[3]) > row_gap:
                # Fondo lontano da ogni ripiano: non e' sullo scaffale
                # (testa del robot, tavolo, pavimento). shelf_row=-1, il
                # chiamante lo scarta dalla pipeline.
                obj.shelf_row = -1
                obj.shelf_slot = -1
                why = ("tagliato dal bordo inferiore dell'immagine" if cut_by_frame
                       else "lontano da ogni ripiano")
                log.warning(f"Oggetto [{obj.obj_id}] {obj.class_name} con "
                            f"fondo a y={obj.bbox[3]} {why}: fuori dallo "
                            f"scaffale, ignorato")
                continue
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
