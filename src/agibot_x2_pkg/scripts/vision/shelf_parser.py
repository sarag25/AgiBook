"""
Helpers to parse the bookshelf structure in an image (shelf rows, slots)
and assign each detected object to its row and slot.
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
    """
    One shelf row in the image (pixel rows), with its slots and objects
    """
    row_id: int
    y_top: int
    y_bottom: int
    y_center: int
    slots: list = field(default_factory=list)   # slots sorted by x
    objects: list = field(default_factory=list)  # assigned DetectedObjects


@dataclass
class ShelfSlot:
    """
    One occupied or empty slot of a row (pixel columns)
    """
    slot_id: int
    row_id: int
    x_center: int
    x_left: int = 0
    x_right: int = 0
    occupied: bool = False
    obj_id: int = -1


class ShelfParser:
    """
    Find the shelf rows in the image and assign each book/object to its row and slot.
    Rows come from the OBJECTS: bbox bottoms (y2) cluster at one height per shelf
    (Hough lines on wood grain gave phantom rows; kept only as fallback). An empty
    shelf yields no row. Empty slots are gaps wider than an average book; they tell
    WHICH ROW has room, not a precise X target for the arm.
    """

    DEFAULT_BOOK_WIDTH_PX = 45   # uncalibrated fallback for a row without books

    def __init__(self, min_row_gap_px: int = 40, min_gap_factor: float = 0.8,
                 row_gap_frac: float = 0.15):
        """
        row_gap_frac 0.15 (108 px at 720p): same-shelf objects at different depths differ ~70 px, shelves 250+.
        min_gap_factor: a gap must be this x the mean book width to count as an empty slot
        """
        self.min_row_gap_px = min_row_gap_px
        # min vertical gap between bottoms on different shelves: max(min_row_gap_px, row_gap_frac * img height)
        self.row_gap_frac = row_gap_frac
        # so the normal spacing between spines is not taken for a free slot
        self.min_gap_factor = min_gap_factor

    def parse(self, image_bgr: np.ndarray,
              objects: list["DetectedObject"]) -> list[ShelfRow]:
        """
        Detect the rows and assign positions; sets shelf_row/shelf_slot of each DetectedObject in place
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
        log.info(f"Shelf rows detected: {len(rows)} ({source})")

        self._assign_objects_to_rows(objects, rows, h)
        for row in rows:
            self._build_slots(row, w)
        return rows

    def _rows_from_objects(self, objects: list["DetectedObject"],
                           img_height: int) -> list[ShelfRow]:
        """
        One row per group of objects with the bbox bottom at the same height.
        Shelf line = median of the bottoms; the row spans from the previous line
        (or the top of the tallest object) to it
        """
        if not objects:
            return []
        # only BOOKS define rows if any: generic "object" hits (robot head, table, wall) create phantom rows
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
        """
        Fallback for images WITHOUT detected objects: Canny + HoughLinesP on horizontal edges
        (fragile at high resolution because of wood grain)
        """
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi/180,
            threshold=80, minLineLength=image_bgr.shape[1] // 3,
            maxLineGap=30
        )

        if lines is None:
            return []

        # keep near-horizontal lines (|dy| < 10 px); reshape because OpenCV <5 returns (N,1,4), OpenCV 5 (N,4)
        h_lines = []
        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
            if abs(y2 - y1) < 10:
                h_lines.append((y1 + y2) // 2)

        if not h_lines:
            return []

        # cluster close lines -> one row per cluster
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

        # last row down to the image bottom
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
        """
        Set shelf_row and shelf_slot of each object; objects far from every row get -1
        """
        if not img_height:
            img_height = max(r.y_bottom for r in rows)
        row_gap = max(self.min_row_gap_px, int(self.row_gap_frac * img_height))
        for obj in objects:
            # row whose line is closest to the bbox bottom (where the object rests, whatever its height)
            best_row = min(rows, key=lambda r: abs(r.y_bottom - obj.bbox[3]))
            cut_by_frame = obj.bbox[3] >= img_height - 2   # bbox cut by the bottom edge
            if cut_by_frame or abs(best_row.y_bottom - obj.bbox[3]) > row_gap:
                # not on the shelf (robot head, table, floor): the caller drops shelf_row=-1
                obj.shelf_row = -1
                obj.shelf_slot = -1
                why = ("tagliato dal bordo inferiore dell'immagine" if cut_by_frame
                       else "lontano da ogni ripiano")
                log.warning(f"Object [{obj.obj_id}] {obj.class_name} with "
                            f"bottom at y={obj.bbox[3]} {why}: off the "
                            f"shelf, ignored")
                continue
            obj.shelf_row = best_row.row_id
            best_row.objects.append(obj)

        # sort by x_center -> shelf_slot
        for row in rows:
            row.objects.sort(key=lambda o: o.center[0])
            for slot_idx, obj in enumerate(row.objects):
                obj.shelf_slot = slot_idx

        log.debug("Row/slot assignment done")

    def _build_slots(self, row: ShelfRow, img_width: int):
        """
        Fill row.slots with ALL slots of the row (row.objects already sorted by x).
        One occupied slot per object, empty slots for each gap wide enough for an
        average book; a large gap becomes several empty slots so they are counted correctly
        """
        row.slots = []

        widths = [o.width_px for o in row.objects if o.width_px > 0]
        ref_width = int(np.median(widths)) if widths else self.DEFAULT_BOOK_WIDTH_PX
        min_gap = max(1, int(ref_width * self.min_gap_factor))

        def emit_empty_run(x_left: int, x_right: int):
            """
            Append the empty slots that fit between x_left and x_right
            """
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
        """
        All empty slots of all rows, sorted by row then x
        """
        return [s for r in rows for s in r.slots if not s.occupied]

    def occupancy_summary(self, rows: list[ShelfRow]) -> dict:
        """
        Geometric occupancy of the bookshelf, per row and total (not whether it is sorted)
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
        """
        Draw the detected rows and the empty slots on a copy of the image for debugging
        """
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
