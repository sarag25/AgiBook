"""
DepthShelfDetector - rilevamento degli oggetti sui ripiani dalla SOLA
profondita' della shelf_camera (2026-09-13).

Serve per far girare la pipeline automatica (foto -> geometria -> presa)
anche sul PC senza GPU dove SAM3 non entra in memoria: niente rete
neurale, solo geometria. I pixel della depth vengono riproiettati nel
frame della libreria (ShelfGeometry) e si tengono quelli che stanno
DENTRO uno scomparto e DAVANTI al pannello posteriore: cio' che resta
sono gli oggetti appoggiati sui ripiani. Le componenti connesse della
maschera danno le bbox; libro vs oggetto lo decide la forma (i dorsi sono
alti e stretti). Interfaccia identica a Sam3BookDetector/BookDetector
(detect(image_bgr) -> list[DetectedObject], draw_detections), cosi'
library_manager_node lo usa con detector:=depth. Non identifica nulla
(titolo/colore restano ai moduli successivi).
"""

from __future__ import annotations

import cv2
import numpy as np

from vision.book_detector import DetectedObject
from vision.shelf_geometry import (ShelfGeometry, SHELF_HALF_INNER_WIDTH, SHELF_BACK_INNER_Y,
                                   SHELF_FRONT_Y, SHELF_SURFACES_Z, SHELF_COMPARTMENT_H)


class DepthShelfDetector:

    def __init__(self, geometry: ShelfGeometry, min_area_px: int = 150,
                 book_aspect: float = 2.0, wall_margin: float = 0.006, cell: float = 0.004):
        self.geo = geometry
        self.min_area = min_area_px
        self.cell = cell             # cella della griglia (laterale, z) in metri
        self.book_aspect = book_aspect
        self.wall_margin = wall_margin

    def shelf_points(self):
        """Pixel validi riproiettati nel frame libreria, con la maschera
        'oggetto su un ripiano'. Ritorna (uu, vv, lat, front, z, sel)."""
        g = self.geo
        if g.depth is None:
            raise RuntimeError("DepthShelfDetector: depth non impostata (geometry.set_depth)")
        H, W = g.depth.shape
        vv, uu = np.mgrid[0:H, 0:W]
        d = g.depth
        ok = np.isfinite(d) & (d > 0.05)
        uu, vv = uu[ok], vv[ok]
        pw = g.pixels_to_world(uu.astype(float), vv.astype(float), d[ok].astype(float))
        loc = g.shelf_local(pw)
        lat, front, z = loc[:, 0], loc[:, 1], loc[:, 2]
        return uu, vv, lat, front, z, g.inside_shelf(loc, self.wall_margin)

    def detect(self, image_bgr: np.ndarray) -> list[DetectedObject]:
        """Componenti connesse NON nell'immagine ma nella griglia
        (laterale, z) del frame libreria: la camera guarda di sbieco e
        nell'immagine la copertina del vicino riempie lo spazio fra due
        dorsi (IT e Hunger Games uscivano come un oggetto solo); nel piano
        (laterale, z) una copertina collassa su una riga sola, lo spazio
        vuoto resta vuoto e gli oggetti si separano."""
        uu, vv, lat, front, z, sel = self.shelf_points()
        uu, vv, lat, z = uu[sel], vv[sel], lat[sel], z[sel]
        if len(lat) == 0:
            return []
        cell = self.cell
        gi = ((lat + SHELF_HALF_INNER_WIDTH) / cell).astype(int)
        gj = (z / cell).astype(int)
        gw, gh = int(2 * SHELF_HALF_INNER_WIDTH / cell) + 2, int(1.4 / cell) + 2
        grid = np.zeros((gh, gw), dtype=np.uint8)
        grid[gj, gi] = 255
        grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        n, labels, stats, _c = cv2.connectedComponentsWithStats(grid, connectivity=8)
        pix_label = labels[gj, gi]
        objects = []
        cands = []
        for k in range(1, n):
            m = pix_label == k
            if m.sum() < self.min_area:
                continue
            x1, x2 = int(uu[m].min()), int(uu[m].max()) + 1
            y1, y2 = int(vv[m].min()), int(vv[m].max()) + 1
            lat_span = float(lat[m].max() - lat[m].min())
            z_span = float(z[m].max() - z[m].min())
            # frammenti (2026-09-13: la fetta di mappamondo non coperta dalla
            # testa del robot usciva come "oggetto" 14 x 30 mm)
            # (2026-09-17: soglia laterale 12 -> 7 mm: Werther, 10 mm di
            # dorso, veniva scartato come frammento; il frammento del
            # mappamondo resta fuori grazie ai 4 cm di altezza minima)
            if lat_span < 0.007 or z_span < 0.04:
                continue
            cands.append((x1, y1, x2, y2, lat_span, z_span))
        cands.sort(key=lambda b: (b[1] // 150, b[0]))
        for k, (x1, y1, x2, y2, lat_span, z_span) in enumerate(cands, start=1):
            is_book = (z_span / max(lat_span, 1e-3)) >= self.book_aspect
            objects.append(DetectedObject(
                obj_id=k, class_name="book" if is_book else "object", is_book=is_book,
                bbox=(x1, y1, x2, y2), confidence=1.0,
                center=((x1 + x2) // 2, (y1 + y2) // 2), width_px=int(x2 - x1), height_px=int(y2 - y1)))
        return objects

    @staticmethod
    def draw_detections(image_bgr: np.ndarray, objects: list[DetectedObject]) -> np.ndarray:
        out = image_bgr.copy()
        for o in objects:
            x1, y1, x2, y2 = o.bbox
            col = (0, 200, 0) if o.is_book else (0, 140, 255)
            cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
            label = f"{o.obj_id} {o.class_name}"
            if o.title:
                label += f" {o.title[:18]}"
            if getattr(o, "thickness_m", 0.0):
                label += f" {o.thickness_m*1000:.0f}mm"
            cv2.putText(out, label, (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        return out
