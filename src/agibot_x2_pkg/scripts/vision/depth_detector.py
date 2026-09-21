"""
Detector of the objects on the shelves from the shelf_camera depth only (no neural net, no GPU needed).
Depth pixels inside a compartment and in front of the back panel are grouped into objects;
book vs object is decided by shape (spines are tall and narrow).
Same interface as Sam3BookDetector/BookDetector; used by library_manager_node with detector:=depth.
"""

from __future__ import annotations

import cv2
import numpy as np

from vision.book_detector import DetectedObject
from vision.shelf_geometry import (ShelfGeometry, SHELF_HALF_INNER_WIDTH, SHELF_BACK_INNER_Y,
                                   SHELF_FRONT_Y, SHELF_SURFACES_Z, SHELF_COMPARTMENT_H)


class DepthShelfDetector:
    """
    Depth-only detector of the objects standing on the shelves
    """

    def __init__(self, geometry: ShelfGeometry, min_area_px: int = 150,
                 book_aspect: float = 2.0, wall_margin: float = 0.006, cell: float = 0.004):
        """
        Store the geometry and the detection thresholds
        """
        self.geo = geometry
        self.min_area = min_area_px
        self.cell = cell             # (lateral, z) grid cell size in m
        self.book_aspect = book_aspect
        self.wall_margin = wall_margin

    def shelf_points(self):
        """
        Valid pixels reprojected into the shelf frame, with the "object on a shelf" mask.
        Returns (uu, vv, lat, front, z, sel)
        """
        g = self.geo
        if g.depth is None:
            raise RuntimeError("DepthShelfDetector: depth not set (geometry.set_depth)")
        H, W = g.depth.shape
        vv, uu = np.mgrid[0:H, 0:W]
        d = g.depth
        ok = np.isfinite(d) & (d > 0.05)
        uu, vv = uu[ok], vv[ok]
        pw = g.pixels_to_world(uu.astype(float), vv.astype(float), d[ok].astype(float))
        loc = g.shelf_local(pw)
        lat, front, z = loc[:, 0], loc[:, 1], loc[:, 2]
        # floor_margin 10 mm: an extra ~0.3 deg body tilt is ~7 mm at 1.3 m, and the shelf strip would merge all books
        return uu, vv, lat, front, z, g.inside_shelf(loc, self.wall_margin, floor_margin=0.010)

    def detect(self, image_bgr: np.ndarray) -> list[DetectedObject]:
        """
        Detect objects as connected components of the (lateral, z) shelf-frame grid.
        Not in the image: the camera looks at an angle and a neighbour's cover fills
        the gap between two spines; in the (lateral, z) plane a cover collapses to one
        line and the objects stay separate
        """
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
            # drop fragments: 7 mm keeps thin spines (10 mm), 4 cm height drops partly hidden objects
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
        """
        Draw the bounding boxes on a copy of the image for debugging
        """
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
