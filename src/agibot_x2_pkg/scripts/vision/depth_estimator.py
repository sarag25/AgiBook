"""
Stima la profondità (distanza dal robot) di ogni oggetto rilevato.

Modalità supportate:
  - "rgbd"  : usa la depth map della camera RGBD del robot (rgbd_head_front_link)
  - "midas" : usa il modello MiDaS per depth monoculare da immagine RGB
  - "stereo": usa la coppia stereo (stereo_head_front_link) — richiede calibrazione
"""

from __future__ import annotations
import cv2
import numpy as np
import logging
from enum import Enum

log = logging.getLogger(__name__)


class DepthMode(str, Enum):
    RGBD   = "rgbd"
    MIDAS  = "midas"
    STEREO = "stereo"


class DepthEstimator:
    """
    Stima la distanza in metri di ogni oggetto nella scena.

    Uso con RGBD (più accurato, usa la camera del robot):
        estimator = DepthEstimator(mode="rgbd")
        estimator.set_depth_map(depth_image_from_ros)
        depth = estimator.get_depth_at_bbox(bbox)

    Uso con MiDaS (monoculare, funziona con qualsiasi foto):
        estimator = DepthEstimator(mode="midas")
        depth_map = estimator.estimate_depth_map(image_bgr)
        depth = estimator.get_depth_at_bbox(bbox)
    """

    def __init__(self, mode: str = "midas",
                 # Parametri camera RGBD (calibrazione reale)
                 fx: float = 615.0, fy: float = 615.0,
                 cx: float = 320.0, cy: float = 240.0):
        self.mode = DepthMode(mode)
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self._depth_map: np.ndarray | None = None
        self._midas_model = None
        self._transform = None

        if self.mode == DepthMode.MIDAS:
            self._load_midas()

    def _load_midas(self):
        try:
            # Fail-fast (2026-09-06): senza timm il torch.hub.load fallisce
            # comunque, ma DOPO ~1 minuto di download/parsing - all'avvio di
            # library_manager_node era tempo perso a ogni run.
            import timm  # noqa: F401
            import torch
            model_type = "MiDaS_small"  # leggero, buono per real-time
            self._midas_model = torch.hub.load("intel-isl/MiDaS", model_type)
            self._midas_model.eval()
            transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self._transform = transforms.small_transform
            self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self._midas_model.to(self._device)
            log.info("MiDaS caricato")
        except Exception as e:
            log.warning(f"MiDaS non disponibile: {e}. Uso depth stimata da dimensioni.")
            self._midas_model = None

    # ─── Modalità RGBD ───────────────────────────────────────────────────────

    def set_depth_map(self, depth_image: np.ndarray):
        """
        Imposta la depth map ricevuta dal topic ROS:
          /rgbd_head_front/depth/image_raw (float32, valori in metri)
        """
        self._depth_map = depth_image.astype(np.float32)

    # ─── Modalità MiDaS ──────────────────────────────────────────────────────

    def estimate_depth_map(self, image_bgr: np.ndarray) -> np.ndarray:
        """
        Calcola la depth map monoculare con MiDaS.
        Valori più alti = più lontano (scala relativa, non metrica).
        """
        if self._midas_model is None:
            # Fallback: usa gradiente verticale come proxy di profondità
            gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
            self._depth_map = cv2.GaussianBlur(gray.astype(np.float32), (15, 15), 0)
            return self._depth_map

        import torch
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        input_batch = self._transform(rgb).to(self._device)
        with torch.no_grad():
            prediction = self._midas_model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=image_bgr.shape[:2],
                mode="bicubic", align_corners=False,
            ).squeeze()
        depth = prediction.cpu().numpy()
        # Normalizza in range 0-10 (metri approssimativi — scala relativa)
        depth = (depth - depth.min()) / (depth.max() - depth.min() + 1e-6)
        depth = depth * 10.0
        self._depth_map = depth
        return depth

    # ─── Query profondità ────────────────────────────────────────────────────

    def get_depth_at_bbox(self, bbox: tuple[int, int, int, int]) -> float:
        """
        Restituisce la profondità media nella regione centrale della bbox.
        Per RGBD: in metri. Per MiDaS: scala relativa (0-10).
        """
        if self._depth_map is None:
            return 0.0

        x1, y1, x2, y2 = bbox
        # Usa solo il 50% centrale della bbox (evita bordi rumorosi)
        mx1 = x1 + (x2 - x1) // 4
        mx2 = x2 - (x2 - x1) // 4
        my1 = y1 + (y2 - y1) // 4
        my2 = y2 - (y2 - y1) // 4

        region = self._depth_map[my1:my2, mx1:mx2]
        valid = region[region > 0]
        if len(valid) == 0:
            return 0.0

        return float(np.median(valid))

    def pixel_to_3d(self, px: int, py: int, depth_m: float) -> tuple[float, float, float]:
        """
        Proietta un pixel (px, py) a coordinate 3D del mondo [m]
        usando il modello pinhole con i parametri intrinseci della camera.
        """
        x = (px - self.cx) * depth_m / self.fx
        y = (py - self.cy) * depth_m / self.fy
        z = depth_m
        return (x, y, z)

    def estimate_shelf_dimensions(self,
                                  shelf_bbox: tuple[int, int, int, int],
                                  depth_m: float,
                                  image_shape: tuple) -> dict:
        """
        Stima larghezza e altezza fisiche dello scaffale in cm.
        """
        x1, y1, x2, y2 = shelf_bbox
        w_px = x2 - x1
        h_px = y2 - y1

        width_m  = w_px * depth_m / self.fx
        height_m = h_px * depth_m / self.fy

        return {
            "width_cm":  round(width_m * 100, 1),
            "height_cm": round(height_m * 100, 1),
            "depth_m":   round(depth_m, 3),
        }
