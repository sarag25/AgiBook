"""
Helpers to estimate the depth (distance from the robot) of each detected object.
Modes: "rgbd" (depth map of rgbd_head_front_link), "midas" (monocular MiDaS on RGB),
"stereo" (stereo_head_front_link pair, needs calibration).
"""

from __future__ import annotations
import cv2
import numpy as np
import logging
from enum import Enum

log = logging.getLogger(__name__)


class DepthMode(str, Enum):
    """
    Supported depth sources
    """
    RGBD   = "rgbd"
    MIDAS  = "midas"
    STEREO = "stereo"


class DepthEstimator:
    """
    Estimate the distance of each object in the scene.
    RGBD (metric, robot camera): set_depth_map(depth) then get_depth_at_bbox(bbox)
    MiDaS (relative, any photo): estimate_depth_map(image_bgr) then get_depth_at_bbox(bbox)
    """

    def __init__(self, mode: str = "midas",
                 # RGBD camera intrinsics
                 fx: float = 615.0, fy: float = 615.0,
                 cx: float = 320.0, cy: float = 240.0):
        """
        Set mode and camera intrinsics; load MiDaS if needed
        """
        self.mode = DepthMode(mode)
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self._depth_map: np.ndarray | None = None
        self._midas_model = None
        self._transform = None

        if self.mode == DepthMode.MIDAS:
            self._load_midas()

    def _load_midas(self):
        """
        Load MiDaS_small; importing timm first fails fast instead of after ~1 min of torch.hub download
        """
        try:
            import timm  # noqa: F401
            import torch
            model_type = "MiDaS_small"  # light, fit for real time
            self._midas_model = torch.hub.load("intel-isl/MiDaS", model_type)
            self._midas_model.eval()
            transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self._transform = transforms.small_transform
            self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self._midas_model.to(self._device)
            log.info("MiDaS loaded")
        except Exception as e:
            log.warning(f"MiDaS not available: {e}. Using depth estimated from sizes.")
            self._midas_model = None

    # ─── RGBD mode ───────────────────────────────────────────────────────────

    def set_depth_map(self, depth_image: np.ndarray):
        """
        Set the depth map from /rgbd_head_front/depth/image_raw (float32, m)
        """
        self._depth_map = depth_image.astype(np.float32)

    # ─── MiDaS mode ──────────────────────────────────────────────────────────

    def estimate_depth_map(self, image_bgr: np.ndarray) -> np.ndarray:
        """
        Monocular depth map with MiDaS (relative scale, not metric)
        """
        if self._midas_model is None:
            # fallback: blurred grey level as a depth proxy
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
        # normalize to 0-10 (relative scale)
        depth = (depth - depth.min()) / (depth.max() - depth.min() + 1e-6)
        depth = depth * 10.0
        self._depth_map = depth
        return depth

    # ─── Depth queries ───────────────────────────────────────────────────────

    def get_depth_at_bbox(self, bbox: tuple[int, int, int, int]) -> float:
        """
        Median depth in the central region of the bbox (RGBD: m, MiDaS: relative 0-10)
        """
        if self._depth_map is None:
            return 0.0

        x1, y1, x2, y2 = bbox
        # central 50% only, edges are noisy
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
        Back-project pixel (px, py) to a 3D point in m with the pinhole intrinsics
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
        Physical width and height of the shelf in cm
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
