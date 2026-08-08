"""
Determina il colore dominante di un libro usando KMeans su HSV.
Mappa il colore a un nome leggibile (rosso, blu, verde, ecc.).
"""

from __future__ import annotations
import cv2
import numpy as np
from sklearn.cluster import KMeans
from dataclasses import dataclass


# Tabella colori: (nome, range_H_min, range_H_max, s_min, v_min)
# H in OpenCV è 0-179
COLOR_TABLE = [
    ("rosso",    0,   10, 80, 60),
    ("arancione",10,  25, 80, 60),
    ("giallo",   25,  35, 80, 60),
    ("verde",    35,  85, 60, 50),
    ("ciano",    85,  100,60, 50),
    ("blu",      100, 130,60, 50),
    ("viola",    130, 150,60, 50),
    ("magenta",  150, 170,60, 50),
    ("rosso",    170, 179,80, 60),   # rosso avvolge in HSV
]


@dataclass
class ColorResult:
    name: str
    rgb: tuple[int, int, int]
    hsv: tuple[int, int, int]
    hex: str


class ColorAnalyzer:
    """
    Estrae il colore dominante dalla copertina/dorso di un libro
    escludendo sfondo bianco/nero (tipicamente non sono il colore del libro).
    """

    def __init__(self, n_clusters: int = 3):
        self.n_clusters = n_clusters

    def analyze(self, image_bgr: np.ndarray,
                bbox: tuple[int, int, int, int]) -> ColorResult:
        """
        Prende il crop del libro dalla bbox e restituisce il colore dominante.
        """
        x1, y1, x2, y2 = bbox
        crop = image_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return ColorResult("sconosciuto", (128, 128, 128), (0, 0, 128), "#808080")

        return self._dominant_color(crop)

    def _dominant_color(self, crop_bgr: np.ndarray) -> ColorResult:
        hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
        pixels = hsv.reshape(-1, 3).astype(np.float32)

        # Escludi pixel quasi bianchi (V>200, S<40) e quasi neri (V<30)
        mask = (
            (pixels[:, 2] > 30) &
            (pixels[:, 2] < 240) &
            (pixels[:, 1] > 30)
        )
        filtered = pixels[mask]

        if len(filtered) < 10:
            filtered = pixels  # fallback: usa tutti i pixel

        k = min(self.n_clusters, len(filtered))
        km = KMeans(n_clusters=k, n_init=5, random_state=0)
        km.fit(filtered)

        # Cluster più grande = colore dominante
        counts = np.bincount(km.labels_)
        dominant_hsv = km.cluster_centers_[np.argmax(counts)].astype(int)

        # Converti in BGR → RGB
        hsv_pixel = np.uint8([[dominant_hsv]])
        bgr_pixel = cv2.cvtColor(hsv_pixel, cv2.COLOR_HSV2BGR)[0][0]
        rgb = (int(bgr_pixel[2]), int(bgr_pixel[1]), int(bgr_pixel[0]))
        hex_col = "#{:02x}{:02x}{:02x}".format(*rgb)

        name = self._hsv_to_name(dominant_hsv)
        return ColorResult(name, rgb, tuple(dominant_hsv), hex_col)

    def _hsv_to_name(self, hsv: np.ndarray) -> str:
        h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])

        # Grigio/bianco/nero basati su saturazione e valore
        if v < 35:
            return "nero"
        if s < 35 and v > 180:
            return "bianco"
        if s < 35:
            return "grigio"

        for name, h_min, h_max, s_min, v_min in COLOR_TABLE:
            if h_min <= h < h_max and s >= s_min and v >= v_min:
                return name

        return "altro"

    def sort_key_by_color(self, color_name: str) -> int:
        """Ritorna un indice per ordinare i libri per arcobaleno."""
        rainbow_order = [
            "rosso", "arancione", "giallo", "verde",
            "ciano", "blu", "viola", "magenta",
            "bianco", "grigio", "nero", "altro", "sconosciuto"
        ]
        try:
            return rainbow_order.index(color_name)
        except ValueError:
            return 99
