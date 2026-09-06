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
    Estrae il colore dominante dal dorso di un libro.

    Rivisto 2026-09-06 (sulla foto della shelf_camera i 4 libri uscivano
    "arancione", "arancione", "magenta", "arancione"):
      - si guarda solo la ZONA CENTRALE della bbox (orizzontalmente il
        60% centrale, verticalmente dal 25% al 92%): la bbox e' allineata
        agli assi attorno a un libro visto in prospettiva, quindi negli
        angoli c'e' il legno dello scaffale (arancione!) e in alto la
        costa bianca delle pagine;
      - il bianco e il nero NON vengono piu' scartati: sono colori di
        libro legittimi (IT e' bianco). Scartarli lasciava, per un dorso
        bianco, solo il legno degli angoli;
      - il clustering avviene in Lab (spazio percettivo, niente hue
        circolare che spezzava i grigi in cluster casuali); il nome si
        assegna sull'HSV del centro dominante.
    """

    # Frazioni della bbox usate per il campionamento (x0, x1, y0, y1)
    SAMPLE_REGION = (0.20, 0.80, 0.25, 0.92)

    def __init__(self, n_clusters: int = 3):
        self.n_clusters = n_clusters

    def analyze(self, image_bgr: np.ndarray,
                bbox: tuple[int, int, int, int]) -> ColorResult:
        """
        Prende la zona centrale del dorso dalla bbox e restituisce il
        colore dominante.
        """
        x1, y1, x2, y2 = bbox
        w, h = x2 - x1, y2 - y1
        fx0, fx1, fy0, fy1 = self.SAMPLE_REGION
        cx1, cx2 = x1 + int(w * fx0), x1 + int(w * fx1)
        cy1, cy2 = y1 + int(h * fy0), y1 + int(h * fy1)
        crop = image_bgr[cy1:cy2, cx1:cx2]
        if crop.size < 3 * 4:          # bbox minuscola: usa tutta la bbox
            crop = image_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return ColorResult("sconosciuto", (128, 128, 128), (0, 0, 128), "#808080")

        return self._dominant_color(crop)

    def _dominant_color(self, crop_bgr: np.ndarray) -> ColorResult:
        lab = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2LAB)
        pixels = lab.reshape(-1, 3).astype(np.float32)

        k = min(self.n_clusters, len(pixels))
        km = KMeans(n_clusters=k, n_init=5, random_state=0)
        km.fit(pixels)

        # Cluster più grande = colore dominante
        counts = np.bincount(km.labels_)
        dominant_lab = km.cluster_centers_[np.argmax(counts)]

        lab_pixel = np.uint8([[np.clip(dominant_lab, 0, 255)]])
        bgr_pixel = cv2.cvtColor(lab_pixel, cv2.COLOR_LAB2BGR)[0][0]
        dominant_hsv = cv2.cvtColor(np.uint8([[bgr_pixel]]),
                                    cv2.COLOR_BGR2HSV)[0][0].astype(int)
        rgb = (int(bgr_pixel[2]), int(bgr_pixel[1]), int(bgr_pixel[0]))
        hex_col = "#{:02x}{:02x}{:02x}".format(*rgb)

        name = self._hsv_to_name(dominant_hsv)
        return ColorResult(name, rgb, tuple(int(v) for v in dominant_hsv), hex_col)

    def _hsv_to_name(self, hsv: np.ndarray) -> str:
        h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])

        # Grigio/bianco/nero basati su saturazione e valore
        if v < 35:
            return "nero"
        if s < 35 and v > 180:
            return "bianco"
        if s < 35:
            return "grigio"

        # Sopra le soglie di grigio/nero il pixel E' colorato: si nomina
        # per sola tonalita'. Le colonne s_min/v_min della tabella non si
        # usano piu' (2026-09-06): con la luce dall'alto dello scaffale un
        # dorso viola scuro ha V~38 e uno bordeaux S~60, ed entrambi
        # finivano in "altro" - un colore inutile per l'ordinamento.
        for name, h_min, h_max, _s_min, _v_min in COLOR_TABLE:
            if h_min <= h < h_max:
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
