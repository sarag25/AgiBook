"""
OCR sul dorso/copertina dei libri per estrarre titolo e autore.
Gestisce le 4 orientazioni del libro ruotando l'immagine fino a trovare testo.
"""

from __future__ import annotations
import cv2
import numpy as np
import re
import logging
from dataclasses import dataclass

log = logging.getLogger(__name__)

# Mapping rotazione → nome orientazione
ROTATIONS = {
    0:   "upright",
    90:  "sideways_right",
    180: "inverted",
    270: "sideways_left",
}


@dataclass
class OCRResult:
    raw_text: str
    title: str
    author: str
    orientation: str     # upright | sideways_right | sideways_left | inverted
    confidence: float    # 0-1 media confidence EasyOCR


class OCRReader:
    """
    Legge il testo sul dorso del libro usando EasyOCR.
    Se il libro non è in posizione upright, lo ruota automaticamente.

    Uso:
        reader = OCRReader(languages=["it", "en"])
        result = reader.read_book(image_bgr, bbox)
    """

    def __init__(self, languages: list[str] = None):
        if languages is None:
            languages = ["it", "en"]
        try:
            import easyocr
            self.reader = easyocr.Reader(languages, gpu=False, verbose=False)
            log.info(f"EasyOCR inizializzato: lingue={languages}")
        except ImportError:
            raise ImportError("Installa EasyOCR: pip install easyocr")

    def read_book(self, image_bgr: np.ndarray,
                  bbox: tuple[int, int, int, int]) -> OCRResult:
        """
        Estrae titolo e autore dal crop del libro.
        Prova tutte le rotazioni e usa quella con più testo.
        """
        x1, y1, x2, y2 = bbox
        crop = image_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return OCRResult("", "", "", "unknown", 0.0)

        best_result = None
        best_score = 0.0

        for angle, orientation_name in ROTATIONS.items():
            rotated = self._rotate(crop, angle)
            texts = self.reader.readtext(rotated)

            if not texts:
                continue

            # Score = somma (lunghezza testo × confidence)
            score = sum(len(t[1]) * t[2] for t in texts)
            if score > best_score:
                best_score = score
                raw = " | ".join(t[1] for t in texts)
                avg_conf = float(np.mean([t[2] for t in texts]))
                best_result = OCRResult(
                    raw_text=raw,
                    title="",
                    author="",
                    orientation=orientation_name,
                    confidence=avg_conf,
                )

        if best_result is None:
            return OCRResult("", "", "", "unknown", 0.0)

        best_result.title, best_result.author = self._parse_title_author(
            best_result.raw_text
        )
        log.debug(f"OCR: '{best_result.raw_text}' → {best_result.orientation}")
        return best_result

    def _rotate(self, image: np.ndarray, angle: int) -> np.ndarray:
        if angle == 0:
            return image
        elif angle == 90:
            return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif angle == 180:
            return cv2.rotate(image, cv2.ROTATE_180)
        elif angle == 270:
            return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return image

    def _parse_title_author(self, raw: str) -> tuple[str, str]:
        """
        Euristica semplice: la parte più lunga prima del separatore "|" è il titolo,
        la parte più breve (spesso in basso sul dorso) può essere l'autore.
        Per un parsing più robusto integrare un LLM (vedi input_handler.py).
        """
        parts = [p.strip() for p in raw.split("|") if p.strip()]
        if not parts:
            return "", ""

        # Il testo più lungo di solito è il titolo
        parts.sort(key=len, reverse=True)
        title = parts[0] if parts else ""

        # Cerca pattern autore: "Nome Cognome" (2 parole, iniziali maiuscole)
        author = ""
        for part in parts[1:]:
            words = part.split()
            if 1 < len(words) <= 4 and all(w[0].isupper() for w in words if w):
                author = part
                break

        return title, author
