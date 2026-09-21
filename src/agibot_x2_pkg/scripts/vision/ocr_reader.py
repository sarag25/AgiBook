"""
Helpers for OCR on book spines/covers to extract title and author.
Handles the 4 book orientations by rotating the crop until text is found.
"""

from __future__ import annotations
import cv2
import numpy as np
import re
import logging
from dataclasses import dataclass

log = logging.getLogger(__name__)

# rotation (deg) -> orientation name
ROTATIONS = {
    0:   "upright",
    90:  "sideways_right",
    180: "inverted",
    270: "sideways_left",
}


@dataclass
class OCRResult:
    """
    OCR text, parsed title/author, orientation and mean confidence
    """
    raw_text: str
    title: str
    author: str
    orientation: str     # upright | sideways_right | sideways_left | inverted
    confidence: float    # 0-1 mean EasyOCR confidence


OCR_TARGET_PX = 800          # long side of the crop before OCR (see read_book)
OCR_MIN_LETTER_SCORE = 6.0   # below this (letters x confidence) retry with enhanced contrast
OCR_MERGE_CONF = 0.5         # min confidence of fragments merged from other rotations
_LETTERS = re.compile(r"[^A-Za-zÀ-ÿ]")   # strips everything that is not a letter


class OCRReader:
    """
    Read the text on a book spine with EasyOCR, rotating the crop if the book is not upright.
    Usage: result = OCRReader(languages=["it", "en"]).read_book(image_bgr, bbox)
    """

    def __init__(self, languages: list[str] = None):
        """
        Initialize EasyOCR (CPU) for the given languages
        """
        if languages is None:
            languages = ["it", "en"]
        try:
            import easyocr
            self.reader = easyocr.Reader(languages, gpu=False, verbose=False)
            log.info(f"EasyOCR initialized: languages={languages}")
        except ImportError:
            raise ImportError("Install EasyOCR: pip install easyocr")

    def read_book(self, image_bgr: np.ndarray,
                  bbox: tuple[int, int, int, int]) -> OCRResult:
        """
        Extract title and author from the book crop.
        Tries all rotations and keeps the one with most LETTERS read (digits do not
        count, or a rotation of spurious numbers can win). With few letters it retries
        on a CLAHE-enhanced crop (e.g. gold on dark red) and keeps the best.
        """
        x1, y1, x2, y2 = bbox
        crop = image_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return OCRResult("", "", "", "unknown", 0.0)
        # upscale: at 960x720 letters are 10-20 px, below what EasyOCR reads reliably
        scale = OCR_TARGET_PX / max(crop.shape[:2])
        if scale > 1.05:
            crop = cv2.resize(crop, None, fx=scale, fy=scale,
                              interpolation=cv2.INTER_CUBIC)

        best = self._read_rotations(crop)
        if best is None or best[0] < OCR_MIN_LETTER_SCORE:
            lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(lab[:, :, 0])
            best2 = self._read_rotations(cv2.cvtColor(lab, cv2.COLOR_LAB2BGR))
            if best2 is not None and (best is None or best2[0] > best[0]):
                best = best2
        if best is None:
            return OCRResult("", "", "", "unknown", 0.0)
        _score, angle, texts = best
        # title often vertical and subtitle/author horizontal: merge confident new fragments of other rotations
        seen = {_LETTERS.sub("", t[1]).lower() for t in texts}
        merged = list(texts)
        for other in self._last_rotations:
            if other[1] == angle:
                continue
            for t in other[2]:
                key = _LETTERS.sub("", t[1]).lower()
                if t[2] >= OCR_MERGE_CONF and len(key) >= 3 and key not in seen:
                    seen.add(key)
                    merged.append(t)
        texts = merged
        raw = " | ".join(t[1] for t in texts)
        avg_conf = float(np.mean([t[2] for t in texts]))
        result = OCRResult(raw_text=raw, title="", author="",
                           orientation=ROTATIONS[angle], confidence=avg_conf)
        result.title, result.author = self._parse_title_author(raw)
        log.debug(f"OCR: '{raw}' -> {result.orientation} title='{result.title}' author='{result.author}'")
        return result

    def _read_rotations(self, crop: np.ndarray):
        """
        (score, angle, EasyOCR fragments) of the rotation with the highest
        sum of letters x confidence, None if no text
        """
        best = None
        self._last_rotations = []
        for angle in ROTATIONS:
            texts = self.reader.readtext(self._rotate(crop, angle))
            if not texts:
                continue
            score = sum(len(_LETTERS.sub("", t[1])) * t[2] for t in texts)
            self._last_rotations.append((score, angle, texts))
            if best is None or score > best[0]:
                best = (score, angle, texts)
        return best

    def _rotate(self, image: np.ndarray, angle: int) -> np.ndarray:
        """
        Rotate the image clockwise by 0/90/180/270 degrees
        """
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
        Split the "|"-separated EasyOCR fragments into (title, author).
        Fragments with fewer than 3 letters are dropped (spurious digits/symbols);
        author = a 2-3 word capitalized fragment without digits, if any;
        title = ALL other fragments joined in order (a single fragment is too partial to search).
        For more robust parsing integrate an LLM (see input_handler.py)
        """
        parts = [p.strip() for p in raw.split("|")]
        parts = [p for p in parts if len(_LETTERS.sub("", p)) >= 3]
        if not parts:
            return "", ""
        author = ""
        for part in parts:
            words = part.split()
            if (2 <= len(words) <= 3 and all(w[0].isupper() for w in words)
                    and all(w[1:].islower() for w in words if len(w) > 1)   # "VaLBA" no
                    and not any(ch.isdigit() for ch in part)):
                author = part
                break
        title_parts = [p for p in parts if p != author]
        title = " ".join(title_parts) if title_parts else author
        return title, author
