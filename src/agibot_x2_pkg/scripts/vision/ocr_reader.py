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


# Lato lungo a cui portare il crop prima dell'OCR (vedi read_book)
OCR_TARGET_PX = 800
# Sotto questo punteggio (lettere x confidenza) si riprova con il contrasto
# aumentato (2026-09-17)
OCR_MIN_LETTER_SCORE = 6.0
# Frammenti delle altre rotazioni aggiunti al testo solo se sicuri
OCR_MERGE_CONF = 0.5
_LETTERS = re.compile(r"[^A-Za-zÀ-ÿ]")   # toglie tutto cio' che non e' lettera


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
        Prova tutte le rotazioni e usa quella con piu' LETTERE lette
        (2026-09-17: prima il punteggio contava anche le cifre, e sul dorso
        rosso di Hunger Games vinceva una rotazione di soli numeri "04 5 0 1
        8"). Se le lettere sono poche riprova sul crop con il contrasto
        aumentato (CLAHE: oro su rosso scuro) e tiene il migliore.
        """
        x1, y1, x2, y2 = bbox
        crop = image_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return OCRResult("", "", "", "unknown", 0.0)
        # Upscaling (2026-09-06): sulla foto 960x720 un dorso e' largo
        # 60-130 px e le lettere 10-20 px, sotto la soglia utile di EasyOCR
        # ("STEPHEN KING" letto "NaHdiis"). Portiamo il lato lungo a
        # ~OCR_TARGET_PX prima delle rotazioni.
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
        # Sui dorsi il titolo e' spesso verticale e il sottotitolo/autore
        # orizzontale ("HUNGER GAMES" + "L'alba sulla mietitura"): una sola
        # rotazione ne perde uno. Alla rotazione migliore si aggiungono i
        # frammenti sicuri (conf >= OCR_MERGE_CONF, >= 3 lettere) delle
        # altre rotazioni non ancora presenti (2026-09-17).
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
        log.debug(f"OCR: '{raw}' -> {result.orientation} titolo='{result.title}' autore='{result.author}'")
        return result

    def _read_rotations(self, crop: np.ndarray):
        """(punteggio, angolo, frammenti EasyOCR) della rotazione con piu'
        lettere (somma lettere x confidenza), None se nessun testo."""
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
        Dai frammenti EasyOCR (separati da "|", nell'ordine di lettura):
        - si tengono solo i frammenti con almeno 3 lettere (le cifre spurie
          "4", "3", "[" dei dorsi vanno via);
        - autore = un frammento di 2-3 parole con l'iniziale maiuscola e
          senza cifre ("Suzanne Collins"), se c'e';
        - titolo = TUTTI gli altri frammenti uniti nell'ordine ("HUNGER |
          GAMES" -> "HUNGER GAMES"). Prima si teneva solo il frammento piu'
          lungo e il titolo usciva "HUNGER", "STEPHEN": irriconoscibili
          per la ricerca (2026-09-17).
        Per un parsing piu' robusto integrare un LLM (vedi input_handler.py).
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
