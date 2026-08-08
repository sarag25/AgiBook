"""
Gestisce input testuale e vocale dell'utente.
Converte un comando in linguaggio naturale in un criterio di ordinamento strutturato.

Supporta:
  - Input testuale diretto
  - Input vocale via microfono (Whisper)
  - Parsing NLP locale (regex) o tramite LLM (OpenAI / Ollama)
"""

from __future__ import annotations
import re
import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional

log = logging.getLogger(__name__)


class SortCriterion(str, Enum):
    COLOR      = "color"
    TITLE      = "title"
    AUTHOR     = "author"
    SIZE       = "size"       # ottimizzazione spazio
    NONE       = "none"


@dataclass
class SortCommand:
    criterion: SortCriterion
    ascending: bool = True        # A→Z, basso→alto, arcobaleno→inverso
    lifo_mode: bool = False       # rimuovi in ordine inverso prima
    target_color: Optional[str] = None   # es. "blu" per raggruppare per colore
    raw_text: str = ""

    def __str__(self):
        direction = "crescente" if self.ascending else "decrescente"
        lifo = " [LIFO]" if self.lifo_mode else ""
        return (f"Criterio: {self.criterion.value} | "
                f"{direction}{lifo} | input: '{self.raw_text}'")


# Pattern regex per il parsing locale (senza LLM)
_PATTERNS = {
    SortCriterion.COLOR: [
        r"\bcolor[ei]?\b", r"\btonali\b", r"\barcobaleno\b",
        r"\bcolor\b", r"\bhue\b", r"\btint[oa]\b",
    ],
    SortCriterion.TITLE: [
        r"\btitol[oi]\b", r"\balfabet\w+\b", r"\bA.?Z\b",
        r"\bnome\b", r"\btitle\b", r"\balphabetical\b",
    ],
    SortCriterion.AUTHOR: [
        r"\bautor[ei]\b", r"\bauthor\b", r"\bscrittor\w+\b",
    ],
    SortCriterion.SIZE: [
        r"\bspazi[oo]\b", r"\bdimensi\w+\b", r"\baltezz\w+\b",
        r"\bsize\b", r"\bspace\b", r"\blarghezz\w+\b", r"\bcompatt\w+\b",
    ],
}

_DESC_PATTERNS = [r"\bdecrescente\b", r"\binverso\b", r"\bZ.?A\b",
                  r"\bdescending\b", r"\breverse\b"]

_LIFO_PATTERNS = [r"\blifo\b", r"\border[e]?\s+invers\w+\b",
                  r"\brisparmia\w+\b", r"\bsenza\s+rimesc\w+\b"]


class InputHandler:
    """
    Converte il comando dell'utente (testo o voce) in un SortCommand.

    Uso:
        handler = InputHandler(use_llm=False)
        cmd = handler.parse("ordina per colore in ordine inverso con LIFO")
        print(cmd)
    """

    def __init__(self, use_llm: bool = False,
                 llm_model: str = "gpt-4o-mini",
                 whisper_model: str = "base"):
        self.use_llm = use_llm
        self.llm_model = llm_model
        self.whisper_model = whisper_model
        self._whisper = None

    # ─── Entry point testuale ─────────────────────────────────────────────

    def parse(self, text: str) -> SortCommand:
        """Analizza una stringa di testo e restituisce il SortCommand."""
        text_lower = text.lower().strip()
        if self.use_llm:
            return self._parse_with_llm(text_lower)
        return self._parse_with_regex(text_lower, text)

    # ─── Entry point vocale ───────────────────────────────────────────────

    def listen_and_parse(self, duration_sec: float = 5.0) -> SortCommand:
        """
        Registra audio dal microfono e trascrivi con Whisper.
        """
        try:
            import sounddevice as sd
            import numpy as np
        except ImportError:
            raise ImportError("Installa sounddevice: pip install sounddevice")

        sample_rate = 16000
        log.info(f"Ascolto per {duration_sec}s...")
        audio = sd.rec(int(duration_sec * sample_rate),
                       samplerate=sample_rate, channels=1, dtype="float32")
        sd.wait()
        audio_flat = audio.flatten()

        text = self._transcribe(audio_flat, sample_rate)
        log.info(f"Trascritto: '{text}'")
        return self.parse(text)

    def _transcribe(self, audio: "np.ndarray", sample_rate: int) -> str:
        if self._whisper is None:
            try:
                import whisper
                self._whisper = whisper.load_model(self.whisper_model)
            except ImportError:
                raise ImportError("Installa Whisper: pip install openai-whisper")

        result = self._whisper.transcribe(audio, language="it", fp16=False)
        return result.get("text", "").strip()

    # ─── Parser locale (regex) ────────────────────────────────────────────

    def _parse_with_regex(self, text_lower: str, raw: str) -> SortCommand:
        criterion = SortCriterion.NONE
        for crit, patterns in _PATTERNS.items():
            if any(re.search(p, text_lower) for p in patterns):
                criterion = crit
                break

        ascending = not any(re.search(p, text_lower) for p in _DESC_PATTERNS)
        lifo = any(re.search(p, text_lower) for p in _LIFO_PATTERNS)

        cmd = SortCommand(
            criterion=criterion,
            ascending=ascending,
            lifo_mode=lifo,
            raw_text=raw,
        )
        log.info(f"Comando parsato (regex): {cmd}")
        return cmd

    # ─── Parser LLM ───────────────────────────────────────────────────────

    def _parse_with_llm(self, text: str) -> SortCommand:
        """
        Usa OpenAI o Ollama per interpretare comandi ambigui o complessi.
        """
        prompt = f"""
Sei un assistente per un robot che riordina libri in una libreria.
L'utente ha detto: "{text}"

Rispondi SOLO con un JSON con questi campi:
- criterion: "color" | "title" | "author" | "size" | "none"
- ascending: true | false
- lifo_mode: true | false  (true se l'utente vuole ottimizzare rimuovendo in ordine inverso)

Esempi:
- "metti i libri per colore arcobaleno" → {{"criterion":"color","ascending":true,"lifo_mode":false}}
- "ordina per autore dalla Z alla A con LIFO" → {{"criterion":"author","ascending":false,"lifo_mode":true}}
"""
        try:
            import json
            from openai import OpenAI
            client = OpenAI()
            resp = client.chat.completions.create(
                model=self.llm_model,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"},
                temperature=0,
            )
            data = json.loads(resp.choices[0].message.content)
            return SortCommand(
                criterion=SortCriterion(data.get("criterion", "none")),
                ascending=data.get("ascending", True),
                lifo_mode=data.get("lifo_mode", False),
                raw_text=text,
            )
        except Exception as e:
            log.warning(f"LLM non disponibile ({e}), uso regex come fallback")
            return self._parse_with_regex(text, text)
