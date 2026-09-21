"""
Helpers that turn a user text or voice command into a structured book sorting criterion.
Text is parsed locally with regexes (Italian and English keywords) or with an LLM (OpenAI / Ollama);
voice is recorded from the microphone and transcribed with Whisper.
    handler = InputHandler(use_llm=False)
    cmd = handler.parse("ordina per colore in ordine inverso con LIFO")
"""

from __future__ import annotations
import re
import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional

log = logging.getLogger(__name__)


class SortCriterion(str, Enum):
    """
    Criterion used to sort the books
    """
    COLOR      = "color"
    TITLE      = "title"
    AUTHOR     = "author"
    SIZE       = "size"       # space optimisation
    NONE       = "none"


@dataclass
class SortCommand:
    """
    Parsed sorting command: criterion, direction, LIFO mode and original text
    """
    criterion: SortCriterion
    ascending: bool = True        # A->Z, low->high, rainbow order (False = reversed)
    lifo_mode: bool = False       # remove books in reverse order first
    target_color: Optional[str] = None   # e.g. "blu" to group by colour
    raw_text: str = ""

    def __str__(self):
        """
        Readable one-line summary of the command
        """
        direction = "crescente" if self.ascending else "decrescente"
        lifo = " [LIFO]" if self.lifo_mode else ""
        return (f"Criterio: {self.criterion.value} | "
                f"{direction}{lifo} | input: '{self.raw_text}'")


# regexes for local parsing (no LLM), Italian and English keywords
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
    Convert a user command (text or voice) into a SortCommand
    """

    def __init__(self, use_llm: bool = False,
                 llm_model: str = "gpt-4o-mini",
                 whisper_model: str = "base"):
        """
        Choose regex or LLM parsing and the LLM/Whisper models; Whisper is loaded lazily
        """
        self.use_llm = use_llm
        self.llm_model = llm_model
        self.whisper_model = whisper_model
        self._whisper = None

    def parse(self, text: str) -> SortCommand:
        """
        Parse a text string into a SortCommand
        """
        text_lower = text.lower().strip()
        if self.use_llm:
            return self._parse_with_llm(text_lower)
        return self._parse_with_regex(text_lower, text)

    def listen_and_parse(self, duration_sec: float = 5.0) -> SortCommand:
        """
        Record audio from the microphone, transcribe it with Whisper and parse it
        """
        try:
            import sounddevice as sd
            import numpy as np
        except ImportError:
            raise ImportError("Install sounddevice: pip install sounddevice")

        sample_rate = 16000
        log.info(f"Listening for {duration_sec}s...")
        audio = sd.rec(int(duration_sec * sample_rate),
                       samplerate=sample_rate, channels=1, dtype="float32")
        sd.wait()
        audio_flat = audio.flatten()

        text = self._transcribe(audio_flat, sample_rate)
        log.info(f"Transcribed: '{text}'")
        return self.parse(text)

    def _transcribe(self, audio: "np.ndarray", sample_rate: int) -> str:
        """
        Transcribe Italian audio with Whisper, loading the model on first use
        """
        if self._whisper is None:
            try:
                import whisper
                self._whisper = whisper.load_model(self.whisper_model)
            except ImportError:
                raise ImportError("Install Whisper: pip install openai-whisper")

        result = self._whisper.transcribe(audio, language="it", fp16=False)
        return result.get("text", "").strip()

    def _parse_with_regex(self, text_lower: str, raw: str) -> SortCommand:
        """
        Parse with keyword regexes: first matching criterion wins, then direction and LIFO flags
        """
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
        log.info(f"Parsed command (regex): {cmd}")
        return cmd

    def _parse_with_llm(self, text: str) -> SortCommand:
        """
        Parse ambiguous or complex commands with an LLM, falling back to regexes on error
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
            log.warning(f"LLM not available ({e}), falling back to regex")
            return self._parse_with_regex(text, text)
