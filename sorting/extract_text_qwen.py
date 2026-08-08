"""
extract_text_qwen.py
=====================
Estrae il testo visibile in un ritaglio di libro usando un modello
multimodale (Qwen/Qwen3-VL-30B-A3B-Instruct) via Inference API di Hugging
Face — stessa interfaccia e stesso ruolo di extract_text_llama.py (che
usa meta-llama/Llama-4-Scout-17B-16E-Instruct), modello diverso da provare
in alternativa nella pipeline di sorting.

Sostituisce SOLO l'estrazione di testo grezzo, non l'identificazione del
libro: l'interpretazione (titolo, autore, ISBN, punteggio candidati) resta
nella logica già esistente in extract_cover.py/extract_isbn.py (ricerca su
Google Books, regex ISBN) — qui si chiede al modello di trascrivere il
testo verbatim, non "che libro è questo".

Stesso HF_TOKEN già usato da detect_sam3.py (SAM3) e extract_text_llama.py
(Llama-4-Scout): se Qwen3-VL-30B-A3B-Instruct richiede l'accettazione di
una licenza su Hugging Face, va fatta con lo stesso account associato al
token prima che le chiamate funzionino.

Per usare questo modello al posto di Llama-4-Scout nella pipeline, cambia
l'import in extract_cover.py/extract_isbn.py:
    from extract_text_qwen import extract_text as _extract_text

Uso diretto:
    from extract_text_qwen import extract_text
    text = extract_text("crops/book_0_spine.png")
"""

import base64
import mimetypes
import os
import time
from pathlib import Path

import requests
from dotenv import load_dotenv

load_dotenv(Path(__file__).parent.parent / ".env")

API_URL = "https://router.huggingface.co/v1/chat/completions"
MODEL = "Qwen/Qwen3-VL-235B-A22B-Instruct"

PROMPT = (
    "Trascrivi verbatim tutto il testo visibile in questa immagine (costa, "
    "copertina o retro di un libro). Riporta solo il testo letto, una riga "
    "per elemento, senza commenti né descrizioni aggiuntive. Se non c'è "
    "testo leggibile, rispondi con una stringa vuota."
)

# Cache per percorso immagine: identify_book() può richiamare lo stesso crop
# più volte (es. rilanci della pipeline) senza rifare la chiamata al modello.
_cache: dict[str, str] = {}


def _image_to_data_url(image_path: str) -> str:
    mime = mimetypes.guess_type(image_path)[0] or "image/png"
    b64 = base64.b64encode(Path(image_path).read_bytes()).decode("ascii")
    return f"data:{mime};base64,{b64}"


def extract_text(image_path: str) -> str:
    """Estrae il testo visibile in image_path con Qwen3-VL-30B-A3B-Instruct."""
    if image_path in _cache:
        return _cache[image_path]

    hf_token = os.environ.get("HF_TOKEN")
    if not hf_token:
        raise RuntimeError(
            "HF_TOKEN mancante in .env: richiesto per Qwen3-VL-30B-A3B-Instruct "
            "(stesso token già usato da detect_sam3.py per SAM3)."
        )

    payload = {
        "model": MODEL,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": PROMPT},
                {"type": "image_url", "image_url": {"url": _image_to_data_url(image_path)}},
            ],
        }],
        "max_tokens": 300,
        "temperature": 0.0,
    }
    headers = {"Authorization": f"Bearer {hf_token}"}

    text = ""
    try:
        resp = None
        for attempt in range(3):
            resp = requests.post(API_URL, headers=headers, json=payload, timeout=60)
            if resp.status_code < 500:
                break
            time.sleep(1.5 * (attempt + 1))  # errore transitorio (5xx): riprova con backoff
        resp.raise_for_status()
        text = resp.json()["choices"][0]["message"]["content"].strip()
    except requests.RequestException as exc:
        print(f"  Qwen3-VL non disponibile ({exc}): nessun testo estratto da {image_path}.")

    _cache[image_path] = text
    return text


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python extract_text_qwen.py <image_path>")
        sys.exit(1)

    print(extract_text(sys.argv[1]))
