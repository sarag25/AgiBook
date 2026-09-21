"""
Extracts the visible text of a book crop with a multimodal model (Qwen3-VL, see MODEL) through the Hugging Face Inference API.

Only the raw text extraction: the book itself is identified by extract_isbn.py (Google Books search, ISBN regex),
so the model is asked to transcribe the text verbatim, not to say which book it is.
Needs HF_TOKEN in .env; if the model requires accepting a license on Hugging Face, do it with the account tied to the token.
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

# cache by image path: the same crop can be requested again (pipeline reruns) without calling the model twice
_cache: dict[str, str] = {}


def _image_to_data_url(image_path: str) -> str:
    """
    Image file as a base64 data URL, the format the chat completions API takes
    """
    mime = mimetypes.guess_type(image_path)[0] or "image/png"
    b64 = base64.b64encode(Path(image_path).read_bytes()).decode("ascii")
    return f"data:{mime};base64,{b64}"


def extract_text(image_path: str) -> str:
    """
    Visible text of image_path as transcribed by MODEL ("" if the API is unavailable)
    """
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
            time.sleep(1.5 * (attempt + 1))  # transient error (5xx): retry with backoff
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
