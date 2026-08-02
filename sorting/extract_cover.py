import os
import re
import sys
from pathlib import Path
from urllib.parse import quote

import requests
from dotenv import load_dotenv

from extract_text_qwen import extract_text as _extract_text

# La console di Windows usa cp1252 di default: l'OCR a volte "legge" caratteri
# CJK spuri su ritagli rumorosi, e stamparli manda in crash lo script.
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    sys.stdout.reconfigure(encoding="utf-8")

load_dotenv(Path(__file__).parent.parent / ".env")

KNOWN_PUBLISHERS = {"pickwick", "mondadori", "feltrinelli", "einaudi", "rizzoli",
                    "garzanti", "longanesi", "adelphi", "bompiani", "sellerio"}


def ocr_text(image_path: str) -> str:
    """Estrazione testo dalla costa/copertina, via extract_text_qwen
    (Qwen3-VL-30B-A3B-Instruct su HF Inference API: vedi quel modulo per i dettagli)."""
    return _extract_text(image_path)


def _is_name_word(w: str) -> bool:
    return w.replace("-", "").isalpha() and len(w) > 1

def _parse_author(words: list[str]) -> tuple[str, list[str]]:
    # Normalizza a titolo per confronto
    normalized = [w.title() for w in words]
    for i in range(len(normalized) - 1):
        if _is_name_word(normalized[i]) and _is_name_word(normalized[i + 1]):
            author = f"{normalized[i]} {normalized[i+1]}"
            rest = words[:i] + words[i+2:]
            return author, rest
    return "", words


class GoogleBooksUnavailable(Exception):
    """Google Books ha risposto con errore anche dopo i retry sulla singola
    query. Segnala a search_google_books() di fermarsi subito invece di
    provare tutte le varianti di fallback (altrimenti, con un OCR lungo e
    _fetch_by_single_words che interroga una parola alla volta, un'unica
    interruzione del servizio genera decine di richieste tutte destinate a
    fallire allo stesso modo — la pipeline sembra bloccata per minuti su un
    solo libro prima di arrivare comunque al segnale "ruota il libro")."""


def _fetch(query: str, api_key: str) -> list[dict]:
    import time

    url = (
        f"https://www.googleapis.com/books/v1/volumes"
        f"?q={quote(query)}&maxResults=5"
        + (f"&key={api_key}" if api_key else "")
    )
    for attempt in range(3):
        resp = requests.get(url, timeout=10)
        if resp.status_code < 500:
            break
        time.sleep(1.5 * (attempt + 1))  # errore transitorio (5xx): riprova con backoff
    try:
        resp.raise_for_status()
    except requests.HTTPError as exc:
        raise GoogleBooksUnavailable(str(exc)) from exc
    return resp.json().get("items", [])


def _best_item(items: list[dict], context_words: list[str] = ()) -> dict | None:
    skip = ("analisi", "guida", "studio", "analysis", "guide", "summary")
    candidates = [
        item for item in items
        if not any(w in item["volumeInfo"].get("title", "").lower() for w in skip)
    ] or items
    if not candidates:
        return None

    # Punteggio = quante parole del testo OCR compaiono nel titolo, meno quante
    # parole ha il titolo che l'OCR non ha letto affatto. Senza la penalità,
    # un titolo "Hunger Games - Il canto della rivolta" batterebbe sempre "The
    # Hunger Games" per una query come "HUNGER GAMES" (entrambi contengono le
    # stesse 2 parole, ma il secondo è la corrispondenza esatta): la penalità
    # premia il titolo più aderente al testo letto, non quello con più parole.
    significant = {w.lower() for w in context_words if len(w) > 2}

    def score(item: dict) -> int:
        title = item["volumeInfo"].get("title", "").lower()
        matched = sum(1 for w in significant if w in title)
        title_words = re.findall(r"\w+", title)
        unmatched = sum(1 for w in title_words if w not in significant)
        return matched - unmatched

    return max(candidates, key=score)["volumeInfo"]


def _fetch_by_single_words(author: str, rest: list[str], api_key: str) -> list[dict]:
    """Fallback quando la query con tutte le parole non trova nulla: con l'OCR
    rumoroso basta una parola inesistente per azzerare la ricerca (Google Books
    usa AND implicito tra i termini). Si prova quindi una parola alla volta,
    raccogliendo tutti i candidati da ri-valutare con _best_item."""
    import time

    pool: dict[str, dict] = {}
    for word in rest:
        if len(word) <= 2:
            continue
        time.sleep(0.3)
        try:
            items = _fetch(f'inauthor:"{author}" {word}', api_key)
        except GoogleBooksUnavailable:
            raise  # servizio giù: inutile ripetere la stessa query parola per parola
        except Exception:
            continue
        for item in items:
            pool[item["id"]] = item
    return list(pool.values())


def search_google_books(query: str) -> dict | None:
    api_key = os.environ.get("GOOGLE_BOOKS_API_KEY", "")

    words = query.split()
    filtered = [w for w in words if w.lower() not in KNOWN_PUBLISHERS]
    author, rest = _parse_author(filtered)

    try:
        items = _fetch(" ".join(filtered), api_key)
        if not items and author:
            items = _fetch(f'inauthor:"{author}" {" ".join(rest)}', api_key)
        if not items and author:
            items = _fetch_by_single_words(author, rest, api_key)
        if not items and author:
            items = _fetch(f'inauthor:"{author}"', api_key)
    except GoogleBooksUnavailable as exc:
        print(f"  Google Books non disponibile ({exc}): salto le altre varianti di ricerca per questo libro.")
        return None

    info = _best_item(items, filtered) if items else None
    if not info:
        return None

    return {
        "Titolo":  info.get("title", "N/A"),
        "Autori":  ", ".join(info.get("authors", ["N/A"])),
        "Editore": info.get("publisher", "N/A"),
        "Anno":    info.get("publishedDate", "N/A")[:4],
        "Lingua":  info.get("language", "N/A"),
        "ISBN-13": next(
            (i["identifier"] for i in info.get("industryIdentifiers", []) if i["type"] == "ISBN_13"),
            "N/A"
        ),
    }


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python extract_cover.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]

    if not Path(image_path).exists():
        print(f"Error: file '{image_path}' not found.")
        sys.exit(1)

    print("Estrazione testo dalla copertina...")
    text = ocr_text(image_path)
    print(f"Testo rilevato: {text}\n")

    if not text.strip():
        print("Nessun testo rilevato nell'immagine.")
        sys.exit(0)

    # Controlla se c'è abbastanza testo per identificare il libro
    words = text.split()
    filtered = [w for w in words if w.lower() not in KNOWN_PUBLISHERS]
    author, rest = _parse_author(filtered)
    if author and not rest:
        print(f"Testo insufficiente: rilevato solo l'autore ({author}).")
        print("Impossibile determinare il titolo. Mostra la copertina del libro.")
        sys.exit(0)

    print("Ricerca su Google Books...")
    try:
        result = search_google_books(text)
    except Exception as e:
        print(f"Errore durante la ricerca ({e}).")
        result = None

    if result:
        for key, value in result.items():
            print(f"  {key+':':<10} {value}")
    else:
        print("Non riesco a rilevare il libro quindi dovrò ruotarlo per vedere meglio.")
