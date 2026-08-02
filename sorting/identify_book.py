import sys
from pathlib import Path

from extract_cover import ocr_text, search_google_books, _parse_author, KNOWN_PUBLISHERS
from extract_isbn import (
    extract_isbns_from_barcode,
    extract_isbns_from_ocr,
    get_book_info,
    get_language,
    print_book_info,
)


def find_image(prefix: Path, suffix: str) -> Path | None:
    matches = sorted(prefix.parent.glob(f"{prefix.name}_{suffix}.*"))
    return matches[0] if matches else None


def has_enough_text(text: str) -> bool:
    """La costa/copertina spesso mostra solo il nome dell'autore: non basta a
    cercare il libro, meglio far ruotare subito il libro."""
    words = [w for w in text.split() if w.lower() not in KNOWN_PUBLISHERS]
    author, rest = _parse_author(words)
    return bool(text.strip()) and not (author and not rest)


def search_from_image(image_path: Path, label: str) -> dict | None:
    print(f"Estrazione testo dalla {label}...")
    text = ocr_text(str(image_path))
    print(f"Testo rilevato: {text}\n")

    if not has_enough_text(text):
        return None

    print(f"Ricerca su Google Books ({label})...")
    return search_google_books(text)


def print_cover_result(result: dict) -> None:
    for key, value in result.items():
        print(f"  {key + ':':<10} {value}")


def search_from_retro(image_path: Path) -> dict:
    print("Ricerca barcode sul retro...")
    isbns = extract_isbns_from_barcode(str(image_path))
    if isbns:
        print(f"ISBN trovati via barcode: {', '.join(isbns)}")
    else:
        print("Nessun barcode trovato, provo con OCR...")
        isbns = extract_isbns_from_ocr(str(image_path))
        if isbns:
            print(f"ISBN trovati via OCR: {', '.join(isbns)}")

    if not isbns:
        print("Nessun ISBN trovato sul retro.")
        return {"status": "not_found"}

    print()
    best_info = None
    for isbn in isbns:
        print(f"--- {isbn} ---")
        meta = get_book_info(isbn)
        if meta:
            print_book_info(meta, isbn)
            info = {
                "Titolo": meta.get("Title", "N/A"),
                "Autori": ", ".join(meta.get("Authors") or []) or "N/A",
                "Editore": meta.get("Publisher", "N/A"),
                "Anno": meta.get("Year", "N/A"),
                "Lingua": get_language(meta, isbn),
                "ISBN-13": isbn,
            }
        else:
            print("  Metadati online non disponibili.")
            print(f"  Lingua stimata: {get_language({}, isbn)}")
            print(f"  ISBN-13:  {isbn}")
            info = {
                "Titolo": "N/A", "Autori": "N/A", "Editore": "N/A",
                "Anno": "N/A", "Lingua": get_language({}, isbn), "ISBN-13": isbn,
            }
        if best_info is None:
            best_info = info
        print()

    return {"status": "found", "source": "retro", "info": best_info}


def _is_complete(info: dict) -> bool:
    """Un risultato conta come identificazione riuscita solo se ha SIA
    titolo SIA autore (non vuoti/'N/A'): un match parziale (es. solo il
    titolo, come con testo di costa ambiguo) non basta a fermare la ricerca
    qui, si prova la vista successiva (copertina, poi retro) per completare
    i dati invece di accontentarsi."""
    title = (info.get("Titolo") or "").strip()
    author = (info.get("Autori") or "").strip()
    return title not in ("", "N/A") and author not in ("", "N/A")


def identify_book(prefix: Path) -> dict:
    """
    Prova a identificare il libro cercando prima la costa, poi la copertina,
    poi il retro (ISBN). Ritorna sempre un dict:
      {"status": "found", "source": "spine"|"cover"|"retro", "info": {...}}
      {"status": "rotate", "next": "cover"|"retro"}  -- serve una nuova foto
      {"status": "not_found"}                         -- retro letto ma nessun ISBN trovato

    "found" richiede titolo E autore entrambi presenti (vedi _is_complete):
    sulla costa/copertina un match con solo uno dei due non è sufficiente,
    si prosegue verso la vista successiva. Il retro è l'ultima vista
    possibile: lì si accetta anche un risultato parziale (es. solo ISBN),
    non c'è altro da ruotare per completarlo.
    """
    spine = find_image(prefix, "spine")
    if spine:
        result = search_from_image(spine, "costa (spine)")
        if result and _is_complete(result):
            print_cover_result(result)
            return {"status": "found", "source": "spine", "info": result}
    print("Libro non identificato (o incompleto) dalla costa: ruota il libro per mostrare la copertina.\n")

    cover = find_image(prefix, "cover")
    if not cover:
        return {"status": "rotate", "next": "cover"}
    result = search_from_image(cover, "copertina")
    if result and _is_complete(result):
        print_cover_result(result)
        return {"status": "found", "source": "cover", "info": result}
    print("Libro non identificato (o incompleto) dalla copertina: ruota il libro per mostrare il retro.\n")

    retro = find_image(prefix, "retro")
    if not retro:
        return {"status": "rotate", "next": "retro"}
    return search_from_retro(retro)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python identify_book.py <cartella/prefisso_libro>")
        print(r"Esempio: python identify_book.py books\hunger_games\hunger_games_ballata")
        sys.exit(1)

    prefix = Path(sys.argv[1])
    if not prefix.parent.is_dir():
        print(f"Error: la cartella '{prefix.parent}' non esiste.")
        sys.exit(1)

    outcome = identify_book(prefix)
    if outcome["status"] == "rotate":
        print(f"Azione richiesta: ruota il libro per mostrare '{outcome['next']}' e rilancia.")
    elif outcome["status"] == "not_found":
        print("Libro non identificato: nessun ISBN leggibile sul retro.")
