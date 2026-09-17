import logging
import os
import re
import sys
from pathlib import Path
from PIL import Image
from pyzbar.pyzbar import decode
import isbnlib
from dotenv import load_dotenv

from extract_text_qwen import extract_text as _extract_text

logging.getLogger("isbnlib").setLevel(logging.CRITICAL)

# La console di Windows usa cp1252 di default: l'OCR a volte "legge" caratteri
# CJK spuri su ritagli rumorosi, e stamparli manda in crash lo script.
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    sys.stdout.reconfigure(encoding="utf-8")

load_dotenv(Path(__file__).parent.parent / ".env")

# Wikimedia rifiuta le richieste senza uno User-Agent descrittivo (bot policy)
WIKIMEDIA_HEADERS = {
    "User-Agent": "SmartRoboticsBookSorter/1.0 (https://github.com/sarag25/SmartRobotics)"
}


ISBN_PATTERN = re.compile(
    r'(?:ISBN[:\s-]*)?'
    r'((?:97[89])[\s-]?\d{1,5}[\s-]?\d+[\s-]?\d+[\s-]?\d|'
    r'\d{9}[\dX])',
    re.IGNORECASE
)


def extract_isbns_from_barcode(image_path: str) -> list[str]:
    """Codici a barre -> ISBN. Prova l'immagine com'e', poi in scala di grigi
    ingrandita x2 e x3 con OpenCV (cubica) e con il contrasto aumentato
    (2026-09-17: nella foto dalla camera della testa del robot, libro a
    28 cm, il codice EAN-13 era largo ~130 px e pyzbar lo leggeva solo
    dopo l'ingrandimento x2 con OpenCV - con PIL no)."""
    import cv2
    import numpy as np

    isbns = []

    def _collect(barcodes):
        for barcode in barcodes:
            data = barcode.data.decode("utf-8")
            digits = re.sub(r'[- ]', '', data)
            if len(digits) in (10, 13) and digits.startswith(('978', '979', '97')):
                isbns.append(digits)

    _collect(decode(Image.open(image_path)))
    if not isbns:
        gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if gray is not None:
            variants = [gray, cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)]
            for scale in (2, 3):
                for v in variants:
                    big = cv2.resize(v, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
                    _collect(decode(big))
                    if isbns:
                        break
                if isbns:
                    break
    return list(dict.fromkeys(isbns))


def extract_isbns_from_ocr(image_path: str) -> list[str]:
    """Estrazione ISBN dal retro via extract_text_qwen (Qwen3-VL-30B-A3B-Instruct),
    poi regex ISBN_PATTERN sul testo trascritto (invariata)."""
    full_text = _extract_text(image_path)

    candidates = ISBN_PATTERN.findall(full_text)
    isbns = []
    for candidate in candidates:
        digits = re.sub(r'[- ]', '', candidate)
        if len(digits) in (10, 13):
            isbns.append(digits)
    return list(dict.fromkeys(isbns))


def get_openlibrary_edition_meta(isbn: str) -> dict | None:
    try:
        return isbnlib.meta(isbn, service="openl")
    except Exception:
        return None


def get_google_books_meta(isbn: str) -> dict | None:
    import time
    import requests

    api_key = os.environ.get("GOOGLE_BOOKS_API_KEY", "")
    url = (
        f"https://www.googleapis.com/books/v1/volumes"
        f"?q=isbn:{isbn}&maxResults=1"
        + (f"&key={api_key}" if api_key else "")
    )
    items = []
    for attempt in range(3):      # 429/errore transitorio: riprova (2026-09-17)
        try:
            time.sleep(0.5)
            resp = requests.get(url, timeout=10)
            if resp.status_code == 429 or resp.status_code >= 500:
                time.sleep(5.0 * (attempt + 1))
                continue
            resp.raise_for_status()
            items = resp.json().get("items", [])
            break
        except Exception:
            time.sleep(2.0)
    try:
        if not items:
            return None
        info = items[0]["volumeInfo"]
        return {
            "ISBN-13": isbn,
            "Title": info.get("title", ""),
            "Authors": info.get("authors", []),
            "Publisher": info.get("publisher", ""),
            "Year": (info.get("publishedDate", "")[:4]),
            "Language": info.get("language", ""),
        }
    except Exception:
        return None


def get_openlibrary_first_publish_year(isbn: str) -> str | None:
    """Anno della prima pubblicazione dell'opera, non della singola edizione."""
    import requests

    try:
        resp = requests.get(f"https://openlibrary.org/isbn/{isbn}.json", timeout=10)
        resp.raise_for_status()
        works = resp.json().get("works", [])
        if not works:
            return None
        work_key = works[0]["key"]

        resp = requests.get(f"https://openlibrary.org{work_key}.json", timeout=10)
        resp.raise_for_status()
        first_publish_date = resp.json().get("first_publish_date", "")
        match = re.search(r"\d{4}", first_publish_date)
        return match.group(0) if match else None
    except Exception:
        return None


def get_openlibrary_first_publish_year_by_title(title: str, author: str = "") -> str | None:
    """Fallback quando l'edizione specifica non è catalogata su OpenLibrary:
    cerca l'opera per titolo/autore, dato che la Search API espone first_publish_year
    direttamente sui risultati."""
    import requests

    if not title:
        return None
    try:
        # L'autore non viene filtrato come frase esatta: fonti diverse abbreviano
        # i nomi in modo diverso (es. "J. Wolfgang Goethe" vs "Johann Wolfgang von
        # Goethe"), e un match esatto azzererebbe i risultati.
        resp = requests.get(
            "https://openlibrary.org/search.json",
            params={"q": f'title:"{title}"', "fields": "first_publish_year", "limit": 1},
            timeout=10,
        )
        resp.raise_for_status()
        docs = resp.json().get("docs", [])
        if not docs:
            return None
        year = docs[0].get("first_publish_year")
        return str(year) if year else None
    except Exception:
        return None


def get_wikidata_first_publish_year(title: str, author: str = "") -> str | None:
    """Fallback tramite Wikidata: il campo P577 (data di pubblicazione) è curato
    editorialmente ed è spesso più preciso della ricerca full-text di OpenLibrary."""
    import time
    import requests

    if not title:
        return None
    try:
        time.sleep(0.3)
        resp = requests.get(
            "https://www.wikidata.org/w/api.php",
            params={
                "action": "wbsearchentities",
                "search": title,
                "language": "en",
                "format": "json",
                "limit": 5,
                "type": "item",
            },
            headers=WIKIMEDIA_HEADERS,
            timeout=10,
        )
        resp.raise_for_status()
        candidates = resp.json().get("search", [])

        author_surname = author.split()[-1].lower() if author else ""
        # Se conosciamo l'autore, scarta i candidati la cui descrizione non lo menziona
        # (utile a evitare di scambiare il romanzo con un film/adattamento omonimo)
        if author_surname:
            candidates = [
                c for c in candidates
                if author_surname in c.get("description", "").lower()
            ] or candidates

        for candidate in candidates:
            entity_id = candidate["id"]
            resp = requests.get(
                f"https://www.wikidata.org/wiki/Special:EntityData/{entity_id}.json",
                headers=WIKIMEDIA_HEADERS,
                timeout=10,
            )
            resp.raise_for_status()
            claims = resp.json()["entities"][entity_id].get("claims", {}).get("P577", [])
            for claim in claims:
                time_value = claim.get("mainsnak", {}).get("datavalue", {}).get("value", {}).get("time", "")
                match = re.search(r"[+-](\d{4})", time_value)
                if match:
                    return match.group(1)
        return None
    except Exception:
        return None


def get_wikipedia_first_publish_year(title: str, author: str = "", lang: str = "it") -> str | None:
    """Fallback: legge l'infobox della pagina Wikipedia del libro (campo
    'ed. originale' / 'publication date'), che spesso riporta l'anno di
    prima pubblicazione anche quando Wikidata o OpenLibrary non lo hanno."""
    import time
    import requests
    from bs4 import BeautifulSoup

    if not title:
        return None
    try:
        time.sleep(0.3)
        search_query = f"{title} {author}".strip()
        resp = requests.get(
            f"https://{lang}.wikipedia.org/w/api.php",
            params={
                "action": "query", "list": "search", "srsearch": search_query,
                "format": "json", "srlimit": 1,
            },
            headers=WIKIMEDIA_HEADERS,
            timeout=10,
        )
        resp.raise_for_status()
        results = resp.json().get("query", {}).get("search", [])
        if not results:
            return None
        page_title = results[0]["title"]

        resp = requests.get(
            f"https://{lang}.wikipedia.org/w/api.php",
            params={
                "action": "parse", "page": page_title, "prop": "text",
                "format": "json", "section": 0,
            },
            headers=WIKIMEDIA_HEADERS,
            timeout=10,
        )
        resp.raise_for_status()
        html = resp.json().get("parse", {}).get("text", {}).get("*", "")
        if not html:
            return None

        soup = BeautifulSoup(html, "html.parser")
        infobox = soup.find("table", class_=re.compile("infobox"))
        if not infobox:
            return None

        field_keywords = (
            "ed. originale", "edizione originale", "pubblicazione originale",
            "publication date", "published", "pub_date",
        )
        for row in infobox.find_all("tr"):
            th, td = row.find("th"), row.find("td")
            if not th or not td:
                continue
            label = th.get_text(" ", strip=True).lower()
            if any(keyword in label for keyword in field_keywords):
                match = re.search(r"\d{4}", td.get_text())
                if match:
                    return match.group(0)
        return None
    except Exception:
        return None


LAST_ERROR = ""      # ultimo errore di rete/quota di search_book_by_title (diagnostica)


def search_book_by_title(title: str, author: str = "") -> dict | None:
    """Metadati dal TITOLO (e autore, se noto) letti sul dorso con l'OCR
    (2026-09-13, pipeline automatica): Google Books volumes API con
    intitle:/inauthor:. Stesse chiavi di get_book_info (ISBN-13, Title,
    Authors, Year, OriginalYear). None se nessun risultato o niente rete."""
    import re
    import requests
    import time as _time

    title = (title or "").strip()
    # titoli corti esistono ("IT"): con l'autore bastano 2 caratteri
    if len(title) < (2 if author and author.strip() else 3):
        return None
    q = f'intitle:"{title}"'
    if author and len(author.strip()) >= 3:
        q += f' inauthor:"{author.strip()}"'
    google_ok = True
    api_key = os.environ.get("GOOGLE_BOOKS_API_KEY", "")
    key_param = {"key": api_key} if api_key else {}

    def _google(params):
        """GET con un secondo tentativo dopo 3 s su 429/5xx (limite al
        minuto: con 15 libri di fila la prova del 2026-09-17 ha perso
        Cat's Cradle). Ritorna (items, ok)."""
        global LAST_ERROR
        for attempt in range(3):
            try:
                resp = requests.get("https://www.googleapis.com/books/v1/volumes",
                                    params={**params, "maxResults": 3, "printType": "books", **key_param},
                                    timeout=10)
                if resp.status_code == 429 or resp.status_code >= 500:
                    LAST_ERROR = f"Google Books HTTP {resp.status_code}"
                    if attempt < 2:
                        _time.sleep(5.0 * (attempt + 1))
                        continue
                    return [], False
                resp.raise_for_status()
                LAST_ERROR = ""
                return resp.json().get("items", []) or [], True
            except Exception as e:
                LAST_ERROR = f"Google Books: {e}"
                return [], False
        return [], False

    # niente rete o quota esaurita: non si torna None qui, si passa ai
    # ripieghi (2026-09-17: prima usciva subito e OpenLibrary non veniva
    # mai interrogata)
    items, google_ok = _google({"q": q})
    if not items and author and google_ok:
        return search_book_by_title(title, "")     # riprova senza autore
    if not items:
        # OCR dai dorsi visti di sbieco ("HUnGER", "SIEPHEN KING"): ricerca a
        # testo libero con le sole parole di >= 4 lettere
        import re
        words = [w for w in re.findall(r"[A-Za-zÀ-ÿ']+", title) if len(w) >= 4]
        # (2026-09-17: prima si saltava se le parole coincidevano con il
        # titolo, ma la prima query era intitle:"..." e questa e' a testo
        # libero: sono ricerche diverse, va fatta sempre)
        if google_ok and words:
            items, google_ok = _google({"q": " ".join(words)})
    # Coerenza con l'OCR (2026-09-17): fra i risultati si prende il PRIMO il
    # cui titolo compare davvero nel testo letto; con "STEPHEN KING" da solo
    # nessuno passa e si torna None (meglio nessuna identificazione che
    # "I segreti di Stephen King": il libro andra' all'ISBN).
    ocr_text = f"{title} {author}"
    if items and not author and ocr_is_only_author(
            title, [it.get("volumeInfo", {}).get("authors", []) for it in items]):
        return None          # sul dorso c'e' solo il nome dell'autore: ambiguo
    all_aw = set()
    for it in items:
        for a in it.get("volumeInfo", {}).get("authors", []) or []:
            all_aw |= {w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ']+", a) if len(w) >= 4}
    items = [it for it in items if title_matches_ocr(
        {"Title": it.get("volumeInfo", {}).get("title", ""),
         "Authors": it.get("volumeInfo", {}).get("authors", []) or []}, ocr_text, all_author_words=all_aw)]
    if not items:
        # Google Books senza chiave API ha una quota giornaliera condivisa
        # (HTTP 429 "Quota exceeded", visto il 2026-09-17): ripiego su
        # OpenLibrary, senza quota.
        return search_book_openlibrary(title, author)
    info = items[0].get("volumeInfo", {})
    isbn13 = next((i.get("identifier") for i in info.get("industryIdentifiers", [])
                   if i.get("type") == "ISBN_13"), "")
    authors = info.get("authors", []) or []
    year = (info.get("publishedDate") or "")[:4]
    merged = {
        "ISBN-13": isbn13,
        "Title": info.get("title", ""),
        "Authors": authors,
        "Publisher": info.get("publisher", ""),
        "Year": year,
        "Language": info.get("language", ""),
    }
    a0 = authors[0] if authors else ""
    original = (
        (get_openlibrary_first_publish_year(isbn13) if isbn13 else None)
        or get_wikidata_first_publish_year(merged["Title"], a0)
        or get_openlibrary_first_publish_year_by_title(merged["Title"], a0)
    )
    merged["OriginalYear"] = original or year
    return merged


def title_matches_ocr(meta: dict, ocr_text: str, min_ratio: float = 0.5, all_author_words=None) -> bool:
    """Il risultato della ricerca e' coerente con il testo OCR? Serve
    perche' con testo OCR povero ("STEPHEN KING", "SuzannE Collins HUNGER")
    la ricerca a testo libero torna un libro qualsiasi ("I segreti di
    Stephen King", "The Panem Companion") (2026-09-17). Regola: almeno
    min_ratio delle parole significative del titolo trovato (>= 4 lettere,
    escluse quelle del nome dell'autore) deve comparire, anche con errori
    OCR (difflib >= 0.75), nel testo OCR."""
    import difflib
    import re

    def words(txt):
        return [w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ']+", txt or "") if len(w) >= 4]

    author_words = set()
    for a in (meta.get("Authors") or []):
        author_words |= set(words(a))
    title_words = [w for w in words(meta.get("Title", "")) if w not in author_words]
    ocr_words = words(ocr_text)
    if not title_words:
        # titolo corto ("It"): deve comparire tale e quale fra le parole OCR
        short = [w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ']+", meta.get("Title", ""))]
        ocr_all = [w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ']+", ocr_text or "")]
        return bool(short) and all(w in ocr_all for w in short)
    hits = 0
    for tw in title_words:
        if any(difflib.SequenceMatcher(None, tw, ow).ratio() >= 0.75 for ow in ocr_words):
            hits += 1
    ratio = hits / len(title_words)
    if ratio < min_ratio:
        return False
    # Titolo che e' solo un nome d'autore (biografia "Suzanne Collins" di
    # E. Hoover): tutte le parole del titolo sono nomi d'autore di altri
    # risultati -> no (2026-09-17, prova con 15 libri).
    if all_author_words and all(w in all_author_words for w in title_words):
        return False
    # Titolo solo parzialmente letto ("TERRA VOLUME" -> "La decima terra -
    # Volume 1"): si accetta solo se anche l'AUTORE compare nel testo OCR.
    if ratio < 1.0:
        aw = [w for w in author_words if len(w) >= 4]
        if not aw or not any(difflib.SequenceMatcher(None, a, ow).ratio() >= 0.8
                             for a in aw for ow in ocr_words):
            return False
    return True


def ocr_is_only_author(ocr_text: str, results_authors) -> bool:
    """True se TUTTE le parole significative dell'OCR sono parti di nomi
    d'autore comparsi nei risultati ("STEPHEN KING" da solo): la ricerca
    troverebbe un libro qualsiasi di/su quell'autore. results_authors:
    lista di liste di nomi."""
    import difflib
    import re
    aw = set()
    for names in results_authors:
        for a in names or []:
            aw |= {w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ']+", a) if len(w) >= 3}
    ow = [w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ']+", ocr_text or "") if len(w) >= 4]
    if not ow or not aw:
        return False
    return all(any(difflib.SequenceMatcher(None, o, a).ratio() >= 0.8 for a in aw) for o in ow)


def search_book_openlibrary(title: str, author: str = "") -> dict | None:
    """Ricerca per titolo (e autore) su OpenLibrary search.json - stesse
    chiavi di search_book_by_title, piu' "Source": "openlibrary". Usata come
    ripiego quando Google Books non risponde (quota) o non trova nulla.
    Testo OCR: si cercano le sole parole di >= 3 lettere."""
    import re
    import requests

    words = [w for w in re.findall(r"[A-Za-zÀ-ÿ']+", title or "") if len(w) >= 3]
    if not words and (title or "").strip():
        words = [(title or "").strip()]        # titolo corto ("IT")
    if not words:
        return None
    params = {"q": " ".join(words), "limit": 3,
              "fields": "title,author_name,first_publish_year,isbn"}
    if author and len(author.strip()) >= 3:
        params["author"] = author.strip()
    try:
        resp = requests.get("https://openlibrary.org/search.json", params=params, timeout=10)
        resp.raise_for_status()
        docs = resp.json().get("docs", []) or []
    except Exception:
        return None
    if not docs and author:
        return search_book_openlibrary(title, "")
    if docs and not author and ocr_is_only_author(title, [d.get("author_name", []) for d in docs]):
        return None
    all_aw = set()
    for d in docs:
        for a in d.get("author_name", []) or []:
            all_aw |= {w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ']+", a) if len(w) >= 4}
    docs = [d for d in docs if title_matches_ocr(
        {"Title": d.get("title", ""), "Authors": d.get("author_name", []) or []}, f"{title} {author}",
        all_author_words=all_aw)]
    if not docs:
        return None
    d = docs[0]
    isbns = [i for i in (d.get("isbn") or []) if len(i) == 13 and i.startswith(("978", "979"))]
    year = str(d.get("first_publish_year") or "")
    return {
        "ISBN-13": isbns[0] if isbns else "",
        "Title": d.get("title", ""),
        "Authors": d.get("author_name", []) or [],
        "Publisher": "",
        "Year": year,
        "Language": "",
        "OriginalYear": year,
        "Source": "openlibrary",
    }


def get_book_info(isbn: str) -> dict | None:
    ol_meta = get_openlibrary_edition_meta(isbn) or {}
    gb_meta = get_google_books_meta(isbn) or {}

    if not ol_meta and not gb_meta:
        return None

    # Merge: preferisce OpenLibrary, riempie i buchi con Google Books
    merged = {
        "ISBN-13": isbn,
        "Title": ol_meta.get("Title") or gb_meta.get("Title", ""),
        "Authors": ol_meta.get("Authors") or gb_meta.get("Authors", []),
        "Publisher": ol_meta.get("Publisher") or gb_meta.get("Publisher", ""),
        "Year": ol_meta.get("Year") or gb_meta.get("Year", ""),  # anno di pubblicazione di questa edizione
        "Language": ol_meta.get("Language") or gb_meta.get("Language", ""),
    }

    # Anno in cui il libro è stato scritto/pubblicato per la prima volta (opera originale)
    author = merged["Authors"][0] if merged["Authors"] else ""
    original_year = (
        get_openlibrary_first_publish_year(isbn)
        or get_wikidata_first_publish_year(merged["Title"], author)
        or get_wikipedia_first_publish_year(merged["Title"], author)
        or get_openlibrary_first_publish_year_by_title(merged["Title"], author)
    )
    merged["OriginalYear"] = original_year or merged["Year"]

    return merged


ISBN_GROUP_LANG = {
    "0": "en", "1": "en",
    "2": "fr",
    "3": "de",
    "4": "ja",
    "5": "ru",
    "7": "zh",
    "88": "it", "978-88": "it",
    "84": "es",
    "85": "pt",
    "86": "sr",
    "87": "da",
    "89": "ko",
    "90": "nl", "91": "sv", "92": "int", "93": "hi",
}


def get_language(meta: dict, isbn: str) -> str:
    lang = meta.get('Language', '')
    if lang:
        return lang
    # Ricava la lingua dal gruppo ISBN (cifre dopo 978/979)
    suffix = isbn[3:] if isbn.startswith(("978", "979")) else isbn
    for prefix in sorted(ISBN_GROUP_LANG, key=len, reverse=True):
        if suffix.startswith(prefix.replace("978-", "")):
            return ISBN_GROUP_LANG[prefix]
    return 'N/A'


def print_book_info(meta: dict, isbn: str) -> None:
    print(f"  Titolo:   {meta.get('Title', 'N/A')}")
    print(f"  Autori:   {', '.join(meta.get('Authors', ['N/A']))}")
    print(f"  Editore:  {meta.get('Publisher', 'N/A')}")
    print(f"  Anno edizione:      {meta.get('Year', 'N/A')}")
    print(f"  Anno prima edizione: {meta.get('OriginalYear', 'N/A')}")
    print(f"  Lingua:   {get_language(meta, isbn)}")
    print(f"  ISBN-13:  {meta.get('ISBN-13', 'N/A')}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python extract_isbn.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]

    if not Path(image_path).exists():
        print(f"Error: file '{image_path}' not found.")
        sys.exit(1)

    # Prima prova con il barcode (più affidabile)
    print("Ricerca barcode...")
    isbns = extract_isbns_from_barcode(image_path)

    if isbns:
        print(f"ISBN trovati via barcode: {', '.join(isbns)}")
    else:
        # Fallback: OCR sul testo
        print("Nessun barcode trovato, provo con OCR...")
        isbns = extract_isbns_from_ocr(image_path)
        if isbns:
            print(f"ISBN trovati via OCR: {', '.join(isbns)}")

    if not isbns:
        print("Nessun ISBN trovato.")
        sys.exit(0)

    print()
    for isbn in isbns:
        print(f"--- {isbn} ---")
        meta = get_book_info(isbn)
        if meta:
            print_book_info(meta, isbn)
        else:
            lang = get_language({}, isbn)
            print(f"  Metadati online non disponibili.")
            print(f"  Lingua stimata: {lang}")
            print(f"  ISBN-13:  {isbn}")
        print()
