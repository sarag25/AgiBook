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
    img = Image.open(image_path)
    barcodes = decode(img)
    isbns = []
    for barcode in barcodes:
        data = barcode.data.decode("utf-8")
        digits = re.sub(r'[- ]', '', data)
        if len(digits) in (10, 13) and digits.startswith(('978', '979', '97')):
            isbns.append(digits)
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
    try:
        time.sleep(0.5)
        resp = requests.get(url, timeout=10)
        resp.raise_for_status()
        items = resp.json().get("items", [])
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


def search_book_by_title(title: str, author: str = "") -> dict | None:
    """Metadati dal TITOLO (e autore, se noto) letti sul dorso con l'OCR
    (2026-09-13, pipeline automatica): Google Books volumes API con
    intitle:/inauthor:. Stesse chiavi di get_book_info (ISBN-13, Title,
    Authors, Year, OriginalYear). None se nessun risultato o niente rete."""
    import requests

    title = (title or "").strip()
    if len(title) < 3:
        return None
    q = f'intitle:"{title}"'
    if author and len(author.strip()) >= 3:
        q += f' inauthor:"{author.strip()}"'
    try:
        resp = requests.get("https://www.googleapis.com/books/v1/volumes",
                            params={"q": q, "maxResults": 3, "printType": "books"}, timeout=10)
        resp.raise_for_status()
        items = resp.json().get("items", []) or []
    except Exception:
        return None
    if not items and author:
        return search_book_by_title(title, "")     # riprova senza autore
    if not items:
        # OCR dai dorsi visti di sbieco ("HUnGER", "SIEPHEN KING"): ricerca a
        # testo libero con le sole parole di >= 4 lettere
        import re
        words = [w for w in re.findall(r"[A-Za-zÀ-ÿ']+", title) if len(w) >= 4]
        if words and " ".join(words).lower() != title.lower():
            try:
                resp = requests.get("https://www.googleapis.com/books/v1/volumes",
                                    params={"q": " ".join(words), "maxResults": 3, "printType": "books"},
                                    timeout=10)
                resp.raise_for_status()
                items = resp.json().get("items", []) or []
            except Exception:
                items = []
    if not items:
        return None
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
