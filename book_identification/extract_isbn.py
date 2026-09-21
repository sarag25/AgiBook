"""
Book metadata from an ISBN read on the book (barcode, then OCR), or from the title read on the spine.
Lookups on OpenLibrary and Google Books, with Wikidata and Wikipedia as fallbacks for the year of first publication.
  python extract_isbn.py <image_path>
"""
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

# the Windows console defaults to cp1252: on noisy crops the OCR sometimes "reads" spurious CJK characters,
# and printing them would crash the script
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    sys.stdout.reconfigure(encoding="utf-8")

load_dotenv(Path(__file__).parent.parent / ".env")

# Wikimedia rejects requests without a descriptive User-Agent (bot policy)
WIKIMEDIA_HEADERS = {
    "User-Agent": "AgiBook/1.0 (https://github.com/sarag25/AgiBook)"
}


ISBN_PATTERN = re.compile(
    r'(?:ISBN[:\s-]*)?'
    r'((?:97[89])[\s-]?\d{1,5}[\s-]?\d+[\s-]?\d+[\s-]?\d|'
    r'\d{9}[\dX])',
    re.IGNORECASE
)


def extract_isbns_from_barcode(image_path: str) -> list[str]:
    """
    Barcodes -> ISBN. Tries the image as is, then the grayscale version and a contrast-enhanced one, upscaled x2 and x3 (OpenCV, cubic).
    On the robot head camera photo (book at 28 cm) the EAN-13 is ~130 px wide and pyzbar reads it only after the x2 OpenCV upscale, not with PIL
    """
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
    """
    ISBNs from the back cover: text transcribed by extract_text_qwen, then ISBN_PATTERN on it
    """
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
    for attempt in range(3):      # 429 or transient error: retry
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
    """
    Year of first publication of the work, not of the single edition
    """
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
    """
    Fallback when the specific edition is not in OpenLibrary: searches the work by title, since the Search API
    exposes first_publish_year directly on the results
    """
    import requests

    if not title:
        return None
    try:
        # the author is not filtered as an exact phrase: sources abbreviate names differently
        # (e.g. "J. Wolfgang Goethe" vs "Johann Wolfgang von Goethe"), and an exact match would return nothing
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
    """
    Fallback through Wikidata: field P577 (publication date) is editorially curated and often more precise
    than OpenLibrary's full-text search
    """
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
        # if the author is known, drop the candidates whose description does not mention them
        # (avoids taking a film or adaptation of the same name for the novel)
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
    """
    Fallback: reads the infobox of the book's Wikipedia page ('ed. originale' / 'publication date'), which often
    gives the year of first publication even when Wikidata or OpenLibrary do not
    """
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


LAST_ERROR = ""      # last network/quota error of search_book_by_title (diagnostics)


def search_book_by_title(title: str, author: str = "") -> dict | None:
    """
    Metadata from the TITLE (and author, if known) read on the spine by OCR (automatic pipeline): Google Books
    volumes API with intitle:/inauthor:. Same keys as get_book_info (ISBN-13, Title, Authors, Year, OriginalYear).
    None if there is no result or no network
    """
    import re
    import requests
    import time as _time

    title = (title or "").strip()
    # short titles exist ("IT"): with the author 2 characters are enough
    if len(title) < (2 if author and author.strip() else 3):
        return None
    q = f'intitle:"{title}"'
    if author and len(author.strip()) >= 3:
        q += f' inauthor:"{author.strip()}"'
    google_ok = True
    api_key = os.environ.get("GOOGLE_BOOKS_API_KEY", "")
    key_param = {"key": api_key} if api_key else {}

    def _google(params):
        """
        GET retried with backoff on 429/5xx (per-minute limit: with 15 books in a row Cat's Cradle was lost).
        Returns (items, ok)
        """
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

    # no network or quota exhausted: do not return None here, go on to the fallbacks
    # (returning early meant OpenLibrary was never queried)
    items, google_ok = _google({"q": q})
    if not items and author and google_ok:
        return search_book_by_title(title, "")     # riprova senza autore
    if not items:
        # OCR on spines seen at an angle ("HUnGER", "SIEPHEN KING"): free-text search with only the words of >= 4 letters
        import re
        words = [w for w in re.findall(r"[A-Za-zÀ-ÿ']+", title) if len(w) >= 4]
        # always run it, even if the words equal the title: the first query was intitle:"..." and this one is
        # free text, two different searches
        if google_ok and words:
            items, google_ok = _google({"q": " ".join(words)})
    # consistency with the OCR: among the results take the FIRST whose title really appears in the text read;
    # with "STEPHEN KING" alone none passes and None is returned (better no identification than
    # "I segreti di Stephen King": the book goes to the ISBN path)
    ocr_text = f"{title} {author}"
    if items and not author and ocr_is_only_author(
            title, [it.get("volumeInfo", {}).get("authors", []) for it in items]):
        return None          # only the author name is on the spine: ambiguous
    all_aw = set()
    for it in items:
        for a in it.get("volumeInfo", {}).get("authors", []) or []:
            all_aw |= {w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ']+", a) if len(w) >= 4}
    items = [it for it in items if title_matches_ocr(
        {"Title": it.get("volumeInfo", {}).get("title", ""),
         "Authors": it.get("volumeInfo", {}).get("authors", []) or []}, ocr_text, all_author_words=all_aw)]
    if not items:
        # Google Books without an API key has a shared daily quota (HTTP 429 "Quota exceeded"):
        # fall back to OpenLibrary, which has none
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
    """
    Is the search result consistent with the OCR text? Needed because with poor OCR text ("STEPHEN KING",
    "SuzannE Collins HUNGER") the free-text search returns any book ("I segreti di Stephen King", "The Panem Companion").
    Rule: at least min_ratio of the significant words of the title found (>= 4 letters, author name excluded)
    must appear in the OCR text, even with OCR errors (difflib >= 0.75)
    """
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
        # short title ("It"): it must appear as is among the OCR words
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
    # title that is only an author name (biography "Suzanne Collins" by E. Hoover): all its words are
    # author names of other results -> reject
    if all_author_words and all(w in all_author_words for w in title_words):
        return False
    # title only partly read ("TERRA VOLUME" -> "La decima terra - Volume 1"):
    # accepted only if the AUTHOR also appears in the OCR text
    if ratio < 1.0:
        aw = [w for w in author_words if len(w) >= 4]
        if not aw or not any(difflib.SequenceMatcher(None, a, ow).ratio() >= 0.8
                             for a in aw for ow in ocr_words):
            return False
    return True


def ocr_is_only_author(ocr_text: str, results_authors) -> bool:
    """
    True if ALL the significant OCR words are parts of author names found in the results ("STEPHEN KING" alone):
    the search would return any book by or about that author. results_authors: list of lists of names
    """
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
    """
    Search by title (and author) on OpenLibrary search.json, same keys as search_book_by_title plus "Source": "openlibrary".
    Fallback when Google Books does not answer (quota) or finds nothing. OCR text: only words of >= 3 letters are searched
    """
    import re
    import requests

    words = [w for w in re.findall(r"[A-Za-zÀ-ÿ']+", title or "") if len(w) >= 3]
    if not words and (title or "").strip():
        words = [(title or "").strip()]        # short title ("IT")
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

    # merge: prefer OpenLibrary, fill the gaps with Google Books
    merged = {
        "ISBN-13": isbn,
        "Title": ol_meta.get("Title") or gb_meta.get("Title", ""),
        "Authors": ol_meta.get("Authors") or gb_meta.get("Authors", []),
        "Publisher": ol_meta.get("Publisher") or gb_meta.get("Publisher", ""),
        "Year": ol_meta.get("Year") or gb_meta.get("Year", ""),  # year of this edition
        "Language": ol_meta.get("Language") or gb_meta.get("Language", ""),
    }

    # year the work was first written/published (original work)
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
    # language from the ISBN group (digits after 978/979)
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

    # barcode first (more reliable)
    print("Ricerca barcode...")
    isbns = extract_isbns_from_barcode(image_path)

    if isbns:
        print(f"ISBN trovati via barcode: {', '.join(isbns)}")
    else:
        # fallback: OCR on the text
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
