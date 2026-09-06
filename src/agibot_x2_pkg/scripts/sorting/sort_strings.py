"""
Logica di ordinamento stringhe CONDIVISA fra le due pipeline (2026-09-06,
TODO "Unire logica riconoscimento libri e riordinamento stringhe"):

  - pipeline ROS  : sorting/sort_planner.py (criteri TITLE/AUTHOR) - i
    titoli/autori arrivano dal riconoscimento (OCR sulla foto della
    shelf_camera, o ri-fotografia dal tavolo)
  - pipeline standalone: <repo>/sorting/sort_books.py e sort_strings.py,
    che ora importano da qui (unica implementazione)

Semantica (quella di sort_books.py, la piu' completa): ordinamento
alfabetico case-insensitive; i valori mancanti ("" o "N/A", cioe' libri non
ancora identificati) vanno SEMPRE in coda, sia in ordine crescente che
decrescente, cosi' non sporcano l'ordine di chi e' gia' stato letto.
"""

from __future__ import annotations


def normalized_key(value: str | None) -> tuple[bool, str]:
    """(mancante, valore normalizzato): tupla ordinabile che mette i
    mancanti in coda e confronta il resto case-insensitive."""
    value = (value or "").strip()
    missing = value == "" or value == "N/A"
    return (missing, value.lower())


def sort_strings(strings: list[str]) -> list[str]:
    """Ordinamento alfabetico case-insensitive di una lista di stringhe."""
    return sorted(strings, key=str.lower)


def sort_by_field(items: list, get_value, ascending: bool = True) -> list:
    """
    Ordina `items` per il valore restituito da get_value(item) con la
    semantica condivisa: presenti ordinati (crescente o decrescente),
    mancanti sempre in coda.
    """
    present = [i for i in items if not normalized_key(get_value(i))[0]]
    missing = [i for i in items if normalized_key(get_value(i))[0]]
    present.sort(key=lambda i: normalized_key(get_value(i))[1],
                 reverse=not ascending)
    return present + missing
