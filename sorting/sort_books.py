"""
sort_books.py
=============
Ordina i libri identificati da crop_and_identify.py per titolo o per autore,
in ordine alfabetico. Lavora sul JSON prodotto da identify_all()
(sorting/identified_books.json di default), così l'ordinamento può essere
rilanciato con criteri diversi senza rifare detection/OCR/ricerche web ogni
volta (costose e soggette a rate limit).

Uso:
    python sort_books.py                   # legge identified_books.json, ordina per titolo
    python sort_books.py --by author
    python sort_books.py --by title --input altro.json
"""

import argparse
import json
import sys
from pathlib import Path

DEFAULT_INPUT = Path(__file__).parent / "identified_books.json"


def sort_books(records: list, by: str) -> list:
    """
    Ordina alfabeticamente per 'title' o 'author'. I libri non ancora
    identificati (senza titolo/autore, status "rotate"/"not_found") vanno in
    coda invece che mescolati in cima, così non sporcano l'ordine di chi è
    già stato letto correttamente.
    """
    # Logica condivisa con la pipeline ROS (2026-09-06): la stessa
    # normalizzazione/ordinamento usata da sort_planner.py - vedi
    # sort_strings.py (wrapper) e
    # src/agibot_x2_pkg/scripts/sorting/sort_strings.py (implementazione).
    from sort_strings import sort_by_field
    key = "title" if by == "title" else "author"
    return sort_by_field(records, lambda rec: rec.get(key), ascending=True)


def print_plan(records: list, by: str) -> None:
    print(f"\nPiano di ordinamento per {by}:\n")
    for i, rec in enumerate(records, start=1):
        if rec.get("status") == "found":
            print(f"  {i}. {rec.get('title') or 'N/A'} — {rec.get('author') or 'N/A'} "
                  f"(libro #{rec['id']})")
        elif rec.get("status") == "object":
            print(f"  {i}. oggetto #{rec['id']} (non un libro) — escluso dall'ordinamento")
        elif rec.get("status") == "rotate":
            print(f"  {i}. libro #{rec['id']}: NON IDENTIFICATO — "
                  f"ruota per mostrare '{rec.get('next')}' e rilancia il rilevamento")
        elif rec.get("status") == "error":
            print(f"  {i}. libro #{rec['id']}: ERRORE durante l'identificazione, riprova più tardi")
        else:
            print(f"  {i}. libro #{rec['id']}: NON IDENTIFICATO (nessun elemento utile trovato)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ordina i libri identificati per titolo o autore")
    parser.add_argument("--by", choices=["title", "author"], default="title")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    args = parser.parse_args()

    if not args.input.exists():
        print(f"Errore: '{args.input}' non trovato. Esegui prima detect_sam3.py sulla foto della libreria.")
        sys.exit(1)

    records = json.loads(args.input.read_text(encoding="utf-8"))
    n_missing = sum(1 for r in records if r.get("status") != "found")
    if n_missing:
        print(f"Attenzione: {n_missing}/{len(records)} libri non identificati "
              "(vedi 'status'/'next' in ogni record per sapere come procedere).")

    ordered = sort_books(records, args.by)
    print_plan(ordered, args.by)
