"""
Wrapper: l'implementazione unica dell'ordinamento stringhe vive in
src/agibot_x2_pkg/scripts/sorting/sort_strings.py (condivisa con la
pipeline ROS - TODO "Unire logica riconoscimento libri e riordinamento
stringhe", 2026-09-06). Qui la si importa via path relativo al repo cosi'
gli script standalone continuano a funzionare invariati.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / "src" / "agibot_x2_pkg" / "scripts"))
from sorting.sort_strings import normalized_key, sort_by_field, sort_strings  # noqa: E402,F401

if __name__ == "__main__":
    book_list = ["It", "Hunger Games", "Emma"]
    print("Prima:", book_list)
    print("Dopo: ", sort_strings(book_list))
