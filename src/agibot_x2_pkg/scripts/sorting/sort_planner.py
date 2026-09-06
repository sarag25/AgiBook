"""
Calcola l'ordine target dei libri in base al criterio scelto dall'utente.
Separa prima gli ostacoli (da spostare sul carrello) dai libri.
Supporta modalità LIFO.
"""

from __future__ import annotations
import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from scripts.vision.book_detector import DetectedObject
    from scripts.input.input_handler import SortCommand, SortCriterion
    from scripts.vision.color_analyzer import ColorAnalyzer

log = logging.getLogger(__name__)


@dataclass
class SortPlan:
    """
    Piano di riordinamento completo.

    obstacles_to_move: lista oggetti da spostare sul carrello (prima di tutto)
    removal_order:     ordine in cui rimuovere i libri dallo scaffale
    insertion_order:   ordine in cui reinserire i libri nello scaffale
    lifo_mode:         se True, removal_order è l'inverso di insertion_order
    """
    obstacles_to_move: list["DetectedObject"] = field(default_factory=list)
    removal_order: list["DetectedObject"] = field(default_factory=list)
    insertion_order: list["DetectedObject"] = field(default_factory=list)
    lifo_mode: bool = False

    def describe(self) -> str:
        lines = [
            f"Piano riordinamento:",
            f"  Ostacoli da spostare: {len(self.obstacles_to_move)}",
            f"  Libri da rimuovere:   {len(self.removal_order)}",
            f"  Ordine inserimento:   {[b.obj_id for b in self.insertion_order]}",
            f"  Modalità LIFO:        {self.lifo_mode}",
        ]
        for i, b in enumerate(self.insertion_order):
            title = b.title or b.color_name or f"Libro#{b.obj_id}"
            lines.append(f"    {i+1}. [{b.obj_id}] {title} "
                         f"(riga {b.shelf_row}, slot {b.shelf_slot})")
        return "\n".join(lines)


class SortPlanner:
    """
    Calcola il piano di riordinamento dato:
      - la lista di oggetti rilevati (DetectedObject)
      - il comando dell'utente (SortCommand)

    Uso:
        planner = SortPlanner(color_analyzer)
        plan = planner.compute_plan(objects, sort_command)
    """

    def __init__(self, color_analyzer: "ColorAnalyzer" = None):
        self._color_analyzer = color_analyzer

    def compute_plan(self, objects: list["DetectedObject"],
                     command: "SortCommand") -> SortPlan:
        # import senza prefisso "scripts." (fix 2026-08-29): a runtime il
        # package "scripts" non esiste - library_manager_node aggiunge la
        # cartella scripts/ a sys.path, quindi i moduli si importano come
        # input.*/vision.* (crash visto al primo run reale, ModuleNotFoundError)
        from input.input_handler import SortCriterion

        # Separa ostacoli (oggetti non-libro davanti ai libri)
        obstacles = [o for o in objects if o.is_obstacle]
        books = [o for o in objects if o.is_book]

        log.info(f"Planning: {len(books)} libri, {len(obstacles)} ostacoli, "
                 f"criterio={command.criterion.value}")

        # Calcola ordine target per i libri
        insertion_order = self._sort_books(books, command)

        # Ordine di rimozione
        if command.lifo_mode:
            # LIFO: rimuovi in ordine inverso così il primo inserito è il
            # primo nell'ordine target (no swap necessari)
            removal_order = list(reversed(insertion_order))
            log.info("Modalità LIFO attiva: rimozione in ordine inverso")
        else:
            # Rimuovi nell'ordine corrente (slot per slot, riga per riga)
            removal_order = sorted(books,
                                   key=lambda b: (b.shelf_row, b.shelf_slot))

        # Ostacoli con profondità: sposta prima quelli più vicini (davanti)
        obstacles_sorted = sorted(obstacles, key=lambda o: o.depth_m)

        plan = SortPlan(
            obstacles_to_move=obstacles_sorted,
            removal_order=removal_order,
            insertion_order=insertion_order,
            lifo_mode=command.lifo_mode,
        )
        log.info(plan.describe())
        return plan

    def _sort_books(self, books: list["DetectedObject"],
                    command: "SortCommand") -> list["DetectedObject"]:
        # import senza prefisso "scripts." (fix 2026-08-29): a runtime il
        # package "scripts" non esiste - library_manager_node aggiunge la
        # cartella scripts/ a sys.path, quindi i moduli si importano come
        # input.*/vision.* (crash visto al primo run reale, ModuleNotFoundError)
        from input.input_handler import SortCriterion

        crit = command.criterion
        asc = command.ascending

        if crit == SortCriterion.COLOR:
            return self._sort_by_color(books, asc)

        elif crit in (SortCriterion.TITLE, SortCriterion.AUTHOR):
            # Logica condivisa con la pipeline standalone (sort_strings.py,
            # 2026-09-06 - TODO "Unire logica riconoscimento libri e
            # riordinamento stringhe"): alfabetico case-insensitive, libri
            # senza titolo/autore ("" o "N/A") sempre in coda - il vecchio
            # trucco "zzz" li mescolava in testa in ordine decrescente.
            from sorting.sort_strings import sort_by_field
            field = "title" if crit == SortCriterion.TITLE else "author"
            return sort_by_field(books, lambda b: getattr(b, field), asc)

        elif crit == SortCriterion.SIZE:
            return self._sort_by_size(books, asc)

        else:
            log.warning("Nessun criterio specificato: mantengo ordine attuale")
            return sorted(books, key=lambda b: (b.shelf_row, b.shelf_slot))

    def _sort_by_color(self, books: list["DetectedObject"],
                       ascending: bool) -> list["DetectedObject"]:
        """
        Ordina per tonalità (ordine arcobaleno).
        Raggruppa libri dello stesso colore per avere blocchi omogenei.
        """
        from vision.color_analyzer import ColorAnalyzer
        ca = self._color_analyzer or ColorAnalyzer()

        rainbow_order = [
            "rosso", "arancione", "giallo", "verde",
            "ciano", "blu", "viola", "magenta",
            "bianco", "grigio", "nero", "altro", "sconosciuto"
        ]

        def color_key(b):
            try:
                return rainbow_order.index(b.color_name)
            except ValueError:
                return 99

        sorted_books = sorted(books, key=color_key, reverse=not ascending)
        return sorted_books

    def _sort_by_size(self, books: list["DetectedObject"],
                      ascending: bool) -> list["DetectedObject"]:
        """
        Ottimizza lo spazio scaffale:
        ordina per altezza decrescente (libri alti a sinistra) per
        minimizzare i gap e massimizzare la stabilità.
        """
        return sorted(books,
                      key=lambda b: b.height_px,
                      reverse=ascending)   # ascending=True → alti prima

    def group_by_shelf_row(self,
                           books: list["DetectedObject"]
                           ) -> dict[int, list["DetectedObject"]]:
        """Raggruppa i libri per ripiano."""
        rows: dict[int, list] = {}
        for b in books:
            rows.setdefault(b.shelf_row, []).append(b)
        return rows
