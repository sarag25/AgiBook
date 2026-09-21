"""
Helpers that compute the target book order from the user's sort criterion.
Obstacles (to be moved to the cart) are separated from the books first; LIFO mode is supported.
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
    Complete reordering plan
    obstacles_to_move: objects to move to the cart first
    removal_order / insertion_order: order to take books off / put them back on the shelf
    lifo_mode: if True, removal_order is the reverse of insertion_order
    """
    obstacles_to_move: list["DetectedObject"] = field(default_factory=list)
    removal_order: list["DetectedObject"] = field(default_factory=list)
    insertion_order: list["DetectedObject"] = field(default_factory=list)
    lifo_mode: bool = False

    def describe(self) -> str:
        """
        Multi-line text summary of the plan
        """
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
    Computes the reordering plan from the detected objects and the user's SortCommand
    Usage: plan = SortPlanner(color_analyzer).compute_plan(objects, sort_command)
    """

    def __init__(self, color_analyzer: "ColorAnalyzer" = None):
        """
        Optional ColorAnalyzer for the COLOR criterion
        """
        self._color_analyzer = color_analyzer

    def compute_plan(self, objects: list["DetectedObject"],
                     command: "SortCommand") -> SortPlan:
        """
        Build the SortPlan: obstacles nearest first, books in target order
        """
        # no "scripts." prefix: library_manager_node puts scripts/ on sys.path
        from input.input_handler import SortCriterion

        # obstacles = non-book objects in front of the books
        obstacles = [o for o in objects if o.is_obstacle]
        books = [o for o in objects if o.is_book]

        log.info(f"Planning: {len(books)} books, {len(obstacles)} obstacles, "
                 f"criterion={command.criterion.value}")

        insertion_order = self._sort_books(books, command)

        if command.lifo_mode:
            # LIFO: remove in reverse so the first inserted is first in the target order (no swaps)
            removal_order = list(reversed(insertion_order))
            log.info("LIFO mode active: removing in reverse order")
        else:
            # current order: row by row, slot by slot
            removal_order = sorted(books,
                                   key=lambda b: (b.shelf_row, b.shelf_slot))

        # nearest obstacles first
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
        """
        Books in target order for the command's criterion
        """
        # no "scripts." prefix: library_manager_node puts scripts/ on sys.path
        from input.input_handler import SortCriterion

        crit = command.criterion
        asc = command.ascending

        if crit == SortCriterion.COLOR:
            return self._sort_by_color(books, asc)

        elif crit in (SortCriterion.TITLE, SortCriterion.AUTHOR):
            # shared with the standalone pipeline: books without title/author always last
            from sorting.sort_strings import sort_by_field
            field = "title" if crit == SortCriterion.TITLE else "author"
            return sort_by_field(books, lambda b: getattr(b, field), asc)

        elif crit == SortCriterion.SIZE:
            return self._sort_by_size(books, asc)

        else:
            log.warning("No criterion specified: keeping the current order")
            return sorted(books, key=lambda b: (b.shelf_row, b.shelf_slot))

    def _sort_by_color(self, books: list["DetectedObject"],
                       ascending: bool) -> list["DetectedObject"]:
        """
        Sort by hue (rainbow order), grouping same-colored books into blocks
        """
        from vision.color_analyzer import ColorAnalyzer
        ca = self._color_analyzer or ColorAnalyzer()

        rainbow_order = [
            "rosso", "arancione", "giallo", "verde",
            "ciano", "blu", "viola", "magenta",
            "bianco", "grigio", "nero", "altro", "sconosciuto"
        ]

        def color_key(b):
            """
            Rainbow index of the book color (unknown colors last)
            """
            try:
                return rainbow_order.index(b.color_name)
            except ValueError:
                return 99

        sorted_books = sorted(books, key=color_key, reverse=not ascending)
        return sorted_books

    def _sort_by_size(self, books: list["DetectedObject"],
                      ascending: bool) -> list["DetectedObject"]:
        """
        Sort by height (tall books on the left) to minimize gaps and maximize stability
        """
        return sorted(books,
                      key=lambda b: b.height_px,
                      reverse=ascending)   # ascending=True -> tall first

    def group_by_shelf_row(self,
                           books: list["DetectedObject"]
                           ) -> dict[int, list["DetectedObject"]]:
        """
        Group the books by shelf row
        """
        rows: dict[int, list] = {}
        for b in books:
            rows.setdefault(b.shelf_row, []).append(b)
        return rows
