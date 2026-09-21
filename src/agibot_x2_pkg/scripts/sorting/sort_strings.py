"""
String sorting logic shared by the ROS pipeline (sorting/sort_planner.py, TITLE/AUTHOR
criteria) and the standalone <repo>/sorting scripts.
Case-insensitive alphabetical order; missing values ("" or "N/A", books not yet identified)
always go last, both ascending and descending.
"""

from __future__ import annotations


def normalized_key(value: str | None) -> tuple[bool, str]:
    """
    (missing, normalized value): sortable tuple that puts missing values last
    and compares the rest case-insensitively
    """
    value = (value or "").strip()
    missing = value == "" or value == "N/A"
    return (missing, value.lower())


def sort_strings(strings: list[str]) -> list[str]:
    """
    Case-insensitive alphabetical sort of a list of strings
    """
    return sorted(strings, key=str.lower)


def sort_by_field(items: list, get_value, ascending: bool = True) -> list:
    """
    Sort `items` by get_value(item): present values ascending or descending,
    missing values always last
    """
    present = [i for i in items if not normalized_key(get_value(i))[0]]
    missing = [i for i in items if normalized_key(get_value(i))[0]]
    present.sort(key=lambda i: normalized_key(get_value(i))[1],
                 reverse=not ascending)
    return present + missing
