"""
Tests for reorder_planner (offline): valid plan, optimality against an independent BFS, sort criteria.
"""
import itertools
import random
from collections import deque

from agibot_x2_pkg_py.reorder_planner import (DEFAULTS, DEMO_BOOKS, SHELF_HALF_INNER, author_key, order_books,
                                              plan_reorder, title_key, year_key)


def _fits(books, y_by_id, i, y, p):
    """
    Book i fits at y given the other positions (independent copy of the planner rule)
    """
    b = {x["id"]: x for x in books}
    if not (p["y_range"][0] - 1e-9 <= y <= p["y_range"][1] + 1e-9) or abs(y) + b[i]["width"] / 2 + p["wall"] > SHELF_HALF_INNER:
        return False
    return all(j == i or abs(y - yj) >= (b[i]["width"] + b[j]["width"]) / 2 + p["gap"] - 1e-9 for j, yj in y_by_id.items())


def check_valid(books, order, plan, p=DEFAULTS):
    """
    Replay the moves one by one: each must fit in a free gap and the final order must be right
    """
    pos = {b["id"]: round(b["y"], 4) for b in books}
    for m in plan["moves"]:
        assert abs(pos[m["book"]] - m["from_y"]) < 1e-6, "la mossa parte da dove il libro non e'"
        assert _fits(books, pos, m["book"], m["to_y"], p), f"mossa non eseguibile: {m}"
        pos[m["book"]] = m["to_y"]
    assert [i for i, _ in sorted(pos.items(), key=lambda kv: -kv[1])] == list(order), "ordine finale sbagliato"
    assert plan["n_moves"] == len(plan["moves"])
    return pos


def bfs_min_moves(books, order, p, grid):
    """
    Minimum number of moves from an independent breadth-first search (coarse grid)
    """
    ids = [b["id"] for b in books]
    ys = [round(p["y_range"][0] + k * grid, 4) for k in range(int(round((p["y_range"][1] - p["y_range"][0]) / grid)) + 1)]
    cands = sorted(set(ys) | {round(b["y"], 4) for b in books})
    start = tuple(round(b["y"], 4) for b in books)
    goal = lambda s: [ids[i] for i in sorted(range(len(ids)), key=lambda i: -s[i])] == list(order)
    if goal(start):
        return 0
    seen, dq = {start}, deque([(start, 0)])
    while dq:
        s, d = dq.popleft()
        for i in range(len(ids)):
            for y in cands:
                if y == s[i] or not _fits(books, {ids[j]: s[j] for j in range(len(ids))}, ids[i], y, p):
                    continue
                ns = s[:i] + (y,) + s[i + 1:]
                if ns in seen:
                    continue
                if goal(ns):
                    return d + 1
                seen.add(ns)
                dq.append((ns, d + 1))
    return None


def test_demo_books_all_criteria():
    """
    Demo books: valid plan for every criterion and direction, never worse than remove-all
    """
    for crit in ("title", "author", "year", "size"):
        for asc in (True, False):
            order, _w = order_books(DEMO_BOOKS, crit, asc)
            plan = plan_reorder(DEMO_BOOKS, order)
            check_valid(DEMO_BOOKS, order, plan)
            assert plan["n_moves"] <= 2 * len(DEMO_BOOKS)


def test_already_sorted_needs_no_move():
    """
    Already sorted shelf needs no move
    """
    order, _w = order_books(DEMO_BOOKS, "size", True)
    assert plan_reorder(DEMO_BOOKS, order)["n_moves"] == 0


def test_optimal_against_independent_bfs():
    """
    A* move count equals the BFS minimum on random cases
    """
    rnd = random.Random(7)
    p = {**DEFAULTS, "grid": 0.05}
    for _ in range(80):
        n = rnd.choice([3, 4])
        ys = sorted(rnd.sample([-0.28, -0.17, -0.06, 0.05, 0.16, 0.27], n), reverse=True)
        books = [{"id": k + 1, "y": y, "width": rnd.choice([0.038, 0.044, 0.054, 0.07])} for k, y in enumerate(ys)]
        # skip cases whose start positions are physically impossible
        if not all(_fits(books, {b["id"]: b["y"] for b in books}, b["id"], b["y"], p) for b in books):
            continue
        order = [b["id"] for b in books]
        rnd.shuffle(order)
        plan = plan_reorder(books, order, grid=0.05)
        check_valid(books, order, plan, p)
        assert plan["n_moves"] == bfs_min_moves(books, order, p, 0.05), (books, order)


def test_sort_keys():
    """
    Title, author and year sort keys
    """
    assert title_key("Il nome della rosa") == "nome della rosa"
    assert title_key("L'alba della mietitura") == "alba della mietitura"
    assert title_key("IT") == "it"
    assert author_key("Stephen King") == ("king", "stephen")
    assert author_key("Suzanne Collins, Altro Autore") == ("collins", "suzanne")
    assert year_key("2008-09-14") == 2008 and year_key(None) is None and year_key("") is None


def test_missing_data_goes_last_with_warning():
    """
    A book without the criterion value goes last with a warning
    """
    books = [dict(b) for b in DEMO_BOOKS]
    books[2]["author"] = ""
    order, warn = order_books(books, "author", True)
    assert order[-1] == books[2]["id"] and warn


def test_pack_left():
    """
    Packed left in the requested order, every move into a free destination
    """
    import itertools
    from agibot_x2_pkg_py.reorder_planner import pack_left_targets, plan_pack_left, DEFAULTS
    books = [dict(b) for b in DEMO_BOOKS]
    for crit, asc in (("author", True), ("year", True), ("title", False), ("size", True)):
        order, _w = order_books(books, crit, asc)
        tgt = pack_left_targets(books, order)
        ys = [tgt[i] for i in order]
        assert ys == sorted(ys, reverse=True), (crit, ys)            # left (y+) -> right (y-)
        assert abs(ys[0] - 0.32) < 1e-6                              # first book at the left end
        w = {b["id"]: b["width"] for b in books}
        for a, b in zip(order, order[1:]):
            assert abs(tgt[a] - tgt[b]) >= (w[a] + w[b]) / 2 + DEFAULTS["gap"] - 1e-6
        plan = plan_pack_left(books, order)
        cur = {b["id"]: b["y"] for b in books}
        for m in plan["moves"]:                                       # destination free at move time
            i = m["book"]
            assert all(j == i or abs(m["to_y"] - y) >= (w[i] + w[j]) / 2 + DEFAULTS["gap"] - 1e-6
                       for j, y in cur.items()), (crit, m)
            cur[i] = m["to_y"]
        assert {i: round(cur[i], 4) for i in cur} == {i: round(tgt[i], 4) for i in tgt}
        assert plan.get("pack_left") is True


def test_criterio_secondario():
    """
    Books by the same author are ordered by title, then year
    """
    from agibot_x2_pkg_py.reorder_planner import order_books
    books = [{"id": 1, "y": 0.05, "width": 0.04, "title": "Zeta", "author": "Suzanne Collins", "year": 2010},
             {"id": 2, "y": -0.05, "width": 0.04, "title": "Alfa", "author": "Suzanne Collins", "year": 2020},
             {"id": 3, "y": -0.15, "width": 0.04, "title": "Beta", "author": "Suzanne Collins", "year": 2000},
             {"id": 4, "y": -0.25, "width": 0.04, "title": "Delta", "author": "Ada Zorn", "year": 1999}]
    order, _ = order_books(books, "author", True)
    assert order == [2, 3, 1, 4], order                     # Collins (Alfa, Beta, Zeta), then Zorn
    order, _ = order_books(books, "author", False)
    assert order == [4, 2, 3, 1], order                     # Z->A on the author, ties still ordered by title
    order, _ = order_books(books, "year", True)
    assert order == [4, 3, 1, 2], order


def test_raggiungibilita_e_ancore():
    """
    A book whose packed slot is unreachable (It at -0.122) stays put and the others are placed around it
    """
    from agibot_x2_pkg_py.reorder_planner import order_books, plan_pack_left
    from agibot_x2_pkg_py.reach_map import move_feasible, arms, unreachable_band
    assert arms(3, -0.161) == "R" and arms(3, -0.122) == "" and arms(9, 0.0) is None
    assert move_feasible(3, -0.161, -0.122) is False and move_feasible(4, -0.283, 0.0) is True
    assert move_feasible(4, -0.283, 0.777) is None                      # not measured: try it
    lo, hi = unreachable_band()[0]
    assert lo < hi and 0.05 < lo < 0.12 and 0.12 < hi < 0.25, (lo, hi)  # derived from the measurements (0.104..0.15 +-0.03)
    books = [{"id": 1, "y": 0.047, "width": 0.038, "title": "A"}, {"id": 2, "y": -0.053, "width": 0.044, "title": "B"},
             {"id": 3, "y": -0.161, "width": 0.056, "title": "C"}, {"id": 4, "y": -0.283, "width": 0.070, "title": "D"}]
    order = [1, 2, 4, 3]
    plan = plan_pack_left(books, order, feasible=move_feasible)
    assert plan["anchored"] == {3: -0.161} and len(plan["moves"]) == 3 and plan["notes"], plan
    assert all(m["book"] != 3 for m in plan["moves"])
    assert plan["final"][3] == -0.161
    plan0 = plan_pack_left(books, order)                                # without the map: 4 moves, the last one impossible
    assert len(plan0["moves"]) == 4


def test_pack_left_con_parcheggi_multipli():
    """
    Cases needing multiple parkings (year ascending, title Z->A, size descending) reach the packed targets
    """
    from agibot_x2_pkg_py.reorder_planner import plan_pack_left, pack_left_targets, DEFAULTS
    books = [dict(b) for b in DEMO_BOOKS]
    for crit, asc in (("year", True), ("title", False), ("size", False)):
        order, _w = order_books(books, crit, asc)
        plan = plan_pack_left(books, order)
        tgt = pack_left_targets(books, order)
        assert plan["final"] == {i: tgt[i] for i in tgt}, crit


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("OK", name)
