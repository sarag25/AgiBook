"""Prove offline di reorder_planner: piano valido, ottimo (vs BFS indipendente), criteri di ordinamento."""
import itertools
import random
from collections import deque

from agibot_x2_pkg_py.reorder_planner import (DEFAULTS, DEMO_BOOKS, SHELF_HALF_INNER, author_key, order_books,
                                              plan_reorder, title_key, year_key)


def _fits(books, y_by_id, i, y, p):
    b = {x["id"]: x for x in books}
    if not (p["y_range"][0] - 1e-9 <= y <= p["y_range"][1] + 1e-9) or abs(y) + b[i]["width"] / 2 + p["wall"] > SHELF_HALF_INNER:
        return False
    return all(j == i or abs(y - yj) >= (b[i]["width"] + b[j]["width"]) / 2 + p["gap"] - 1e-9 for j, yj in y_by_id.items())


def check_valid(books, order, plan, p=DEFAULTS):
    """Riesegue le mosse una per una: ognuna deve entrare in un vuoto; alla fine l'ordine e' giusto."""
    pos = {b["id"]: round(b["y"], 4) for b in books}
    for m in plan["moves"]:
        assert abs(pos[m["book"]] - m["from_y"]) < 1e-6, "la mossa parte da dove il libro non e'"
        assert _fits(books, pos, m["book"], m["to_y"], p), f"mossa non eseguibile: {m}"
        pos[m["book"]] = m["to_y"]
    assert [i for i, _ in sorted(pos.items(), key=lambda kv: -kv[1])] == list(order), "ordine finale sbagliato"
    assert plan["n_moves"] == len(plan["moves"])
    return pos


def bfs_min_moves(books, order, p, grid):
    """Minimo numero di mosse con una ricerca in ampiezza indipendente (griglia grossolana)."""
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
    for crit in ("title", "author", "year", "size"):
        for asc in (True, False):
            order, _w = order_books(DEMO_BOOKS, crit, asc)
            plan = plan_reorder(DEMO_BOOKS, order)
            check_valid(DEMO_BOOKS, order, plan)
            assert plan["n_moves"] <= 2 * len(DEMO_BOOKS)


def test_already_sorted_needs_no_move():
    order, _w = order_books(DEMO_BOOKS, "size", True)
    assert plan_reorder(DEMO_BOOKS, order)["n_moves"] == 0


def test_optimal_against_independent_bfs():
    rnd = random.Random(7)
    p = {**DEFAULTS, "grid": 0.05}
    for _ in range(80):
        n = rnd.choice([3, 4])
        ys = sorted(rnd.sample([-0.28, -0.17, -0.06, 0.05, 0.16, 0.27], n), reverse=True)
        books = [{"id": k + 1, "y": y, "width": rnd.choice([0.038, 0.044, 0.054, 0.07])} for k, y in enumerate(ys)]
        # posizioni di partenza fisicamente possibili? se no, il caso non e' valido
        if not all(_fits(books, {b["id"]: b["y"] for b in books}, b["id"], b["y"], p) for b in books):
            continue
        order = [b["id"] for b in books]
        rnd.shuffle(order)
        plan = plan_reorder(books, order, grid=0.05)
        check_valid(books, order, plan, p)
        assert plan["n_moves"] == bfs_min_moves(books, order, p, 0.05), (books, order)


def test_sort_keys():
    assert title_key("Il nome della rosa") == "nome della rosa"
    assert title_key("L'alba della mietitura") == "alba della mietitura"
    assert title_key("IT") == "it"
    assert author_key("Stephen King") == ("king", "stephen")
    assert author_key("Suzanne Collins, Altro Autore") == ("collins", "suzanne")
    assert year_key("2008-09-14") == 2008 and year_key(None) is None and year_key("") is None


def test_missing_data_goes_last_with_warning():
    books = [dict(b) for b in DEMO_BOOKS]
    books[2]["author"] = ""
    order, warn = order_books(books, "author", True)
    assert order[-1] == books[2]["id"] and warn


def test_pack_left():
    """2026-09-19: impacchettati a sinistra nell'ordine richiesto, ogni mossa in una destinazione libera."""
    import itertools
    from agibot_x2_pkg_py.reorder_planner import pack_left_targets, plan_pack_left, DEFAULTS
    books = [dict(b) for b in DEMO_BOOKS]
    for crit, asc in (("author", True), ("year", True), ("title", False), ("size", True)):
        order, _w = order_books(books, crit, asc)
        tgt = pack_left_targets(books, order)
        ys = [tgt[i] for i in order]
        assert ys == sorted(ys, reverse=True), (crit, ys)            # sinistra (y+) -> destra (y-)
        assert abs(ys[0] - 0.32) < 1e-6                              # primo libro all'estremo sinistro
        w = {b["id"]: b["width"] for b in books}
        for a, b in zip(order, order[1:]):
            assert abs(tgt[a] - tgt[b]) >= (w[a] + w[b]) / 2 + DEFAULTS["gap"] - 1e-6
        plan = plan_pack_left(books, order)
        cur = {b["id"]: b["y"] for b in books}
        for m in plan["moves"]:                                       # la destinazione e' libera al momento
            i = m["book"]
            assert all(j == i or abs(m["to_y"] - y) >= (w[i] + w[j]) / 2 + DEFAULTS["gap"] - 1e-6
                       for j, y in cur.items()), (crit, m)
            cur[i] = m["to_y"]
        assert {i: round(cur[i], 4) for i in cur} == {i: round(tgt[i], 4) for i in tgt}
        assert plan.get("pack_left") is True


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("OK", name)
