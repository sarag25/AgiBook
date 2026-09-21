#!/usr/bin/env python3
"""
Shelf reorder planner with the MINIMUM number of picks (A* over book positions).

Each pick+place costs ~15-20 real minutes at RTF 0.05, so the number of picks matters, not the sort cost.
Books are y-intervals on the shelf, never parked on the table; a book moved more than once is a "parcheggio".
  python3 -m agibot_x2_pkg_py.reorder_planner --by author
  python3 -m agibot_x2_pkg_py.reorder_planner --library /tmp/x2_library.json --by year --desc
  python3 -m agibot_x2_pkg_py.reorder_planner --demo --by title --json
"""
import argparse
import heapq
import itertools
import json
import re
import sys
import unicodedata


try:                                       # half inner width, from bookshelf.urdf (scene_config)
    from agibot_x2_pkg.scene_config import SHELF_HALF_INNER_WIDTH as SHELF_HALF_INNER
except ImportError:                        # UI without the ROS package on PYTHONPATH: reference value
    SHELF_HALF_INNER = 0.378
def _band_default():
    """
    Unreachable y band, derived from the reach_map measurements
    """
    try:
        from agibot_x2_pkg_py.reach_map import unreachable_band
        return unreachable_band()
    except Exception:                                   # without the module: reference value
        return ((0.075, 0.215),)


DEFAULTS = dict(
    # slot centers no arm can reach (no pick or place there); None = derived from reach_map.
    # The hand retracting in front of the left shoulder (y ~ +0.15) has no room.
    unreachable=None,
    gap=0.059,                # min free space between two books so the gripper palm fits (see book_placer)
    y_range=(-0.32, 0.32),    # where the arm reaches well (globe at +0.28 and Hunger at -0.28 picked)
    wall=0.03,                # margin from the inner side wall of the case
    grid=0.02,                # step of the candidate positions
    sec_per_move=1100.0,      # REAL seconds per pick+place (RTF 0.05, measured 700-1400 s)
    max_expansions=400000,
)

# real books from a pipeline run (decreasing y = left to right), for offline tests
DEMO_BOOKS = [
    {"id": 1, "y": 0.047, "width": 0.038, "title": "L'alba della mietitura", "author": "Suzanne Collins", "year": 2020},
    {"id": 2, "y": -0.053, "width": 0.044, "title": "La ballata dell'usignolo e del serpente", "author": "Suzanne Collins", "year": 2020},
    {"id": 3, "y": -0.162, "width": 0.054, "title": "IT", "author": "Stephen King", "year": 1986},
    {"id": 4, "y": -0.283, "width": 0.070, "title": "Hunger Games", "author": "Suzanne Collins", "year": 2008},
]

_ARTICLES = {"il", "lo", "la", "l", "i", "gli", "le", "un", "uno", "una", "the", "a", "an"}


# sort criteria
def _norm(s):
    """
    Lowercase ASCII text with only letters, digits and spaces
    """
    s = unicodedata.normalize("NFKD", str(s or "")).encode("ascii", "ignore").decode()
    return re.sub(r"[^a-z0-9 ]+", " ", s.casefold()).strip()


def title_key(t):
    """
    Sort key of a title, skipping leading articles; None if missing
    """
    words = _norm(t).split()
    while words and words[0] in _ARTICLES and len(words) > 1:
        words = words[1:]
    return " ".join(words) or None


def author_key(a):
    """
    Sort key (surname, first names) of the first author; None if missing
    """
    first = re.split(r"[,;&]| e | and ", str(a or ""))[0]
    words = _norm(first).split()
    return (words[-1], " ".join(words[:-1])) if words else None      # surname, then first name


def year_key(y):
    """
    Year as int from the first 4 characters; None if missing
    """
    try:
        return int(str(y)[:4])
    except (TypeError, ValueError):
        return None


CRITERIA = {
    "title": ("Titolo", lambda b: title_key(b.get("title"))),
    "author": ("Autore", lambda b: author_key(b.get("author"))),
    "year": ("Anno", lambda b: year_key(b.get("year"))),
    "size": ("Spessore", lambda b: float(b.get("width") or 0) or None),
}


# secondary criteria in priority order, to break ties (e.g. three books by the same author)
SECONDARY = {"title": ("author", "year"), "author": ("title", "year"), "year": ("author", "title"), "size": ("title",)}
_SEC_FN = {"title": title_key, "author": author_key, "year": year_key}


def _sec_key(b, name):
    """
    Secondary sort key; books without the value go last
    """
    v = _SEC_FN[name](b.get(name))
    return (1, ()) if v is None else (0, v)          # missing value: last


def order_books(books, criterion, ascending=True):
    """
    Return (ids in the requested left-to-right order, warnings)
    Books missing the value go last (in current order) with a warning; ties are broken by the
    SECONDARY criteria (always ascending), then by the current order.
    """
    if criterion not in CRITERIA:
        raise ValueError(f"unknown criterion '{criterion}': {list(CRITERIA)}")
    _label, key = CRITERIA[criterion]
    have = [b for b in books if key(b) is not None]
    miss = sorted([b for b in books if key(b) is None], key=lambda b: -b["y"])
    have.sort(key=lambda b: -b["y"])                                  # last tie-break: current order (stable)
    for name in reversed(SECONDARY[criterion]):                       # stable sorts: the last one wins
        have.sort(key=lambda b, n=name: _sec_key(b, n))
    have.sort(key=key, reverse=not ascending)
    warn = [f"libro {b['id']} senza {CRITERIA[criterion][0].lower()}: messo in fondo" for b in miss]
    return [b["id"] for b in have + miss], warn


def _params(kw):
    """
    DEFAULTS overridden by kw, with the unreachable band resolved
    """
    p = {**DEFAULTS, **{k: v for k, v in kw.items() if k not in ("feasible", "anchors")}}
    if p.get("unreachable") is None:
        p["unreachable"] = _band_default()
    return p


def _lis(seq):
    """
    Length of the longest increasing subsequence
    """
    best = [1] * len(seq)
    for i in range(len(seq)):
        for j in range(i):
            if seq[j] < seq[i]:
                best[i] = max(best[i], best[j] + 1)
    return max(best, default=0)


def plan_reorder(books, order, **kw):
    """
    Optimal plan (fewest picks, then shortest distance) to reach `order`; ValueError if impossible
    books: [{id, y, width, ...}], order: ids left to right (decreasing y).
    A* with cost 1 + 0.1*|dy| and heuristic n - LIS: each move raises the LIS by at most 1, so it is
    admissible and consistent (optimal over the `grid` candidates plus the start positions).
    feasible(book, y_from, y_to) -> True/False/None (reach_map.move_feasible): False moves are excluded.
    """
    p = _params(kw)
    feasible = kw.get("feasible") or (lambda b, a, c: None)
    ids = [b["id"] for b in books]
    n = len(ids)
    w = [float(b["width"]) for b in books]
    rank = [order.index(i) for i in ids]
    gap, y_lo, y_hi = p["gap"], p["y_range"][0], p["y_range"][1]
    start = tuple(round(float(b["y"]), 4) for b in books)

    def fits(i, y, state):
        """
        Book i fits at y: inside the reachable range and the case, with `gap` from the others
        """
        if not (y_lo - 1e-9 <= y <= y_hi + 1e-9) or abs(y) + w[i] / 2 + p["wall"] > SHELF_HALF_INNER:
            return False
        return all(j == i or abs(y - state[j]) >= (w[i] + w[j]) / 2 + gap - 1e-9 for j in range(n))

    def h(state):
        """
        Lower bound on moves: n - LIS of the current order
        """
        seq = [rank[i] for i in sorted(range(n), key=lambda i: -state[i])]
        return n - _lis(seq)

    grid = [round(y_lo + k * p["grid"], 4) for k in range(int(round((y_hi - y_lo) / p["grid"])) + 1)]
    cands = sorted(set(grid) | set(start))
    if h(start) == 0:
        return _result(books, order, start, start, [], p)

    tick = itertools.count()
    heap = [(h(start), next(tick), 0.0, start, None)]
    best = {start: 0.0}
    parent = {start: None}
    expansions = 0
    while heap:
        _f, _t, g, state, _m = heapq.heappop(heap)
        if g > best.get(state, 1e9) + 1e-12:
            continue
        if h(state) == 0:
            moves = []
            s = state
            while parent[s] is not None:
                s, mv = parent[s]
                moves.append(mv)
            moves.reverse()
            return _result(books, order, start, state, moves, p)
        expansions += 1
        if expansions > p["max_expansions"]:
            raise ValueError("troppi stati: riordino troppo grande per la ricerca esatta")
        for i in range(n):
            for y in cands:
                if y == state[i] or not fits(i, y, state) or feasible(ids[i], state[i], y) is False:
                    continue
                ns = state[:i] + (y,) + state[i + 1:]
                ng = g + 1.0 + 0.1 * abs(y - state[i])
                if ng < best.get(ns, 1e9) - 1e-12:
                    best[ns] = ng
                    parent[ns] = (state, (i, state[i], y))
                    heapq.heappush(heap, (ng + h(ns), next(tick), ng, ns, None))
    raise ValueError("nessun riordino possibile con questi spazi (serve piu' spazio libero sullo scaffale)")


def pack_left_targets(books, order, anchors=None, **kw):
    """
    Final {id: y_center} of books PACKED LEFT: the first at the max reachable y, the others to the right
    with `gap` between them (the room the fingers need to place/pick).
    anchors {id: y}: books that do NOT move (they cannot reach their packed slot); the others are placed
    around them. ValueError if an anchor is incompatible.
    """
    p = _params(kw)
    anchors = anchors or {}
    by = {b["id"]: b for b in books}
    y_hi = min(p["y_range"][1], SHELF_HALF_INNER - p["wall"] - float(by[order[0]]["width"]) / 2.0)
    out, y, prev_w = {}, y_hi, None
    for i in order:
        w = float(by[i]["width"])
        y = y_hi if prev_w is None else y - prev_w / 2.0 - p["gap"] - w / 2.0
        if i in anchors:
            if anchors[i] > y + 1e-9:
                raise ValueError(f"il libro {i} resta dov'e' ma i libri prima di lui non ci stanno")
            y = float(anchors[i])
        else:
            for lo, hi in p["unreachable"]:             # skip the unreachable band (hole in the shelf)
                if lo <= y <= hi:
                    y = min(y, lo - 0.075)              # below the band, where the left arm reaches (map: 0.0 is LR)
        out[i] = round(y, 4)
        prev_w = w
    low = min(out[i] - float(by[i]["width"]) / 2.0 for i in out)
    if low < -SHELF_HALF_INNER + p["wall"]:
        raise ValueError("i libri non stanno nello scaffale impacchettati a sinistra con questo spazio")
    return out


def _search_targets(ids, w, cur, tgt, p, feasible):
    """
    A* from the current positions to `tgt`, parking anywhere free
    Needed when several books block each other and parking a single blocker is not enough.
    Return [(index, y_from, y_to)] with the minimum number of moves (cost 1 + 0.1*|dy|).
    """
    n, gap, tol = len(ids), p["gap"], 0.005
    grid = [round(-0.32 + 0.02 * k, 4) for k in range(33)]
    cands = sorted(set(grid) | set(cur.values()) | set(tgt.values()))
    start = tuple(cur[i] for i in ids)
    goal = tuple(tgt[i] for i in ids)
    wi = [w[i] for i in ids]

    def ok(k, y, state):
        """
        Book k can go to y: inside the case, not in the unreachable band (unless it is its goal), with `gap`
        """
        if abs(y) + wi[k] / 2 + p["wall"] > SHELF_HALF_INNER + 1e-9:
            return False
        if any(lo <= y <= hi for lo, hi in p["unreachable"]) and abs(y - goal[k]) > tol:
            return False
        return all(j == k or abs(y - state[j]) >= (wi[k] + wi[j]) / 2 + gap - 1e-9 for j in range(n))

    def h(state):
        """
        Number of books not yet at their goal
        """
        return sum(1 for k in range(n) if abs(state[k] - goal[k]) > tol)

    tick = itertools.count()
    heap = [(h(start), next(tick), 0.0, start)]
    best, parent, expansions = {start: 0.0}, {start: None}, 0
    while heap:
        _f, _t, g, state = heapq.heappop(heap)
        if g > best.get(state, 1e9) + 1e-12:
            continue
        if h(state) == 0:
            moves, s_ = [], state
            while parent[s_] is not None:
                s_, mv = parent[s_]
                moves.append(mv)
            return list(reversed(moves))
        expansions += 1
        if expansions > p["max_expansions"]:
            break
        for k in range(n):
            for y in cands:
                if y == state[k] or not ok(k, y, state) or feasible(ids[k], state[k], y) is False:
                    continue
                ns = state[:k] + (y,) + state[k + 1:]
                ng = g + 1.0 + 0.1 * abs(y - state[k])
                if ng < best.get(ns, 1e9) - 1e-12:
                    best[ns], parent[ns] = ng, (state, (k, state[k], y))
                    heapq.heappush(heap, (ng + h(ns), next(tick), ng, ns))
    raise ValueError("nessun modo di portare i libri nelle posizioni finali con gli spazi attuali")


def plan_pack_left(books, order, **kw):
    """
    Plan to bring the books, in `order` (left -> right), to the pack_left_targets positions
    Moves are sequenced so each destination is free at move time; a blocking book is parked first
    ("parcheggio"), falling back to A* when simple parking is not enough. No book ever goes on the table.
    feasible(book, y_from, y_to): a book that cannot reach its packed slot stays put as an anchor.
    """
    p = _params(kw)
    feasible = kw.get("feasible") or (lambda b, a, c: None)
    ids = [b["id"] for b in books]
    by = {b["id"]: b for b in books}
    w = {i: float(by[i]["width"]) for i in ids}
    cur = {i: round(float(by[i]["y"]), 4) for i in ids}
    start = dict(cur)
    gap, tol = p["gap"], 0.005
    pkw = {k: v for k, v in kw.items() if k not in ("feasible", "anchors")}

    anchors, notes = {}, []
    tgt = pack_left_targets(books, order, anchors=anchors, **pkw)
    for _ in range(len(ids) + 1):
        bad = [i for i in order if abs(cur[i] - tgt[i]) > tol and i not in anchors
               and feasible(i, cur[i], tgt[i]) is False]
        if not bad:
            break
        for i in bad:
            anchors[i] = cur[i]
            notes.append(f"«{by[i].get('title') or 'libro ' + str(i)}» resta dov'è: il robot non riesce ad "
                         f"arrivare nella nuova posizione.")
        try:
            tgt = pack_left_targets(books, order, anchors=anchors, **pkw)
        except ValueError:
            raise ValueError("un libro non raggiunge la sua nuova posizione e gli altri non stanno attorno a lui")

    def free(i, y):
        """
        Book i can be placed at y now: inside the case, reachable and with `gap` from the others
        """
        return (abs(y) + w[i] / 2 + p["wall"] <= SHELF_HALF_INNER + 1e-9 and
                not any(lo <= y <= hi for lo, hi in p["unreachable"]) and
                feasible(i, cur[i], y) is not False and
                all(j == i or abs(y - cur[j]) >= (w[i] + w[j]) / 2 + gap - 1e-9 for j in ids))

    def greedy():
        """
        Place every book whose target is free, parking the first blocker when all are blocked
        """
        moves, guard = [], 0
        while any(abs(cur[i] - tgt[i]) > tol for i in ids):
            guard += 1
            if guard > 4 * len(ids) + 4:
                raise ValueError("riordino impossibile con gli spazi attuali")
            pend = [i for i in order if abs(cur[i] - tgt[i]) > tol]
            ready = [i for i in pend if free(i, tgt[i])]
            if ready:
                i = ready[0]
                moves.append({"book": i, "from_y": cur[i], "to_y": tgt[i], "kind": "definitiva"})
                cur[i] = tgt[i]
                continue
            # all blocked: park the first blocker in the nearest free spot
            i = pend[0]
            blockers = [j for j in ids if j != i and abs(tgt[i] - cur[j]) < (w[i] + w[j]) / 2 + gap]
            b = blockers[0]
            cands = [round(-0.30 + 0.01 * k, 3) for k in range(0, 61)]
            spots = sorted((y for y in cands if free(b, y) and abs(y - tgt[b]) > 0.02 and
                            all(abs(y - tgt[j]) >= (w[b] + w[j]) / 2 + gap for j in ids if j != b)),
                           key=lambda y: abs(y - cur[b]))
            if not spots:
                raise ValueError("nessun punto libero per parcheggiare un libro")
            moves.append({"book": b, "from_y": cur[b], "to_y": spots[0], "kind": "parcheggio"})
            cur[b] = spots[0]
        return [(ids.index(m["book"]), m["from_y"], m["to_y"]) for m in moves]

    try:
        mv = greedy()
    except ValueError:
        cur.update(start)                                # restore and search with A*
        mv = _search_targets(ids, w, cur, tgt, p, feasible)
    res = _result(books, order, tuple(start[i] for i in ids), tuple(tgt[i] for i in ids), mv, p)
    return res | {"pack_left": True, "anchored": dict(anchors), "notes": notes}


def _result(books, order, start, final, moves, p):
    """
    Plan dict: moves (the last move of each book is "definitiva"), counts, time estimates, positions
    """
    ids = [b["id"] for b in books]
    last = {}
    for k, (i, _a, _b) in enumerate(moves):
        last[i] = k
    out = []
    for k, (i, a, b) in enumerate(moves):
        out.append({"book": ids[i], "from_y": a, "to_y": b,
                    "kind": "definitiva" if last[i] == k else "parcheggio"})
    n = len(books)
    return {
        "order": list(order),
        "moves": out,
        "n_moves": len(out),
        "parking_moves": sum(1 for m in out if m["kind"] == "parcheggio"),
        "books_moved": len({m["book"] for m in out}),
        "baseline_moves": 0 if not out else 2 * n,           # remove all + reinsert (old planner)
        "estimated_seconds": round(len(out) * p["sec_per_move"]),
        "baseline_seconds": round((0 if not out else 2 * n) * p["sec_per_move"]),
        "start": {ids[i]: start[i] for i in range(n)},
        "final": {ids[i]: final[i] for i in range(n)},
        "method": "A* esatto (minimo numero di prese)",
    }


def describe(plan, books):
    """
    Human-readable plan text for the CLI
    """
    by = {b["id"]: b for b in books}
    name = lambda i: f"[{i}] {by[i].get('title') or 'libro ' + str(i)}"
    lines = [f"Ordine richiesto (sinistra -> destra): {', '.join(name(i) for i in plan['order'])}"]
    if not plan["moves"]:
        lines.append("Gia' in ordine: nessuna presa.")
    for k, m in enumerate(plan["moves"], 1):
        lines.append(f"  {k}. {name(m['book'])}: y {m['from_y']:+.3f} -> {m['to_y']:+.3f}"
                     + ("  (parcheggio)" if m["kind"] == "parcheggio" else ""))
    lines.append(f"Prese: {plan['n_moves']} (di cui {plan['parking_moves']} di parcheggio) invece di "
                 f"{plan['baseline_moves']} (rimuovi tutto e reinserisci); circa "
                 f"{plan['estimated_seconds'] / 60:.0f} min invece di {plan['baseline_seconds'] / 60:.0f}.")
    return "\n".join(lines)


def load_books(path="/tmp/x2_library.json"):
    """
    Books from the "library" section written by library_pipeline; raises if missing or empty
    """
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    lib = data.get("library") or []
    if not lib:
        raise ValueError(f"{path}: nessuna sezione 'library' (rilancia la pipeline con la versione nuova)")
    return [{"id": b["id"], "y": float(b["shelf_y"]), "width": float(b["thickness_mm"]) / 1000.0,
             "title": b.get("title") or b.get("ocr_title") or "", "author": b.get("author") or "",
             "year": b.get("year"), "isbn": b.get("isbn")} for b in lib]


def main(argv=None):
    """
    CLI: print the reorder plan for a criterion
    """
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--library", default="/tmp/x2_library.json")
    ap.add_argument("--demo", action="store_true", help="use the 4 demo books")
    ap.add_argument("--by", default="author", choices=list(CRITERIA))
    ap.add_argument("--desc", action="store_true")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    books = DEMO_BOOKS if a.demo else load_books(a.library)
    order, warn = order_books(books, a.by, not a.desc)
    plan = plan_reorder(books, order)
    plan["warnings"] = warn
    print(json.dumps(plan, indent=1, ensure_ascii=False) if a.json else describe(plan, books))
    for w_ in warn:
        print("WARNING:", w_, file=sys.stderr)


if __name__ == "__main__":
    main()
