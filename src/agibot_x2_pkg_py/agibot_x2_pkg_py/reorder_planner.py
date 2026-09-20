#!/usr/bin/env python3
"""
reorder_planner - riordino dello scaffale con il MINIMO numero di prese
(2026-09-19).

Perche': con la simulazione a RTF 0.05 ogni presa+rimessa costa ~15-20 minuti
reali, quindi conta il NUMERO DI PRESE, non la complessita' dell'algoritmo di
ordinamento (con 4 libri e' irrilevante). Il pianificatore precedente
(scripts/sorting/sort_planner.py) toglie TUTTI i libri e li rimette in ordine:
2n prese, con parcheggio sul tavolo (vietato per i libri: restano in mano o
sullo scaffale).

Modello (fisico, non a slot uguali):
  * ogni libro e' un intervallo sull'asse y dello scaffale: centro y, larghezza
    w (spessore del dorso); fra due libri servono almeno `gap` metri liberi
    (il palmo della pinza, 10 cm, deve entrare: 5.9 cm, vedi book_placer);
  * una mossa = prendere UN libro e metterlo in un punto libero dello scaffale
    (anche lo spazio libero dove stavano gli oggetti: e' il "buffer");
  * obiettivo: ordine lungo y uguale a quello richiesto (da sinistra a destra
    guardando lo scaffale = y decrescente), spaziature libere.

Algoritmo: A* sugli stati (posizioni dei libri) con costo 1 + 0.1*|dy| per
mossa (prima il numero di prese, poi la distanza) e euristica
h = n - LIS, dove LIS e' la piu' lunga sottosequenza di libri gia' nell'ordine
relativo giusto: almeno n - LIS libri devono muoversi, e una mossa aumenta la
LIS al piu' di 1 -> h ammissibile e consistente, la soluzione e' OTTIMA per
numero di mosse (fra le posizioni candidate: griglia di `grid` metri piu' le
posizioni di partenza). Un libro che si muove piu' di una volta e' un
parcheggio (serve quando i vuoti sono troppo stretti per inserirlo subito).

Uso:
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

SHELF_HALF_INNER = 0.378                  # meta' larghezza interna (book_placer.SHELF_HALF_INNER_WIDTH)
DEFAULTS = dict(
    # Centri di slot NON raggiungibili da nessun braccio (mappa IK del 2026-09-19, libri profondi 16 cm,
    # uscita di 20 cm): la mano che si ritira davanti alla spalla sinistra (y ~ +0.15) non ha spazio.
    # Dry-run: y 0.104/0.15/0.2 falliscono con entrambe le braccia (ritirata 75-100 mm di errore),
    # 0.05 e 0.25 vanno. Un libro non puo' avere qui ne' la presa ne' la posa.
    unreachable=((0.075, 0.215),),
    gap=0.059,                # spazio libero minimo fra due libri (palmo della pinza)
    y_range=(-0.32, 0.32),    # dove il braccio arriva bene (mappamondo a +0.28 e Hunger a -0.28 presi)
    wall=0.03,                # margine dal bordo interno del mobile (sul lato del libro)
    grid=0.02,                # passo delle posizioni candidate
    sec_per_move=1100.0,      # secondi REALI a presa+rimessa (RTF 0.05, misurato: 700-1400 s)
    max_expansions=400000,
)

# libri veri del giro 2026-09-19 (y decrescente = da sinistra a destra), per provare offline
DEMO_BOOKS = [
    {"id": 1, "y": 0.047, "width": 0.038, "title": "L'alba della mietitura", "author": "Suzanne Collins", "year": 2020},
    {"id": 2, "y": -0.053, "width": 0.044, "title": "La ballata dell'usignolo e del serpente", "author": "Suzanne Collins", "year": 2020},
    {"id": 3, "y": -0.162, "width": 0.054, "title": "IT", "author": "Stephen King", "year": 1986},
    {"id": 4, "y": -0.283, "width": 0.070, "title": "Hunger Games", "author": "Suzanne Collins", "year": 2008},
]

_ARTICLES = {"il", "lo", "la", "l", "i", "gli", "le", "un", "uno", "una", "the", "a", "an"}


# ─── criteri di ordinamento ─────────────────────────────────────────────────
def _norm(s):
    s = unicodedata.normalize("NFKD", str(s or "")).encode("ascii", "ignore").decode()
    return re.sub(r"[^a-z0-9 ]+", " ", s.casefold()).strip()


def title_key(t):
    words = _norm(t).split()
    while words and words[0] in _ARTICLES and len(words) > 1:
        words = words[1:]
    return " ".join(words) or None


def author_key(a):
    first = re.split(r"[,;&]| e | and ", str(a or ""))[0]
    words = _norm(first).split()
    return (words[-1], " ".join(words[:-1])) if words else None      # cognome, poi nome


def year_key(y):
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


def order_books(books, criterion, ascending=True):
    """(ids nell'ordine richiesto da sinistra a destra, avvisi). I libri senza il
    dato richiesto vanno in fondo (nell'ordine attuale), con un avviso."""
    if criterion not in CRITERIA:
        raise ValueError(f"criterio '{criterion}' sconosciuto: {list(CRITERIA)}")
    _label, key = CRITERIA[criterion]
    have = [b for b in books if key(b) is not None]
    miss = sorted([b for b in books if key(b) is None], key=lambda b: -b["y"])
    have.sort(key=lambda b: -b["y"])                                  # tie-break: ordine attuale (stabile)
    have.sort(key=key, reverse=not ascending)
    warn = [f"libro {b['id']} senza {CRITERIA[criterion][0].lower()}: messo in fondo" for b in miss]
    return [b["id"] for b in have + miss], warn


# ─── pianificazione ─────────────────────────────────────────────────────────
def _lis(seq):
    best = [1] * len(seq)
    for i in range(len(seq)):
        for j in range(i):
            if seq[j] < seq[i]:
                best[i] = max(best[i], best[j] + 1)
    return max(best, default=0)


def plan_reorder(books, order, **kw):
    """books: [{id, y, width, ...}], order: ids da sinistra a destra (y decrescente).
    Ritorna un dict con le mosse ottime. Solleva ValueError se impossibile."""
    p = {**DEFAULTS, **kw}
    ids = [b["id"] for b in books]
    n = len(ids)
    w = [float(b["width"]) for b in books]
    rank = [order.index(i) for i in ids]
    gap, y_lo, y_hi = p["gap"], p["y_range"][0], p["y_range"][1]
    start = tuple(round(float(b["y"]), 4) for b in books)

    def fits(i, y, state):
        if not (y_lo - 1e-9 <= y <= y_hi + 1e-9) or abs(y) + w[i] / 2 + p["wall"] > SHELF_HALF_INNER:
            return False
        return all(j == i or abs(y - state[j]) >= (w[i] + w[j]) / 2 + gap - 1e-9 for j in range(n))

    def h(state):
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
                if y == state[i] or not fits(i, y, state):
                    continue
                ns = state[:i] + (y,) + state[i + 1:]
                ng = g + 1.0 + 0.1 * abs(y - state[i])
                if ng < best.get(ns, 1e9) - 1e-12:
                    best[ns] = ng
                    parent[ns] = (state, (i, state[i], y))
                    heapq.heappush(heap, (ng + h(ns), next(tick), ng, ns, None))
    raise ValueError("nessun riordino possibile con questi spazi (serve piu' spazio libero sullo scaffale)")


def pack_left_targets(books, order, **kw):
    """Posizioni finali dei libri IMPACCHETTATI A SINISTRA (2026-09-19): il primo dell'ordine
    richiesto sta all'estremo sinistro dello scaffale (y massimo raggiungibile), gli altri
    seguono verso destra (y decrescente) con `gap` libero fra un libro e il successivo (lo
    stesso spazio che serve alle dita per posare/prendere). Ritorna {id: y_centro}."""
    p = {**DEFAULTS, **kw}
    by = {b["id"]: b for b in books}
    y_hi = min(p["y_range"][1], SHELF_HALF_INNER - p["wall"] - float(by[order[0]]["width"]) / 2.0)
    out, y, prev_w = {}, y_hi, None
    for i in order:
        w = float(by[i]["width"])
        y = y_hi if prev_w is None else y - prev_w / 2.0 - p["gap"] - w / 2.0
        for lo, hi in p["unreachable"]:                 # salta la fascia irraggiungibile (buco nello scaffale)
            if lo <= y <= hi:
                y = min(y, lo - 0.075)                  # sotto la fascia, dove il sinistro arriva (mappa: 0.0 e' LR)
        out[i] = round(y, 4)
        prev_w = w
    low = min(out[i] - float(by[i]["width"]) / 2.0 for i in out)
    if low < -SHELF_HALF_INNER + p["wall"]:
        raise ValueError("i libri non stanno nello scaffale impacchettati a sinistra con questo spazio")
    return out


def plan_pack_left(books, order, **kw):
    """Piano per portare i libri, nell'ordine `order` (sinistra -> destra), alle posizioni di
    pack_left_targets. Le mosse (una presa ciascuna) vengono ordinate in modo che la
    destinazione sia LIBERA al momento della mossa (nessuna sovrapposizione con i libri non
    ancora spostati, spazio `gap` libero); se serve, un libro che blocca viene prima parcheggiato
    in un punto libero dello scaffale (mossa "parcheggio"). Nessun libro va mai sul tavolo."""
    p = {**DEFAULTS, **kw}
    ids = [b["id"] for b in books]
    w = {b["id"]: float(b["width"]) for b in books}
    cur = {b["id"]: round(float(b["y"]), 4) for b in books}
    tgt = pack_left_targets(books, order, **kw)
    start = dict(cur)
    gap, tol = p["gap"], 0.005

    def free(i, y):
        return (abs(y) + w[i] / 2 + p["wall"] <= SHELF_HALF_INNER + 1e-9 and
                not any(lo <= y <= hi for lo, hi in p["unreachable"]) and
                all(j == i or abs(y - cur[j]) >= (w[i] + w[j]) / 2 + gap - 1e-9 for j in ids))

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
        # tutti bloccati: parcheggia il primo bloccante in un punto libero (il piu' vicino)
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
    return _result(books, order, tuple(start[i] for i in ids), tuple(cur[i] for i in ids),
                   [(k, m["from_y"], m["to_y"]) for k, m in enumerate(moves)] and
                   [(ids.index(m["book"]), m["from_y"], m["to_y"]) for m in moves], p) | {"pack_left": True}


def _result(books, order, start, final, moves, p):
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
        "baseline_moves": 0 if not out else 2 * n,           # rimuovi tutto + reinserisci (piano precedente)
        "estimated_seconds": round(len(out) * p["sec_per_move"]),
        "baseline_seconds": round((0 if not out else 2 * n) * p["sec_per_move"]),
        "start": {ids[i]: start[i] for i in range(n)},
        "final": {ids[i]: final[i] for i in range(n)},
        "method": "A* esatto (minimo numero di prese)",
    }


def describe(plan, books):
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
    """Libri dalla sezione "library" scritta da library_pipeline (id, shelf_y,
    thickness_mm, titolo/autore/anno). Solleva se manca o e' vuota."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    lib = data.get("library") or []
    if not lib:
        raise ValueError(f"{path}: nessuna sezione 'library' (rilancia la pipeline con la versione nuova)")
    return [{"id": b["id"], "y": float(b["shelf_y"]), "width": float(b["thickness_mm"]) / 1000.0,
             "title": b.get("title") or b.get("ocr_title") or "", "author": b.get("author") or "",
             "year": b.get("year"), "isbn": b.get("isbn")} for b in lib]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--library", default="/tmp/x2_library.json")
    ap.add_argument("--demo", action="store_true", help="usa i 4 libri di prova")
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
        print("AVVISO:", w_, file=sys.stderr)


if __name__ == "__main__":
    main()
