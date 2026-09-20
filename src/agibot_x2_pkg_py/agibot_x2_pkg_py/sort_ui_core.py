"""
sort_ui_core - logica della pagina di riordino (senza Streamlit, provabile da sola):
frase libera -> criterio, lettura dei libri, piano, disegno SVG dello scaffale e
pubblicazione ROS del piano. Usata da sort_ui.py (Streamlit).
"""
import html
import json
import re
import threading
import time
import unicodedata

from agibot_x2_pkg_py.reorder_planner import DEMO_BOOKS, load_books, order_books, plan_pack_left, plan_reorder

PLAN_FILE = "/tmp/x2_reorder_plan.json"
TOPIC = "/library_manager/reorder_plan"

DIRECTION_LABELS = {
    "title": ("A → Z", "Z → A"),
    "author": ("A → Z", "Z → A"),
    "year": ("Meno recente prima", "Più recente prima"),
    "size": ("Più sottile prima", "Più spesso prima"),
}


def parse_text(text):
    """Frase libera ("ordina per autore dalla Z alla A") -> {"criterion", "ascending"} o None."""
    n = unicodedata.normalize("NFKD", text or "").encode("ascii", "ignore").decode().casefold()
    n = re.sub(r"[^a-z0-9 ]+", " ", n)
    crit = None
    for key, words in (("author", ("autor", "scrittor")), ("title", ("titol", "nome", "alfabet")),
                       ("year", ("anno", "data", "cronolog", "pubblicaz", "recente")),
                       ("size", ("spessor", "dimension", "grandezz", "largh", "sottil", "grosso", "grossi"))):
        if any(w in n for w in words):
            crit = key
            break
    if crit is None:
        return None
    desc = any(w in n for w in ("invers", "decresc", "dalla z", "z alla a", "piu recente", "piu spess", "piu grand",
                                "piu grossi", "piu nuov", "dall ultimo", "contrario"))
    if "meno recente" in n or "piu vecch" in n or "dal piu piccolo" in n or "piu sottil" in n or "crescent" in n:
        desc = False
    return {"criterion": crit, "ascending": not desc}


def get_books(path):
    """(libri, demo, sorgente_o_errore): i dati veri se la pipeline ha scritto la sezione library."""
    try:
        return load_books(path), False, path
    except Exception as e:
        return [dict(b) for b in DEMO_BOOKS], True, str(e)


def make_plan(books, criterion, ascending, pack_left=True):
    """pack_left (default, richiesto dall'utente): i libri finiscono IMPACCHETTATI A SINISTRA nel
    primo scaffale, in ordine da sinistra a destra (plan_pack_left); False = minimo numero di
    prese lasciando i libri dove possibile (plan_reorder)."""
    order, warn = order_books(books, criterion, ascending)
    plan = plan_pack_left(books, order) if pack_left else plan_reorder(books, order)
    plan["warnings"] = warn
    plan["criterion"], plan["ascending"], plan["pack_left"] = criterion, ascending, bool(pack_left)
    return plan


def fmt_pos(y):
    return "al centro" if abs(y * 100) < 0.5 else f"{abs(y * 100):.0f} cm a {'sinistra' if y >= 0 else 'destra'}"


def fmt_min(seconds):
    m = round(seconds / 60)
    return f"{m // 60} h {m % 60} min" if m >= 60 else f"{m} min"


def shelf_svg(books, pos, moved=None, W=760, H=170, Y0=0.38):
    """Scaffale visto dal robot (sinistra = y positivo). moved: {id: numero_della_mossa}."""
    moved = moved or {}
    sc, base = W / (2 * Y0), 132
    out = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="scaffale" '
           f'style="width:100%;height:auto">',
           f'<rect x="0" y="{base}" width="{W}" height="9" rx="3" fill="#b39c76"/>']
    for b in books:
        w = max(b["width"] * sc, 18)
        x = (Y0 - pos[b["id"]]) * sc - w / 2
        h = 92 + (b["id"] * 13 % 24)
        y = base - h
        m = moved.get(b["id"])
        title = b.get("title") or f"libro {b['id']}"
        mx = int(h // 7)
        title = title if len(title) <= mx else title[:mx - 1] + "…"
        out.append(f'<rect x="{x:.1f}" y="{y}" width="{w:.1f}" height="{h}" rx="3" '
                   f'fill="hsl({(b["id"] * 67 + 140) % 360} 38% 42%)" '
                   f'stroke="{"#2f9e7a" if m else "rgba(0,0,0,.3)"}" stroke-width="{4 if m else 1}"/>')
        out.append(f'<text transform="translate({x + w / 2 + 4:.1f},{y + h - 8}) rotate(-90)" font-size="12" '
                   f'fill="#fff">{html.escape(title)}</text>')
        out.append(f'<text x="{x + w / 2:.1f}" y="{base + 26}" text-anchor="middle" font-size="12" '
                   f'fill="currentColor" opacity=".6">{b["id"]}</text>')
        if m:
            out.append(f'<circle cx="{x + w / 2:.1f}" cy="{y - 12}" r="11" fill="#2f9e7a"/>'
                       f'<text x="{x + w / 2:.1f}" y="{y - 8}" text-anchor="middle" font-size="12" '
                       f'font-weight="700" fill="#fff">{m}</text>')
    out.append("</svg>")
    return "".join(out)


class RosPublisher:
    """Un solo nodo per tutta la vita dell'app: il messaggio latched resta valido finche' e' su."""

    def __init__(self):
        self.lock, self.ready, self.err, self.pub = threading.Lock(), False, None, None

    def _init(self):
        try:
            import rclpy
            from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
            from std_msgs.msg import String
            rclpy.init()
            self.node = rclpy.create_node("sort_ui")
            self.pub = self.node.create_publisher(
                String, TOPIC, QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                          durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.String, self.rclpy, self.ready = String, rclpy, True
        except Exception as e:
            self.err = str(e)

    def publish(self, text):
        with self.lock:
            if not self.ready and self.err is None:
                self._init()
            if not self.ready:
                return False, (f"ROS non disponibile in questo terminale ({self.err}): "
                               "lancia l'app dopo `source install/setup.bash`")
            self.pub.publish(self.String(data=text))
            self.rclpy.spin_once(self.node, timeout_sec=0.2)
            return True, f"pubblicato su {TOPIC}"


def send_plan(books, plan, demo, ros):
    """Salva il piano in PLAN_FILE e lo pubblica. Ritorna (pubblicato, messaggio)."""
    doc = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "demo": demo, "books": books, "plan": plan}
    with open(PLAN_FILE, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    ok, note = ros.publish(json.dumps(doc, ensure_ascii=False))
    return ok, f"Piano salvato in {PLAN_FILE}. {note}"
