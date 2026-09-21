#!/usr/bin/env python3
"""
Phone-format Streamlit reorder page for library users (same logic, book file and plan sending as sort_ui.py).
Never shows the user how long the reorder takes; progress is read from /tmp/x2_reorder_status.json (reorder_executor).
URL params: ?cornice=0 removes the phone frame, ?tecnico=1 adds free text, lists and technical details.
X2_UI_INVIO=0: demo mode, "Avvia" sends nothing to the robot.
  X2_UI_INVIO=0 PYTHONPATH=src/agibot_x2_pkg_py python -m streamlit run src/agibot_x2_pkg_py/agibot_x2_pkg_py/sort_ui_utente.py
"""
import html
import json
import os
import sys
import time

DEFAULT_LIBRARY = "/tmp/x2_library.json"
STATUS_FILE = "/tmp/x2_reorder_status.json"
STATUS_FRESH_S = 45 * 60          # robot status shown only if updated in the last 45 min

# criterion -> (name, (direction 1, direction 2))
CHOICES = {
    "title": ("Titolo", ("Da A a Z", "Da Z ad A")),
    "author": ("Autore", ("Da A a Z", "Da Z ad A")),
    "year": ("Anno", ("Dal più vecchio", "Dal più recente")),
    "size": ("Spessore", ("Dal più sottile", "Dal più spesso")),
}

CSS = """
<style>
header[data-testid="stHeader"], [data-testid="stToolbar"], [data-testid="stStatusWidget"], #MainMenu, footer
   {display:none !important;}
.block-container, [data-testid="stMainBlockContainer"] {
   max-width: 396px !important; margin: 1rem auto !important; position: relative;
   padding: 2.4rem 1.05rem 1.4rem !important; border: 9px solid #3a3a3f; border-radius: 42px;
   box-shadow: 0 10px 34px rgba(0,0,0,.35);}
.block-container::before, [data-testid="stMainBlockContainer"]::before {
   content:""; position:absolute; top:-1px; left:50%; transform:translateX(-50%);
   width:96px; height:17px; background:#3a3a3f; border-radius:0 0 14px 14px;}
/* Streamlit tira su ogni blocco di testo di 1rem (margine negativo): con HTML senza <p> copre l'elemento sopra */
[data-testid="stMarkdownContainer"], .stMarkdown {margin-bottom: 0 !important;}
[data-testid="stMarkdownContainer"] p {margin: 0 !important;}
[data-testid="stVerticalBlock"] {gap: .8rem !important;}
.hero h1 {font-size: 1.55rem; line-height:1.2; margin: 0 0 .3rem 0; padding: 0;}
.hero p {font-size: .95rem; opacity: .72; margin: 0;}
.sh {display:flex; align-items:center; gap:.6rem; margin: .9rem 0 0 0; font-size:1.1rem; font-weight:650; line-height:1.3;}
.sh .n {display:inline-flex; width:1.6rem; height:1.6rem; border-radius:50%; background:#2f9e7a; color:#fff;
   align-items:center; justify-content:center; font-weight:700; font-size:.85rem; flex:none;}
.card {border:1px solid rgba(128,128,128,.32); border-radius:14px; padding:.8rem 1rem; background:rgba(128,128,128,.07);
   font-size:1rem; line-height:1.45;}
.stButton button {min-height: 3rem; border-radius: 14px; font-weight: 650; font-size: 1rem;}
[data-testid="stSegmentedControl"] button {font-size:.92rem;}
.ord {margin:.1rem 0;} .ord b {display:inline-block; min-width:1.5rem;}
</style>
"""
CSS_NO_FRAME = """
<style>
.block-container, [data-testid="stMainBlockContainer"] {border:none !important; border-radius:0 !important;
   box-shadow:none !important; margin: 0 auto !important;}
.block-container::before, [data-testid="stMainBlockContainer"]::before {display:none !important;}
</style>
"""


def _short(text, n=46):
    """
    Text truncated to n characters with an ellipsis
    """
    text = (text or "").strip()
    return text if len(text) <= n else text[:n - 1].rstrip() + "…"


def short_title(text):
    """
    Title without its subtitle: "Hunger games. La trilogia" -> "Hunger games"
    """
    t = (text or "").strip()
    for sep in (". ", ": ", " - ", " — "):
        if sep in t:
            t = t.split(sep)[0].strip()
    return t


def wrap_title(text, per_line, lines=2):
    """
    Title wrapped on at most `lines` rows of `per_line` characters, with "…" if it does not fit
    """
    rows, cur = [], ""
    for w in (text or "").split():
        if cur and len(cur) + 1 + len(w) > per_line:
            rows.append(cur)
            cur = w
        else:
            cur = f"{cur} {w}".strip()
    if cur:
        rows.append(cur)
    cut = len(rows) > lines
    rows = rows[:lines]
    rows = [r if len(r) <= per_line else r[:per_line - 1] + "…" for r in rows]
    if cut and rows:
        rows[-1] = rows[-1][:per_line - 1].rstrip() + "…"
    return rows


def shelf_svg(books, pos, moved=None, numbered=False, uid="a", W=340, Y0=0.38):
    """
    SVG of the shelf as seen by the robot (left = positive y); moved: ids of the books the robot moves
    Sized for a phone column (1 drawing unit = 1 screen pixel); books are wider than real
    (not to scale) so the title is readable.
    """
    moved = set(moved or ())
    sc = W / (2 * Y0)
    hmax = 132
    top = 20                                   # room for the arrow above moved books
    base = top + hmax
    H = base + (44 if numbered else 20)
    order = {b["id"]: k + 1 for k, b in enumerate(sorted(books, key=lambda b: -pos[b["id"]]))}
    out = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="scaffale" '
           f'style="width:100%;height:auto;display:block"><defs>'
           f'<linearGradient id="wood{uid}" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#c9ad82"/>'
           f'<stop offset="1" stop-color="#a98a5c"/></linearGradient>'
           f'<linearGradient id="shine{uid}" x1="0" y1="0" x2="1" y2="0"><stop offset="0" stop-color="#fff" stop-opacity=".24"/>'
           f'<stop offset=".5" stop-color="#fff" stop-opacity="0"/><stop offset="1" stop-color="#000" stop-opacity=".24"/>'
           f'</linearGradient></defs>',
           f'<rect x="0" y="{top - 6}" width="{W}" height="{base - top + 6}" rx="8" fill="rgba(140,110,70,.09)"/>',
           f'<rect x="0" y="{base}" width="{W}" height="10" rx="3" fill="url(#wood{uid})"/>',
           f'<rect x="0" y="{base + 10}" width="{W}" height="3" fill="rgba(0,0,0,.16)"/>']
    for b in books:
        w = max(b["width"] * sc * 1.3, 34)
        x = (Y0 - pos[b["id"]]) * sc - w / 2
        h = 110 + (b["id"] * 13 % 23)
        y = base - h
        hue = (b["id"] * 67 + 140) % 360
        mv = b["id"] in moved
        out.append(f'<rect x="{x:.1f}" y="{y}" width="{w:.1f}" height="{h}" rx="3" fill="hsl({hue} 40% 40%)"/>')
        out.append(f'<rect x="{x:.1f}" y="{y}" width="{w:.1f}" height="{h}" rx="3" fill="url(#shine{uid})" '
                   f'stroke="{"#2f9e7a" if mv else "rgba(0,0,0,.35)"}" stroke-width="{3 if mv else 1}"/>')
        out.append(f'<rect x="{x + 3:.1f}" y="{y + 7}" width="{w - 6:.1f}" height="2" fill="rgba(255,255,255,.38)"/>')
        out.append(f'<rect x="{x + 3:.1f}" y="{y + h - 10}" width="{w - 6:.1f}" height="2" fill="rgba(255,255,255,.38)"/>')
        fs = 11
        rows = wrap_title(short_title(b.get("title")) or f"libro {b['id']}", max(int((h - 24) / (fs * 0.55)), 6))
        cx = x + w / 2
        first = cx - (0.3 if len(rows) == 2 else -0.34) * fs          # one row: centered; two rows: side by side
        for k, row in enumerate(rows):
            out.append(f'<text transform="translate({first + k * 1.15 * fs:.1f},{y + h - 13}) rotate(-90)" font-size="{fs}" '
                       f'fill="#fff" font-weight="600">{html.escape(row)}</text>')
        if numbered:
            out.append(f'<text x="{x + w / 2:.1f}" y="{base + 34}" text-anchor="middle" font-size="16" font-weight="700" '
                       f'fill="currentColor" opacity=".9">{order[b["id"]]}</text>')
        if mv:
            out.append(f'<circle cx="{x + w / 2:.1f}" cy="{y - 9}" r="8" fill="#2f9e7a"/>'
                       f'<text x="{x + w / 2:.1f}" y="{y - 4}" text-anchor="middle" font-size="12" font-weight="700" '
                       f'fill="#fff">→</text>')
    out.append("</svg>")
    return "".join(out)


def _title_of(books, bid, n=40):
    """
    Short title of book bid
    """
    b = next((x for x in books if x["id"] == bid), None)
    return _short((b or {}).get("title") or f"libro {bid}", n)


def status_is_recent(status_file=STATUS_FILE, max_age_s=STATUS_FRESH_S):
    """
    True if the executor status file was modified less than max_age_s ago
    """
    try:
        return time.time() - os.path.getmtime(status_file) < max_age_s
    except OSError:
        return False


def read_progress(books, status_file=STATUS_FILE):
    """
    (sentence, fraction 0..1, outcome) of the LAST run from the reorder_executor status, or None
    Sentences carry no times or durations: the user is never told how long it takes.
    """
    try:
        with open(status_file, encoding="utf-8") as f:
            hist = json.load(f).get("history") or []
    except Exception:
        return None
    starts = [i for i, h in enumerate(hist) if h.get("state") == "INIZIO"]
    if not starts:
        return None
    ev = hist[starts[-1]:]
    total = int(ev[0].get("mosse") or 0) or 1
    done = sum(1 for h in ev if h.get("state") == "MOSSA-OK")
    last = ev[-1]
    st_ = last.get("state")
    if st_ == "FINITO":
        return "Fatto! I libri sono stati riordinati.", 1.0, "ok"
    if st_ in ("FERMATO", "ERRORE"):
        # the reason is in the ERRORE event before FERMATO; the FERMATO "nota" is technical and ignored
        why = last.get("motivo") or next((h.get("motivo") for h in reversed(ev)
                                          if h.get("state") == "ERRORE" and h.get("motivo")), "")
        n = last.get("mossa") or (done + 1)
        if "portata" in why:
            why = "il robot non riesce ad arrivare in quel punto"
        bid = last.get("libro")
        who = f" con «{_title_of(books, int(bid), 24)}»" if bid not in (None, "") else ""
        return (f"Mi sono fermato allo spostamento {n} di {total}{who}" + (f": {why}" if why else "") +
                f". Gli altri {done} libri sono al loro posto."), done / total, "stop"
    mv = [h for h in ev if h.get("state") == "MOSSA"]
    if mv:
        m = mv[-1]
        if st_ == "MOSSA-OK":
            return f"«{_title_of(books, int(m['libro']), 28)}» è al suo posto.", done / total, "run"
        return (f"Sto spostando «{_title_of(books, int(m['libro']), 28)}» "
                f"(spostamento {str(m.get('n', '')).replace('/', ' di ')})."), done / total, "run"
    return f"Il robot ha ricevuto il piano ({total} spostamenti).", 0.0, "run"


def run_app(preview=False):
    """
    Build the phone-format Streamlit page
    """
    import streamlit as st
    from agibot_x2_pkg_py import sort_ui_core as core

    if not preview:
        st.set_page_config(page_title="Riordina la libreria", page_icon="📚", layout="centered",
                           initial_sidebar_state="collapsed")
    st.markdown(CSS, unsafe_allow_html=True)
    qp = st.query_params
    if qp.get("cornice") == "0":
        st.markdown(CSS_NO_FRAME, unsafe_allow_html=True)
    tecnico = qp.get("tecnico") == "1"
    invio_attivo = os.environ.get("X2_UI_INVIO", "1") != "0"

    @st.cache_resource
    def ros_publisher():
        """
        Single ROS node for the whole app lifetime
        The plan is a latched message, valid only while its publisher node is alive: a node per click
        was destroyed before the executor received it.
        """
        return core.RosPublisher()

    st.session_state.setdefault("u_crit", "title")
    st.session_state.setdefault("u_dir", True)

    def on_crit():
        """
        Reset the direction when the criterion changes
        """
        st.session_state["u_dir"] = True

    def on_words():
        """
        Parse the free-text request into criterion and direction
        """
        r = core.parse_text(st.session_state.get("u_free", ""))
        if r is None:
            st.session_state["u_words_err"] = "Non ho capito: prova con «per titolo», «per autore», «per anno» o «per spessore»."
        else:
            st.session_state.update(u_crit=r["criterion"], u_dir=r["ascending"], u_words_err=None)

    st.markdown('<div class="hero"><h1>📚 Riordina la libreria</h1>'
                '<p>Scegli come vuoi i libri: ci pensa il robot.</p></div>', unsafe_allow_html=True)

    lib_path = st.session_state.get("u_lib_path") or os.environ.get("X2_LIBRARY", DEFAULT_LIBRARY)
    books, demo, src = core.get_books(lib_path)
    if demo:
        st.info("Libri d'esempio: il robot non ha ancora letto la tua libreria.", icon="ℹ️")

    def head(n, title):
        """
        Numbered section header
        """
        st.markdown(f'<div class="sh"><span class="n">{n}</span>{title}</div>', unsafe_allow_html=True)

    # 1. the books
    head(1, "I tuoi libri, adesso")
    now = {b["id"]: b["y"] for b in books}
    st.markdown(shelf_svg(books, now, uid="ora"), unsafe_allow_html=True)
    if tecnico:
        with st.expander("Elenco dei libri"):
            for k, b in enumerate(sorted(books, key=lambda b: -b["y"]), 1):
                extra = ", ".join(str(x) for x in (b.get("author"), b.get("year")) if x)
                st.markdown(f"**{k}.** {b.get('title') or 'senza titolo'}" + (f" — {extra}" if extra else ""))

    # 2. how to reorder them
    head(2, "Come li vuoi?")
    crit = st.segmented_control("Criterio", list(CHOICES), format_func=lambda k: CHOICES[k][0], key="u_crit",
                                on_change=on_crit, label_visibility="collapsed") or "title"
    labels = CHOICES[crit][1]
    asc = st.segmented_control("Verso", [True, False], format_func=lambda v: labels[0] if v else labels[1], key="u_dir",
                               label_visibility="collapsed")
    asc = True if asc is None else asc
    pack = st.toggle("Tutti a sinistra, senza spazi vuoti", value=True, key="u_pack")
    if tecnico:
        with st.expander("Scrivilo con parole tue"):
            c1, c2 = st.columns([3, 1])
            c1.text_input("Cosa vuoi", key="u_free", placeholder="ordina per autore dalla Z alla A",
                          label_visibility="collapsed")
            c2.button("Capito", on_click=on_words, width="stretch")
            err = st.session_state.pop("u_words_err", None)
            if err:
                st.warning(err)

    try:
        plan = core.make_plan(books, crit, asc, bool(pack))
    except ValueError as e:
        st.error(f"Con questa scelta non riesco a trovare un ordine: {e}")
        return

    # 3. final shelf
    moving = sorted({m["book"] for m in plan["moves"]})
    head(3, "Ecco come sarà")
    st.markdown(shelf_svg(books, plan["final"], moved=moving, numbered=True, uid="dopo"), unsafe_allow_html=True)
    if moving:
        line = f"Il robot sposterà <b>{len(moving)}</b> {'libro' if len(moving) == 1 else 'libri'}."
        if len(moving) < len(books):
            line += " Gli altri restano dove sono."
        st.markdown(f'<div class="card">{line}</div>', unsafe_allow_html=True)
    else:
        st.success("Sono già in questo ordine: non c'è niente da spostare.")
    for w_ in plan.get("warnings", []):
        st.warning(w_)
    if tecnico:
        with st.expander("Ordine finale, da sinistra a destra"):
            for k, i in enumerate(sorted(plan["final"], key=lambda i: -plan["final"][i]), 1):
                st.markdown(f'<div class="ord"><b>{k}.</b> {html.escape(_title_of(books, i))}</div>', unsafe_allow_html=True)

    # start
    @st.dialog("Vuoi avviare il riordino?")
    def confirm():
        """
        Confirmation dialog that sends the plan
        """
        st.write(f"Il robot sposterà {len(moving)} {'libro' if len(moving) == 1 else 'libri'}.")
        st.checkbox("Il robot è acceso e sullo scaffale ci sono solo i libri.", key="u_ok")
        a, b_ = st.columns(2)
        if a.button("Sì, avvia", type="primary", disabled=not st.session_state.get("u_ok"), width="stretch"):
            if invio_attivo:
                ok, msg = core.send_plan(books, plan, demo, ros_publisher())
            else:
                ok, msg = False, "modalità dimostrativa: il piano non è stato inviato"
            st.session_state["u_sent"] = (ok, msg)
            st.rerun()
        if b_.button("Annulla", width="stretch"):
            st.rerun()

    if st.button("Avvia il riordino", type="primary", disabled=not moving, width="stretch"):
        confirm()
    sent = st.session_state.get("u_sent")
    if sent:
        if sent[0]:
            st.success("Piano inviato al robot.")
        else:
            st.warning("Il robot non è collegato in questo momento: non è partito niente.")

    # progress (only while the robot works)
    @st.fragment(run_every="6s")
    def progress():
        """
        Progress bar and message, refreshed every 6 s
        """
        if not (sent and sent[0]) and not status_is_recent() and qp.get("avanzamento") != "1":
            return
        p = read_progress(books)
        if p is None:
            return
        text, frac, state = p
        st.progress(min(max(frac, 0.0), 1.0))
        {"ok": st.success, "stop": st.warning}.get(state, st.info)(text)

    progress()

    # technical details: only with ?tecnico=1
    if tecnico:
        with st.expander("Dettagli tecnici"):
            st.text_input("File dei libri letti dal robot", value=lib_path, key="u_lib_path")
            st.caption(f"Sorgente dati: {src} · stima interna del riordino: {core.fmt_min(plan['estimated_seconds'])} "
                       f"(non mostrata all'utente)")
            st.dataframe([{"n.": b["id"], "titolo": b.get("title") or "—", "autore": b.get("author") or "—",
                           "anno": b.get("year") or "—", "spessore (mm)": round(b["width"] * 1000),
                           "posizione (m)": round(b["y"], 3)} for b in books], hide_index=True, width="stretch")
            st.dataframe([{"libro": _title_of(books, m["book"]), "da (m)": round(m["from_y"], 3),
                           "a (m)": round(m["to_y"], 3), "tipo": m["kind"]} for m in plan["moves"]],
                         hide_index=True, width="stretch")
            st.download_button("Scarica il piano (JSON)", data=json.dumps(plan, ensure_ascii=False, indent=1),
                               file_name="piano_riordino.json", mime="application/json")


def main(argv=None):
    """
    Entry point of `ros2 run agibot_x2_pkg_py sort_ui_utente`: launch Streamlit on this file
    """
    import argparse
    import subprocess
    ap = argparse.ArgumentParser(description="phone-format library reorder page")
    ap.add_argument("--port", type=int, default=8501)
    ap.add_argument("--library", default=DEFAULT_LIBRARY)
    a = ap.parse_args(argv)
    env = dict(os.environ, X2_LIBRARY=a.library)
    cmd = [sys.executable, "-m", "streamlit", "run", os.path.abspath(__file__), "--server.port", str(a.port),
           "--server.address", "0.0.0.0", "--server.headless", "true", "--browser.gatherUsageStats", "false",
           "--theme.primaryColor", "#2f9e7a"]
    print(f"sort_ui_utente: open http://localhost:{a.port}  (data: {a.library})")
    raise SystemExit(subprocess.call(cmd, env=env))


if __name__ == "__main__":       # run by `streamlit run`
    run_app()
