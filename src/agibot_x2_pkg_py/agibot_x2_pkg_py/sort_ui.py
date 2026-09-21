#!/usr/bin/env python3
"""
Streamlit app to choose how to reorder the library and preview the minimum-pick plan before sending it.
Reads the books from /tmp/x2_library.json (4 demo books if missing); "send" saves /tmp/x2_reorder_plan.json
and publishes it on /library_manager/reorder_plan (needs ROS sourced).
  ros2 run agibot_x2_pkg_py sort_ui                # then open http://localhost:8501
  streamlit run src/agibot_x2_pkg_py/agibot_x2_pkg_py/sort_ui.py   # equivalent
"""
import os
import sys

DEFAULT_LIBRARY = "/tmp/x2_library.json"


def run_app():
    """
    Build the Streamlit page: criterion choice, plan preview and send button
    """
    import json
    import streamlit as st
    from agibot_x2_pkg_py.reorder_planner import CRITERIA
    from agibot_x2_pkg_py import sort_ui_core as core

    st.set_page_config(page_title="Riordino libreria", page_icon="📚", layout="centered")

    @st.cache_resource
    def ros_publisher():
        """
        ROS publisher shared across Streamlit reruns
        """
        return core.RosPublisher()

    @st.cache_data(show_spinner=False)
    def cached_plan(books_json, crit, asc, pack_left):
        """
        Plan cached per (books, criterion, direction, packing)
        """
        return core.make_plan(json.loads(books_json), crit, asc, pack_left)

    # state: the "Capito" button updates it in a callback, before the widgets are created
    st.session_state.setdefault("crit", "author")
    st.session_state.setdefault("asc", True)

    def on_parse():
        """
        Parse the free-text request into criterion and direction
        """
        r = core.parse_text(st.session_state.get("free", ""))
        if r is None:
            st.session_state["parse_err"] = "Non ho capito il criterio: prova con titolo, autore, anno o spessore."
        else:
            st.session_state.update(crit=r["criterion"], asc=r["ascending"], parse_err=None)

    def on_crit():
        """
        Reset the direction to ascending when the criterion changes
        """
        st.session_state["asc"] = True

    st.title("📚 Riordino della libreria")
    st.caption("Scegli come ordinare i libri: vedi subito quante prese servono e come resta lo scaffale, "
               "prima che il robot si muova.")

    lib_path = st.sidebar.text_input("File dei libri", os.environ.get("X2_LIBRARY", DEFAULT_LIBRARY))
    st.sidebar.caption("Lo scrive `library_pipeline` (sezione `library`). Dopo una nuova prova ricarica la pagina.")
    books, demo, src = core.get_books(lib_path)
    if demo:
        st.info(f"**Dati di prova** (4 libri della simulazione): {src}. Esegui la pipeline per i dati veri.", icon="ℹ️")

    st.subheader("Come vuoi ordinarli?")
    st.radio("Criterio", list(CRITERIA), format_func=lambda k: CRITERIA[k][0], key="crit", horizontal=True,
             on_change=on_crit)
    labels = core.DIRECTION_LABELS[st.session_state["crit"]]
    st.radio("Verso", [True, False], format_func=lambda v: labels[0] if v else labels[1], key="asc", horizontal=True)
    c1, c2 = st.columns([4, 1])
    c1.text_input("Oppure scrivi", key="free", placeholder="es. ordina per autore dalla Z alla A",
                  label_visibility="collapsed")
    c2.button("Capito", on_click=on_parse, use_container_width=True)
    err = st.session_state.pop("parse_err", None)      # shown only once, right after "Capito"
    if err:
        st.warning(err)

    pack_left = st.checkbox("Impacchetta i libri a sinistra, partendo dall'estremo sinistro dello scaffale",
                            value=True, key="pack_left",
                            help="Spento: il minimo numero di prese, lasciando fermi i libri già nell'ordine giusto.")
    try:
        plan = cached_plan(json.dumps(books), st.session_state["crit"], st.session_state["asc"], bool(pack_left))
    except ValueError as e:
        st.error(str(e))
        return

    st.subheader("Risultato")
    m1, m2, m3 = st.columns(3)
    m1.metric("Prese", plan["n_moves"],
              f"{plan['parking_moves']} di parcheggio" if plan["parking_moves"] else None, delta_color="off")
    m2.metric("Tempo stimato", core.fmt_min(plan["estimated_seconds"]))
    m3.metric("Col vecchio piano", f"{plan['baseline_moves']} prese", core.fmt_min(plan["baseline_seconds"]),
              delta_color="off")

    by = {b["id"]: b for b in books}
    last = {m["book"]: i + 1 for i, m in enumerate(plan["moves"]) if m["kind"] == "definitiva"}
    st.markdown("**Scaffale adesso** (sinistra → destra guardando lo scaffale; posizioni dal centro)")
    st.markdown(core.shelf_svg(books, plan["start"]), unsafe_allow_html=True)
    st.markdown("**Dopo il riordino**")
    st.markdown(core.shelf_svg(books, plan["final"], last), unsafe_allow_html=True)

    if plan["moves"]:
        for i, m in enumerate(plan["moves"], 1):
            t = by[m["book"]].get("title") or f"libro {m['book']}"
            st.markdown(f"{i}. **{t}**: da **{core.fmt_pos(m['from_y'])}** a **{core.fmt_pos(m['to_y'])}**"
                        + (" · _parcheggio_" if m["kind"] == "parcheggio" else ""))
    else:
        st.success("Sono già nell'ordine richiesto: nessuna presa.")
    for w in plan.get("warnings", []):
        st.warning(w)

    if st.button("Invia il piano al robot", type="primary"):
        ok, msg = core.send_plan(books, plan, demo, ros_publisher())
        (st.success if ok else st.warning)(msg)
    st.caption("Il robot esegue il piano solo se `reorder_executor` è in ascolto "
               "(`ros2 run agibot_x2_pkg_py reorder_executor --wait`): «Invia» salva il piano in "
               "`/tmp/x2_reorder_plan.json` e lo pubblica su `/library_manager/reorder_plan`. Ogni presa "
               "richiede diversi minuti reali con il simulatore lento. I libri restano sempre in mano o sullo "
               "scaffale, mai sul tavolo.")

    with st.expander("Libri letti"):
        st.dataframe([{"#": b["id"], "Titolo": b.get("title") or "—", "Autore": b.get("author") or "—",
                       "Anno": b.get("year") or "—", "Spessore (mm)": round(b["width"] * 1000)} for b in books],
                     hide_index=True, use_container_width=True)


def main(argv=None):
    """
    Entry point of `ros2 run agibot_x2_pkg_py sort_ui`: launch Streamlit on this file
    """
    import argparse
    import subprocess
    ap = argparse.ArgumentParser(description="Streamlit app to reorder the library")
    ap.add_argument("--port", type=int, default=8501)
    ap.add_argument("--library", default=DEFAULT_LIBRARY)
    a = ap.parse_args(argv)
    env = dict(os.environ, X2_LIBRARY=a.library)
    cmd = [sys.executable, "-m", "streamlit", "run", os.path.abspath(__file__), "--server.port", str(a.port),
           "--server.address", "0.0.0.0", "--server.headless", "true", "--browser.gatherUsageStats", "false"]
    print(f"sort_ui: open http://localhost:{a.port}  (data: {a.library})")
    raise SystemExit(subprocess.call(cmd, env=env))


if __name__ == "__main__":       # run by `streamlit run`
    run_app()
