#!/usr/bin/env python3
"""
reorder_executor - esegue sul robot il piano di riordino scelto nell'app (2026-09-19).

  ros2 run agibot_x2_pkg_py reorder_executor --wait          # aspetta il piano da sort_ui ("Invia")
  ros2 run agibot_x2_pkg_py reorder_executor --plan /tmp/x2_reorder_plan.json   # lo esegue subito

Il piano (sort_ui_core.send_plan) e' {"books": [...], "plan": {"moves": [{book, from_y, to_y, kind}]}}
e arriva latched su /library_manager/reorder_plan (e in /tmp/x2_reorder_plan.json). Per ogni
mossa lancia `pick_test_book -p target:=<id> -p dest_y:=<to_y>`: il libro viene preso, sfilato,
traslato di lato DAVANTI allo scaffale e rimesso nello slot di destinazione, sempre in mano (mai
sul tavolo). Il braccio si sceglie fra sinistro e destro: prima quello dal lato della
destinazione; se pick_test_book esce con codice 3 (destinazione fuori portata, verificato PRIMA di
muoversi) si prova l'altro. Dopo ogni mossa la posizione reale del libro (Gazebo, se leggibile)
aggiorna il file delle rilevazioni e la planning scene. Alla prima mossa fallita ci si ferma.

Stato: /tmp/x2_reorder_status.json e /library_manager/reorder_status. Risultato finale:
/tmp/x2_library_riordinata.json (la sezione "library" con le nuove posizioni).
"""
import argparse
import json
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import os

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PLAN_TOPIC = "/library_manager/reorder_plan"
STATUS_TOPIC = "/library_manager/reorder_status"
DET_TOPIC = "/library_manager/detections"
GZ_NAMES = {1: "gt_alba", 2: "gt_ballata", 3: "gt_it", 4: "gt_hunger"}     # scena dei 4 libri veri
DEFAULT_DEPTH_BY_ID = {3: 0.012}      # IT (221 mm): a 18 mm la ritirata esce a 27 mm di errore laterale
STATUS_FILE = "/tmp/x2_reorder_status.json"
# Forme misurate dal rilevatore (depth) di mappamondo (obj5) e portapenne (obj6), 2026-09-19.
OBJ_SHAPES = json.loads(r'''{
 "5": {
  "id": 5,
  "class": "object",
  "is_book": false,
  "bbox": [
   544,
   810,
   647,
   964
  ],
  "confidence": 1.0,
  "color": "",
  "title": "",
  "author": "",
  "orientation": "unknown",
  "depth_m": 1.374,
  "shelf_row": 0,
  "shelf_slot": 0,
  "isbn": "",
  "year": "",
  "world_x": 0.2724,
  "world_y": 0.2799,
  "z_bottom": 0.9979,
  "z_top": 1.1064,
  "thickness_m": 0.066,
  "height_m": 0.1085,
  "length_m": 0.0344,
  "free_plus_m": 0.0651,
  "free_minus_m": 0.0685,
  "width_profile": [
   [
    1.0029,
    0.0431
   ],
   [
    1.0429,
    0.0286
   ],
   [
    1.0529,
    0.0472
   ],
   [
    1.0629,
    0.057
   ],
   [
    1.0729,
    0.0618
   ],
   [
    1.0829,
    0.0607
   ],
   [
    1.0929,
    0.0535
   ],
   [
    1.1029,
    0.0401
   ]
  ]
 },
 "6": {
  "id": 6,
  "class": "object",
  "is_book": false,
  "bbox": [
   734,
   833,
   800,
   948
  ],
  "confidence": 1.0,
  "color": "",
  "title": "",
  "author": "",
  "orientation": "unknown",
  "depth_m": 1.467,
  "shelf_row": 0,
  "shelf_slot": 1,
  "isbn": "",
  "year": "",
  "world_x": 0.3723,
  "world_y": 0.1534,
  "z_bottom": 0.9979,
  "z_top": 1.0889,
  "thickness_m": 0.05,
  "height_m": 0.091,
  "length_m": 0.0457,
  "free_plus_m": 0.0685,
  "free_minus_m": 0.0626,
  "width_profile": [
   [
    1.0029,
    0.0471
   ],
   [
    1.0129,
    0.047
   ],
   [
    1.0229,
    0.047
   ],
   [
    1.0329,
    0.047
   ],
   [
    1.0429,
    0.047
   ],
   [
    1.0529,
    0.047
   ],
   [
    1.0629,
    0.047
   ],
   [
    1.0729,
    0.047
   ],
   [
    1.0829,
    0.047
   ],
   [
    1.0929,
    0.0456
   ]
  ]
 }
}''')
# Mappa di raggiungibilita' della presa (dry_run IK del 2026-09-19, uscita di 20 cm): (libro, y) -> bracci
# che ci arrivano. Serve solo a scegliere l'ORDINE dei bracci senza perdere ~10 min di IK sul braccio
# sbagliato: se un punto non e' in mappa si prova comunque (exit 3 = fuori portata, nessun movimento).
REACH = {
    (1, 0.047): "R", (1, 0.32): "LR",
    (2, -0.053): "L", (2, 0.22): "LR",
    (4, -0.283): "LR", (4, 0.0): "LR", (4, 0.05): "R", (4, 0.104): "", (4, 0.15): "", (4, 0.2): "L",
    (4, 0.25): "LR", (4, 0.3): "LR",
    (3, -0.161): "R", (3, -0.018): "LR", (3, 0.05): "LR", (3, 0.11): "",
}


def reach(book, y):
    """Bracci ("L", "R", "LR", "") per (libro, y) dalla mappa, o None se non misurato (tol. 1 cm)."""
    for (b, yy), arms in REACH.items():
        if b == int(book) and abs(yy - float(y)) <= 0.01:
            return arms
    return None


def gz_pose(model):
    """(x, y, z) mondo del modello Gazebo, o None."""
    try:
        txt = subprocess.run(["gz", "topic", "-e", "-t", "/world/bookshelf_world/pose/info", "-n", "1"],
                             capture_output=True, text=True, timeout=60).stdout
        i = txt.find(f'name: "{model}"')
        if i < 0:
            return None
        m = re.search(r"position\s*\{([^}]*)\}", txt[i:i + 900])
        v = {k: float(x) for k, x in re.findall(r"(\w):\s*([-+0-9.e]+)", m.group(1))}
        return v.get("x", 0.0), v.get("y", 0.0), v.get("z", 0.0)
    except Exception:
        return None


def gz_book_y(model):
    """y reale (mondo) del modello Gazebo, o None."""
    try:
        txt = subprocess.run(["gz", "topic", "-e", "-t", "/world/bookshelf_world/pose/info", "-n", "1"],
                             capture_output=True, text=True, timeout=60).stdout
        i = txt.find(f'name: "{model}"')
        if i < 0:
            return None
        blk = txt[i:i + 900]
        m = re.search(r"position\s*\{([^}]*)\}", blk)
        return float(re.search(r"y:\s*([-+0-9.e]+)", m.group(1)).group(1))
    except Exception:
        return None


class ReorderExecutor(Node):

    def __init__(self, a):
        super().__init__("reorder_executor")
        self.a = a
        self.pub_status = self.create_publisher(String, STATUS_TOPIC, LATCHED)
        self.pub_det = self.create_publisher(String, DET_TOPIC, LATCHED)
        self.running = False
        self._js = {}
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self._js.update(dict(zip(m.name, m.position))), 10)
        self.log_lines = []
        self.depth_by_id = dict(DEFAULT_DEPTH_BY_ID)
        for kv in [x for x in a.depth_by_id.replace(" ", "").split(",") if ":" in x]:
            k, v = kv.split(":")
            self.depth_by_id[int(k)] = float(v)
        if not a.plan:
            self.create_subscription(String, PLAN_TOPIC, self._plan_cb, LATCHED)
            self.get_logger().info(f"reorder_executor: aspetto il piano su {PLAN_TOPIC} (app: 'Invia il piano')")

    # ─── stato ─────────────────────────────────────────────────────────────
    def status(self, state, **kw):
        doc = {"state": state, "time": time.strftime("%H:%M:%S"), **kw}
        self.log_lines.append(doc)
        with open(STATUS_FILE, "w", encoding="utf-8") as f:
            json.dump({"last": doc, "history": self.log_lines}, f, ensure_ascii=False, indent=1)
        self.pub_status.publish(String(data=json.dumps(doc, ensure_ascii=False)))
        self.get_logger().info(f"[{state}] " + ", ".join(f"{k}={v}" for k, v in kw.items()))

    def _plan_cb(self, msg):
        if self.running:
            self.get_logger().warn("un riordino e' gia' in corso: ignoro il nuovo piano")
            return
        try:
            doc = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"piano non valido: {e}")
            return
        self.run_plan(doc)

    # ─── rilevazioni ───────────────────────────────────────────────────────
    def _load_dets(self):
        with open(self.a.detections, encoding="utf-8") as f:
            return json.load(f)

    def _save_dets(self, dets):
        with open(self.a.detections, "w", encoding="utf-8") as f:
            json.dump(dets, f, ensure_ascii=False, indent=2)
        self.pub_det.publish(String(data=json.dumps(dets, ensure_ascii=False)))   # planning scene builder

    # ─── video ─────────────────────────────────────────────────────────────
    def _video_start(self):
        """Registra il riordino (regista + camera tavolo) dall'inizio del piano alla fine."""
        self.videos = []
        if not self.a.video_prefix:
            return
        base = ["ros2", "run", "agibot_x2_pkg_py", "record_video", "--ros-args"]
        self.videos.append(subprocess.Popen(base + ["-p", f"prefix:={self.a.video_prefix}"],
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        time.sleep(3.0)
        self.videos.append(subprocess.Popen(base + ["-p", f"prefix:={self.a.video_prefix}_tavolo",
                                                    "-p", "main_topic:=/video_camera_table/image",
                                                    "-p", "tcp_left_pip:=false", "-p", "head_pip:=false"],
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        time.sleep(8.0)
        self.get_logger().info(f"video: registro {self.a.video_prefix}* in /home/robot/SmartRobotics/videos/")

    def _video_stop(self):
        import os
        import signal
        time.sleep(15.0)              # ultimi secondi di scena dopo l'ultima mossa
        # SIGINT a TUTTO il gruppo del processo (2026-09-20): `ros2 run` non lo inoltra a record_video, che
        # restava vivo e i .mp4 senza "moov atom" (illeggibili) finche' non lo si fermava a mano.
        for pr in getattr(self, "videos", []):
            try:
                os.killpg(os.getpgid(pr.pid), signal.SIGINT)
            except Exception:
                pass
        for pr in getattr(self, "videos", []):
            try:
                pr.wait(timeout=300)
            except Exception:
                pr.kill()
        self.videos = []

    # ─── esecuzione ────────────────────────────────────────────────────────
    def run_plan(self, doc):
        self.running = True
        video_on = False
        try:
            if doc.get("demo") and not self.a.allow_demo:
                self.status("RIFIUTATO", motivo="il piano e' fatto con i libri di prova (demo), non con quelli letti")
                return False
            moves = (doc.get("plan") or {}).get("moves") or []
            if not moves:
                self.status("FINITO", esito="niente da spostare")
                return True
            self.status("INIZIO", mosse=len(moves))
            self._video_start()
            video_on = True
            for k, m in enumerate(moves, 1):
                if not self.do_move(k, len(moves), m):
                    self.status("FERMATO", mossa=k, libro=m["book"], nota="il libro potrebbe essere in mano: controllare")
                    return False
            self._write_final(doc)
            self.status("FINITO", esito="riordino completato")
            return True
        finally:
            if video_on:
                self._video_stop()
            self.running = False

    def _arms(self, book, src_y, dst_y):
        """Ordine dei bracci da provare. Con la mappa: prima quelli che arrivano sia alla presa sia alla
        destinazione, poi (solo se la destinazione non e' misurata) quelli che arrivano alla presa."""
        name = {"L": "left", "R": "right"}
        s, d = reach(book, src_y), reach(book, dst_y)
        both = ["LR"] if s is None else []
        cand = [a for a in "LR" if (s is None or a in s)]
        good = [a for a in cand if d is not None and a in d]
        unknown = [a for a in cand if d is None]
        out = [name[a] for a in good + unknown]
        if not out and s is None and d is None:
            first = "left" if dst_y > 0.10 else "right"
            out = [first, "right" if first == "left" else "left"]
        return out

    def do_move(self, k, n, m):
        bid, to_y = int(m["book"]), float(m["to_y"])
        dets = self._load_dets()
        det = next((d for d in dets if int(d["id"]) == bid), None)
        if det is None:
            self.status("ERRORE", mossa=k, libro=bid, motivo="libro non nelle rilevazioni")
            return False
        src_y = float(det["world_y"])
        self.status("MOSSA", n=f"{k}/{n}", libro=bid, da_y=round(src_y, 3), a_y=round(to_y, 3), tipo=m.get("kind"))
        arms = self._arms(bid, src_y, to_y)
        if not arms:
            self.status("ERRORE", libro=bid, motivo=f"nessun braccio raggiunge y={src_y:+.3f} o y={to_y:+.3f} (mappa)")
            return False
        self.get_logger().info(f"bracci da provare (mappa di raggiungibilita'): {arms}")
        for arm in arms:
            cmd = ["ros2", "run", "agibot_x2_pkg_py", "pick_test_book", "--ros-args",
                   "-p", f"target:={bid}", "-p", f"detections_file:={self.a.detections}",
                   "-p", "head_isbn:=false", "-p", f"dest_y:={to_y}", "-p", f"arm:={arm}",
                   "-p", f"grasp_depth:={self.depth_by_id.get(bid, self.a.grasp_depth)}"]
            if self.a.planner_id:
                cmd += ["-p", f"planner_id:={self.a.planner_id}"]
            self.get_logger().info("   $ " + " ".join(cmd))
            t0 = time.time()
            rc = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr).returncode
            self.status("PRESA-RISULTATO", libro=bid, braccio=arm, exit=rc, secondi=round(time.time() - t0))
            if rc != 0 and rc != 3 and self._placed_marker(bid, t0):
                # il libro E' nello slot di destinazione (detach fatto) ma la ritirata/casa non e' finita
                self.get_logger().warn(f"libro {bid}: posato, ma il rientro a casa e' finito in errore: aspetto che il braccio arrivi a casa")
                if self._wait_home(1800.0):
                    rc = 0
                else:
                    self.status("ERRORE", libro=bid, motivo="libro posato ma il braccio non e' a casa")
                    return False
            if rc == 0:
                break
            if rc == 3:
                continue                # destinazione fuori portata per questo braccio: provo l'altro
            return False
        else:
            self.status("ERRORE", libro=bid, motivo="destinazione fuori portata con entrambe le braccia")
            return False
        # posizione REALE dopo la posa: aggiorna rilevazioni e planning scene
        real = gz_book_y(GZ_NAMES.get(bid, "")) if GZ_NAMES.get(bid) else None
        new_y = real if real is not None and abs(real - to_y) < 0.03 else to_y
        if real is not None and abs(real - to_y) >= 0.03:
            self.get_logger().warn(f"libro {bid}: y reale {real:+.3f} lontana dalla destinazione {to_y:+.3f}: uso la prevista")
        det["world_y"] = round(new_y, 4)
        self._save_dets(dets)
        self.status("MOSSA-OK", libro=bid, y_reale=None if real is None else round(real, 3), y_salvata=round(new_y, 3))
        return True

    def _placed_marker(self, bid, t0):
        try:
            with open(f"/tmp/x2_reorder_done_obj{bid}.json") as f:
                return float(json.load(f).get("time", 0)) >= t0 - 1.0
        except Exception:
            return False

    def _wait_home(self, timeout_s):
        """Aspetta (spin) che braccia e vita siano a casa: tutti i giunti entro ~3 gradi da zero."""
        joints = [j for j in self._js if any(k in j for k in ("_shoulder_", "_elbow_", "_wrist_yaw", "waist_"))]
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.5)
            joints = [j for j in self._js if any(k in j for k in ("_shoulder_", "_elbow_", "_wrist_yaw", "waist_"))]
            if joints and all(abs(self._js[j]) < 0.06 for j in joints):
                return True
        return False

    # ─── oggetti: dal tavolo allo scaffale, a destra dell'ultimo libro ─────────────
    OBJ_GZ = {5: "gt_globe", 6: "gt_pen"}
    OBJ_REF = None      # forme in OBJ_SHAPES

    def run_objects(self, attempts=3):
        """Il portapenne (obj6) e il mappamondo (obj5), che stanno sul tavolo, vengono messi sullo scaffale a
        destra dell'ultimo libro (ordine irrilevante). Ogni oggetto: fino a `attempts` tentativi; il
        successo si verifica con la posizione reale in Gazebo. Video separato."""
        self.running = True
        video_on = False
        try:
            books = [d for d in self._load_dets() if d.get("is_book")]
            last = min(books, key=lambda d: float(d["world_y"]))          # il piu' a destra (y minima)
            last_edge = float(last["world_y"]) - float(last["thickness_m"]) / 2.0
            ref = {int(k): v for k, v in OBJ_SHAPES.items()}
            # pen all'estremo destro, mappamondo fra il libro e il portapenne (spazi ~2.5 cm)
            pen_w, gl_w = float(ref[6]["thickness_m"]), float(ref[5]["thickness_m"])
            y_pen = -0.378 + 0.02 + pen_w / 2.0
            y_globe = min(last_edge - 0.024 - gl_w / 2.0, y_pen + pen_w / 2.0 + 0.024 + gl_w / 2.0 + 0.002)
            plan = [(5, y_globe), (6, y_pen)]      # il mappamondo (piu' vicino al robot) prima: il portapenne e' dietro di lui
            self.status("OGGETTI-INIZIO", ultimo_libro_bordo_destro=round(last_edge, 3),
                        portapenne_y=round(y_pen, 3), mappamondo_y=round(y_globe, 3))
            self._video_start()
            video_on = True
            for oid, dest_y in plan:
                ok = False
                for attempt in range(1, attempts + 1):
                    if self._place_object(oid, dest_y, ref[oid], attempt):
                        ok = True
                        break
                    self.status("OGGETTO-RITENTO", oggetto=oid, tentativo=attempt)
                if not ok:
                    self.status("FERMATO", oggetto=oid, nota="non riuscito dopo i tentativi: controllare")
                    return False
            self.status("FINITO", esito="oggetti sullo scaffale")
            return True
        finally:
            if video_on:
                self._video_stop()
            self.running = False

    def _place_object(self, oid, dest_y, ref, attempt):
        gz = self.OBJ_GZ[oid]
        pose = gz_pose(gz)
        if pose is None:
            self.status("ERRORE", oggetto=oid, motivo="posizione Gazebo non leggibile")
            return False
        x, y, z = pose
        if z > 0.9 and x > 0.27 and abs(y - dest_y) < 0.04:
            self.status("OGGETTO-GIA-A-POSTO", oggetto=oid, posizione=[round(x, 3), round(y, 3), round(z, 3)])
            return True
        if z > 0.9:
            self.status("ERRORE", oggetto=oid, motivo=f"l'oggetto non e' sul tavolo (z={z:.2f}): niente da fare")
            return False
        dets = [d for d in self._load_dets() if int(d["id"]) not in (5, 6)]
        d = dict(ref)
        d["world_x"], d["world_y"] = 0.30, round(dest_y, 4)
        dets.append(d)
        path = "/tmp/x2_detections_obj.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(dets, f, ensure_ascii=False, indent=2)
        self.status("OGGETTO", oggetto=oid, gz=gz, da_tavolo=[round(x, 3), round(y, 3)], a_scaffale_y=round(dest_y, 3),
                    tentativo=attempt)
        for arm in ("right", "left"):
            cmd = ["ros2", "run", "agibot_x2_pkg_py", "pick_test_book", "--ros-args",
                   "-p", f"target:={oid}", "-p", f"detections_file:={path}", "-p", "head_isbn:=false",
                   "-p", f"from_table_x:={x}", "-p", f"from_table_y:={y}", "-p", f"arm:={arm}"]
            self.get_logger().info("   $ " + " ".join(cmd))
            t0 = time.time()
            rc = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr).returncode
            self.status("OGGETTO-RISULTATO", oggetto=oid, braccio=arm, exit=rc, secondi=round(time.time() - t0))
            if rc == 3:
                continue
            break
        after = gz_pose(gz)
        # dentro lo slot: sullo scaffale (z), al posto giusto (y) E alla profondita' giusta (x): con l'oggetto ancora
        # in mano fuori dallo scaffale (x ~ 0.19) z e y erano gia' "giusti" (falso positivo del 2026-09-20)
        good = (after is not None and after[2] > 0.9 and abs(after[1] - dest_y) < 0.04 and after[0] > 0.27)
        self.status("OGGETTO-VERIFICA", oggetto=oid, posizione=None if after is None else [round(v, 3) for v in after],
                    sullo_scaffale_al_posto_giusto=good)
        return good

    def _write_final(self, doc):
        try:
            dets = {int(d["id"]): d for d in self._load_dets()}
            books = doc.get("books") or []
            out = []
            for b in books:
                b = dict(b)
                if int(b["id"]) in dets:
                    b["y"] = dets[int(b["id"])]["world_y"]
                out.append(b)
            with open("/tmp/x2_library_riordinata.json", "w", encoding="utf-8") as f:
                json.dump({"books_riordinati": out, "ordine": (doc.get("plan") or {}).get("order")},
                          f, ensure_ascii=False, indent=1)
        except Exception as e:
            self.get_logger().warn(f"scrittura del risultato finale non riuscita: {e}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--plan", help="file del piano da eseguire subito (default: aspetta il topic)")
    ap.add_argument("--wait", action="store_true", help="aspetta il piano dall'app (default se manca --plan)")
    ap.add_argument("--detections", default="/tmp/x2_detections.json")
    ap.add_argument("--grasp-depth", dest="grasp_depth", type=float, default=0.018)
    ap.add_argument("--depth-by-id", dest="depth_by_id", default="", help='es. "3:0.012,2:0.015"')
    ap.add_argument("--planner-id", dest="planner_id", default="")
    ap.add_argument("--objects-only", dest="objects_only", action="store_true",
                    help="solo la fase oggetti: tavolo -> scaffale a destra dell'ultimo libro")
    ap.add_argument("--allow-demo", dest="allow_demo", action="store_true")
    ap.add_argument("--video-prefix", dest="video_prefix", default="riordino_v1",
                    help="prefisso dei video del riordino in videos/ ('' = nessun video)")
    a, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = ReorderExecutor(a)
    rc = 0
    try:
        if a.objects_only:
            rc = 0 if node.run_objects() else 1
        elif a.plan:
            with open(a.plan, encoding="utf-8") as f:
                rc = 0 if node.run_plan(json.load(f)) else 1
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
