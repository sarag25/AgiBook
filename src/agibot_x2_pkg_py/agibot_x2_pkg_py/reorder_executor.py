#!/usr/bin/env python3
"""
ROS 2 node that executes on the robot the reorder plan chosen in the app (latched on /library_manager/reorder_plan).

Each move runs `pick_test_book -p target:=<id> -p dest_y:=<to_y>`: the book stays in hand, moved sideways in front of the shelf;
arm order comes from reach_map, exit code 3 (out of reach, checked BEFORE moving) tries the other arm; stops at the first failure.
Status: /tmp/x2_reorder_status.json and /library_manager/reorder_status; result: /tmp/x2_library_riordinata.json.
`ros2 run agibot_x2_pkg_py reorder_executor --wait`   (waits for the plan from sort_ui)
`ros2 run agibot_x2_pkg_py reorder_executor --plan /tmp/x2_reorder_plan.json`
"""
import argparse
import json
import math
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
# fallback when Gazebo is not readable; normally names are discovered by discover_gz_names
GZ_NAMES_FALLBACK = {1: "gt_alba", 2: "gt_ballata", 3: "gt_it", 4: "gt_hunger"}
DEFAULT_DEPTH_BY_ID = {3: 0.012}      # IT (221 mm): at 18 mm the retreat has 27 mm lateral error
STATUS_FILE = "/tmp/x2_reorder_status.json"
# shapes of globe (obj5) and pen holder (obj6) measured by the depth detector
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
# reachability map picks the arm ORDER, avoiding ~5 min of IK on the wrong arm; unmeasured points are tried anyway
from agibot_x2_pkg_py.reach_map import arms as _reach_arms
PALM_HALF_WIDTH = 0.05      # m, half width of palm with fingers (hand URDF, checked in the planning scene)
PALM_WALL_MARGIN = 0.008    # m from the side wall, tuned (docs/COSTANTI.md)
from agibot_x2_pkg_py.reorder_planner import SHELF_HALF_INNER


def reach(book, y):
    """
    Arms ("L", "R", "LR", "") for (book, y) from the map, or None if not measured (1 cm tolerance)
    """
    return _reach_arms(book, y)


def gz_pose(model):
    """
    World (x, y, z) of the Gazebo model, or None
    """
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


def gz_upright(model):
    """
    Cosine of the tilt of the model vertical axis (1 = upright, 0 = lying), or None
    A lying pen holder has almost the same z as an upright one: only the quaternion shows it
    """
    try:
        txt = subprocess.run(["gz", "topic", "-e", "-t", "/world/bookshelf_world/pose/info", "-n", "1"],
                             capture_output=True, text=True, timeout=60).stdout
        i = txt.find(f'name: "{model}"')
        m = re.search(r"orientation\s*\{([^}]*)\}", txt[i:i + 900]) if i >= 0 else None
        if not m:
            return None
        q = {k: float(x) for k, x in re.findall(r"(\w):\s*([-+0-9.e]+)", m.group(1))}
        qx, qy = q.get("x", 0.0), q.get("y", 0.0)
        return 1.0 - 2.0 * (qx * qx + qy * qy)
    except Exception:
        return None


def gz_model_poses():
    """
    {model name: (x, y, z)} of all gt_* models in Gazebo (single read), or {}
    """
    try:
        txt = subprocess.run(["gz", "topic", "-e", "-t", "/world/bookshelf_world/pose/info", "-n", "1"],
                             capture_output=True, text=True, timeout=60).stdout
    except Exception:
        return {}
    out = {}
    for m in re.finditer(r'name:\s*"(gt_[A-Za-z0-9_]+)"[^{}]*?position\s*\{([^}]*)\}', txt, re.S):
        v = {k: float(x) for k, x in re.findall(r"(\w):\s*([-+0-9.e]+)", m.group(2))}
        out[m.group(1)] = (v.get("x", 0.0), v.get("y", 0.0), v.get("z", 0.0))
    return out


def discover_gz_names(book_dets, max_dist=0.06):
    """
    Match each detected book to the nearest Gazebo gt_* model (by y and x): {id: name}
    Decorations are excluded; returns {} if Gazebo does not answer (fallback is used)
    """
    poses = gz_model_poses()
    try:
        from agibot_x2_pkg.book_placer import test_entities
        deco = {n for n, _k, kind, _x, _y in test_entities("grasp_test") if kind != "book"}
    except Exception:
        deco = {"gt_globe", "gt_pen"}
    cand = {n: p for n, p in poses.items() if n not in deco}
    out, used = {}, set()
    pairs = sorted(((abs(float(d["world_y"]) - p[1]) + 0.5 * abs(float(d.get("world_x", p[0])) - p[0]), int(d["id"]), n)
                    for d in book_dets for n, p in cand.items()), key=lambda t: t[0])
    for dist, i, n in pairs:
        if i in out or n in used or dist > max_dist:
            continue
        out[i] = n
        used.add(n)
    return out


def gz_book_y(model):
    """
    Real world y of the Gazebo model, or None
    """
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
    """
    Runs reorder plans and the objects phase through pick_test_book subprocesses
    """

    def __init__(self, a):
        """
        Create publishers/subscriptions; without --plan wait for the plan on PLAN_TOPIC
        """
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
        self.gz_names = {}
        for kv in [x for x in a.depth_by_id.replace(" ", "").split(",") if ":" in x]:
            k, v = kv.split(":")
            self.depth_by_id[int(k)] = float(v)
        if not a.plan:
            self.create_subscription(String, PLAN_TOPIC, self._plan_cb, LATCHED)
            self.get_logger().info(f"reorder_executor: waiting for the plan on {PLAN_TOPIC} (app: 'Invia il piano')")

    # ─── status ────────────────────────────────────────────────────────────
    def status(self, state, **kw):
        """
        Write the status file, publish it on STATUS_TOPIC and log it
        """
        doc = {"state": state, "time": time.strftime("%H:%M:%S"), **kw}
        self.log_lines.append(doc)
        with open(STATUS_FILE, "w", encoding="utf-8") as f:
            json.dump({"last": doc, "history": self.log_lines}, f, ensure_ascii=False, indent=1)
        self.pub_status.publish(String(data=json.dumps(doc, ensure_ascii=False)))
        self.get_logger().info(f"[{state}] " + ", ".join(f"{k}={v}" for k, v in kw.items()))

    def _plan_cb(self, msg):
        """
        Run a plan received on the topic, unless one is already running
        """
        if self.running:
            self.get_logger().warn("a reorder is already running: ignoring the new plan")
            return
        try:
            doc = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"invalid plan: {e}")
            return
        self.run_plan(doc)

    # ─── detections ────────────────────────────────────────────────────────
    def _load_dets(self):
        """
        Load the detections file
        """
        with open(self.a.detections, encoding="utf-8") as f:
            return json.load(f)

    def _save_dets(self, dets):
        """
        Save the detections and publish them for the planning scene builder
        """
        with open(self.a.detections, "w", encoding="utf-8") as f:
            json.dump(dets, f, ensure_ascii=False, indent=2)
        self.pub_det.publish(String(data=json.dumps(dets, ensure_ascii=False)))   # planning scene builder

    # ─── video ─────────────────────────────────────────────────────────────
    def _video_start(self):
        """
        Record the reorder (main + table camera) from plan start to end
        """
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
        self.get_logger().info(f"video: recording {self.a.video_prefix}* in /home/robot/SmartRobotics/videos/")

    def _video_stop(self):
        """
        Stop the recorders with SIGINT to the whole process group
        (`ros2 run` does not forward it, leaving unreadable mp4 without "moov atom")
        """
        import os
        import signal
        time.sleep(15.0)              # last seconds of scene after the final move
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

    # ─── execution ─────────────────────────────────────────────────────────
    def run_plan(self, doc):
        """
        Execute all moves of the plan, stopping at the first failure
        """
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
            self.gz_names = discover_gz_names([d for d in self._load_dets() if d.get("is_book")]) or dict(GZ_NAMES_FALLBACK)
            self.status("INIZIO", mosse=len(moves), modelli_gazebo=self.gz_names)
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
        """
        Order of arms to try: first those reaching both grasp and destination,
        then (only if the destination is not measured) those reaching the grasp
        """
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
        """
        Execute one move with the candidate arms, then save the real book pose
        """
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
        self.get_logger().info(f"arms to try (reachability map): {arms}")
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
                # book is placed (detach done) but the retreat/home did not finish
                self.get_logger().warn(f"book {bid}: placed, but the return home failed: waiting for the arm to get home")
                if self._wait_home(1800.0):
                    rc = 0
                else:
                    self.status("ERRORE", libro=bid, motivo="libro posato ma il braccio non e' a casa")
                    return False
            if rc == 0:
                break
            if rc == 3:
                continue                # destination out of reach for this arm: try the other
            return False
        else:
            self.status("ERRORE", libro=bid, motivo="destinazione fuori portata con entrambe le braccia")
            return False
        # REAL pose after placing: update detections and planning scene
        real = gz_book_y(self.gz_names.get(bid, "")) if self.gz_names.get(bid) else None
        new_y = real if real is not None and abs(real - to_y) < 0.03 else to_y
        if real is not None and abs(real - to_y) >= 0.03:
            self.get_logger().warn(f"book {bid}: real y {real:+.3f} far from destination {to_y:+.3f}: using the expected one")
        det["world_y"] = round(new_y, 4)
        self._save_dets(dets)
        # expected center x = book front + half depth; >3 cm deeper means the release pushed it (y order still right)
        extra = {}
        gzp = gz_pose(self.gz_names[bid]) if self.gz_names.get(bid) else None
        if gzp is not None:
            expected_x = float(det.get("world_x", 0.27)) + float(det.get("length_m") or 0.16) / 2.0
            push = gzp[0] - expected_x
            extra = dict(x_reale=round(gzp[0], 3), spinto_indietro_cm=round(push * 100.0, 1))
            if push > 0.03:
                self.get_logger().warn(f"book {bid}: {push * 100:.0f} cm deeper than expected after release (y correct)")
        self.status("MOSSA-OK", libro=bid, y_reale=None if real is None else round(real, 3), y_salvata=round(new_y, 3), **extra)
        return True

    def _placed_marker(self, bid, t0):
        """
        True if pick_test_book wrote the "placed" marker for this book after t0
        """
        try:
            with open(f"/tmp/x2_reorder_done_obj{bid}.json") as f:
                return float(json.load(f).get("time", 0)) >= t0 - 1.0
        except Exception:
            return False

    def _wait_home(self, timeout_s):
        """
        Spin until arms and waist are home: all joints within ~3 degrees of zero
        """
        joints = [j for j in self._js if any(k in j for k in ("_shoulder_", "_elbow_", "_wrist_yaw", "waist_"))]
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.5)
            joints = [j for j in self._js if any(k in j for k in ("_shoulder_", "_elbow_", "_wrist_yaw", "waist_"))]
            if joints and all(abs(self._js[j]) < 0.06 for j in joints):
                return True
        return False

    # ─── objects: from the table to the shelf, right of the last book ─────────────
    OBJ_GZ = {5: "gt_globe", 6: "gt_pen"}
    OBJ_REF = None      # shapes are in OBJ_SHAPES

    def run_objects(self, attempts=3):
        """
        Move pen holder (obj6) and globe (obj5) from the table to the shelf, right of the last book
        Up to `attempts` tries per object, success checked on the real Gazebo pose
        """
        self.running = True
        video_on = False
        try:
            dets_all = self._load_dets()
            books = [d for d in dets_all if d.get("is_book")]
            last = min(books, key=lambda d: float(d["world_y"]))          # rightmost (minimum y)
            last_edge = float(last["world_y"]) - float(last["thickness_m"]) / 2.0
            # shapes: measured detections first, built-in shapes as fallback
            ref = {int(k): v for k, v in OBJ_SHAPES.items()}
            for d in dets_all:
                if not d.get("is_book") and int(d["id"]) in (5, 6) and d.get("thickness_m") and d.get("length_m"):
                    ref[int(d["id"])] = d
            # pen at the far right, globe between the last book and the pen (~2.5 cm gaps)
            pen_w, gl_w = float(ref[6]["thickness_m"]), float(ref[5]["thickness_m"])
            # pen at palm half width + 8 mm from the wall, so the palm does not touch the wall
            y_pen = -SHELF_HALF_INNER + max(0.02 + pen_w / 2.0, PALM_HALF_WIDTH + PALM_WALL_MARGIN)
            # 36 mm from the book (outer finger sticks out 27 mm past the object); 24 mm to the pen is enough (different depth)
            y_globe = min(last_edge - 0.036 - gl_w / 2.0, y_pen + pen_w / 2.0 + 0.024 + gl_w / 2.0 + 0.002)
            # globe first (deeper), then pen: the 10 cm palm placing the second does not reach the first
            plan = [(5, y_globe, float(self.a.globe_front_x)), (6, y_pen, float(self.a.pen_front_x))]
            self.status("OGGETTI-INIZIO", ultimo_libro_bordo_destro=round(last_edge, 3),
                        portapenne_y=round(y_pen, 3), mappamondo_y=round(y_globe, 3))
            self._video_start()
            video_on = True
            for oid, dest_y, dest_x in plan:
                ok = False
                for attempt in range(1, attempts + 1):
                    if self._place_object(oid, dest_y, dest_x, ref[oid], attempt):
                        ok = True
                        break
                    if getattr(self, "_abort_object", False):
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

    def _place_object(self, oid, dest_y, dest_x, ref, attempt):
        """
        One attempt to move an object from the table to (dest_x, dest_y); a lying object aborts the retries
        """
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
        up = gz_upright(gz)
        if up is not None and abs(up) < 0.8:
            self.status("OGGETTO-CORICATO", oggetto=oid, inclinazione_gradi=round(math.degrees(math.acos(max(-1.0, min(1.0, abs(up))))), 1),
                        nota="l'oggetto sul tavolo e' coricato: la presa dall'alto non lo afferra, inutile riprovare")
            self._abort_object = True
            return False
        dets = [d for d in self._load_dets() if int(d["id"]) not in (5, 6)]
        d = dict(ref)
        d["world_x"], d["world_y"] = round(dest_x, 4), round(dest_y, 4)
        dets.append(d)
        path = "/tmp/x2_detections_obj.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(dets, f, ensure_ascii=False, indent=2)
        self.status("OGGETTO", oggetto=oid, gz=gz, da_tavolo=[round(x, 3), round(y, 3)], a_scaffale_y=round(dest_y, 3),
                    a_scaffale_x=round(dest_x, 3), tentativo=attempt)
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
        # check z, y AND x: an object still in hand outside the shelf (x ~ 0.19) already has the right z and y
        good = (after is not None and after[2] > 0.9 and abs(after[1] - dest_y) < 0.04 and after[0] > 0.27)
        self.status("OGGETTO-VERIFICA", oggetto=oid, posizione=None if after is None else [round(v, 3) for v in after],
                    in_piedi=gz_upright(gz), sullo_scaffale_al_posto_giusto=good)
        return good

    def _write_final(self, doc):
        """
        Write the reordered library with the new y positions
        """
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
            self.get_logger().warn(f"writing the final result failed: {e}")


def main(argv=None):
    """
    CLI: run the objects phase, a plan file, or wait for plans on the topic
    """
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--plan", help="plan file to run immediately (default: wait for the topic)")
    ap.add_argument("--wait", action="store_true", help="wait for the plan from the app (default without --plan)")
    ap.add_argument("--detections", default="/tmp/x2_detections.json")
    ap.add_argument("--grasp-depth", dest="grasp_depth", type=float, default=0.018)
    ap.add_argument("--depth-by-id", dest="depth_by_id", default="", help='e.g. "3:0.012,2:0.015"')
    ap.add_argument("--planner-id", dest="planner_id", default="")
    ap.add_argument("--objects-only", dest="objects_only", action="store_true",
                    help="objects phase only: table -> shelf right of the last book")
    ap.add_argument("--globe-front-x", dest="globe_front_x", type=float, default=0.38,
                    help="globe front x on the shelf (deeper than the pen holder)")
    ap.add_argument("--pen-front-x", dest="pen_front_x", type=float, default=0.30,
                    help="pen holder front x on the shelf")
    ap.add_argument("--allow-demo", dest="allow_demo", action="store_true")
    ap.add_argument("--video-prefix", dest="video_prefix", default="riordino_v1",
                    help="prefix of the reorder videos in videos/ ('' = no video)")
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
