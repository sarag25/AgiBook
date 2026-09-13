#!/usr/bin/env python3
"""
library_pipeline - orchestratore della pipeline automatica (2026-09-13):

  1. foto della libreria (shelf_camera rgbd) -> libri e oggetti con misure 3D
  2. gli OGGETTI (non libri) vengono presi e parcheggiati sul tavolo
     (uno slot ciascuno, object_slots_xy)
  3. seconda foto, ora solo libri: OCR dei dorsi + ricerca su Google Books
     per titolo (library_manager_node, title_lookup) -> metadati
  4. i libri SENZA metadati (niente ISBN) vengono portati uno alla volta a
     faccia in giu' sotto la table_camera (book_slots_xy), ri-fotografati,
     ISBN dal codice a barre -> metadati
  5. risultato in out_file (JSON: libri con metadati, oggetti parcheggiati,
     irrisolti) e riepilogo nel log

Ogni presa e' un processo `pick_test_book -p target:=<id> -p release_x/y`
(stessa cosa che si fa a mano), le foto passano da library_manager_node
(/library_manager/trigger 'shelf', /library_manager/rephotograph '<id>',
risultati su /library_manager/detections). Solo la mano DESTRA: la sinistra
non ha ne' DetachableJoint ne' catena IK (vedi PickAndPlace.md).

Prerequisiti: simulazione su, controller attivi, robot nella posa di
lavoro (walk_to_shelf fatto, oppure walk:=true qui), library_manager_node
acceso dalla radice del repo con detector:=depth (o sam3).

  ros2 run agibot_x2_pkg_py library_pipeline
  ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p dry_run:=true      # foto vere, prese solo pianificate
  ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p force_isbn:=true   # ignora i titoli: tutti i libri via ISBN
"""

import json
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LibraryPipeline(Node):

    def __init__(self):
        super().__init__("library_pipeline")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("walk", False)
        # Prima foto DA LONTANO (2026-09-13, prova dal vivo): nella posa di
        # lavoro la testa del robot copre il mappamondo (a y=+0.05, dietro la
        # testa vista dalla shelf_camera). Con photo_from_afar il robot va a
        # base_x = photo_x (0.6 -> 0.9 m dietro la posa di lavoro, fuori
        # dall'inquadratura), scatta, poi cammina alla posa di lavoro. La
        # seconda foto (solo libri, a destra della testa) va bene da vicino.
        self.declare_parameter("photo_from_afar", True)
        self.declare_parameter("photo_x", 0.6)
        self.declare_parameter("force_isbn", False)
        self.declare_parameter("skip_objects", False)
        self.declare_parameter("skip_books", False)
        # slot sul tavolo (x, y mondo del TCP al rilascio), verificati con l'IK
        # il 2026-09-13: fascia raggiungibile y -0.37..-0.48, x -0.45..+0.05
        self.declare_parameter("object_slots_xy", [0.03, -0.40, 0.03, -0.50])
        # sotto la table_camera (x -0.555..-0.025 con hfov 0.9): 2 libri affiancati
        self.declare_parameter("book_slots_xy", [-0.43, -0.37, -0.17, -0.37])
        self.declare_parameter("detections_file", "/tmp/x2_detections.json")
        self.declare_parameter("out_file", "/tmp/x2_library.json")
        self.declare_parameter("shelf_wait_s", 900.0)    # SAM3 su CPU: minuti
        self.declare_parameter("rephoto_wait_s", 180.0)
        g = lambda n: self.get_parameter(n).value
        self.dry_run = bool(g("dry_run"))
        self.force_isbn = bool(g("force_isbn"))
        self.skip_objects = bool(g("skip_objects"))
        self.skip_books = bool(g("skip_books"))
        self.walk = bool(g("walk"))
        self.photo_from_afar = bool(g("photo_from_afar"))
        self.photo_x = float(g("photo_x"))
        so, sb = list(g("object_slots_xy")), list(g("book_slots_xy"))
        self.object_slots = [(float(so[i]), float(so[i + 1])) for i in range(0, len(so) - 1, 2)]
        self.book_slots = [(float(sb[i]), float(sb[i + 1])) for i in range(0, len(sb) - 1, 2)]
        self.det_file = str(g("detections_file"))
        self.out_file = str(g("out_file"))
        self.shelf_wait = float(g("shelf_wait_s"))
        self.rephoto_wait = float(g("rephoto_wait_s"))

        self._det_seq = 0
        self._dets = None
        self._status = ""
        self.create_subscription(String, "/library_manager/detections", self._det_cb, LATCHED)
        self.create_subscription(String, "/library_manager/status", self._status_cb, LATCHED)
        self.pub_trigger = self.create_publisher(String, "/library_manager/trigger", 10)
        self.pub_rephoto = self.create_publisher(String, "/library_manager/rephotograph", 10)
        self.results = {"objects": [], "books": [], "unresolved": [], "log": []}

    def _det_cb(self, msg):
        try:
            self._dets = json.loads(msg.data)
        except Exception:
            self._dets = None
        self._det_seq += 1

    def _status_cb(self, msg):
        self._status = msg.data

    # ─── passi ────────────────────────────────────────────────────────────

    def _wait_seq(self, seq0, timeout, what):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._det_seq > seq0 and self._dets is not None:
                return True
            if self._status.startswith("error"):
                self.get_logger().error(f"{what}: library_manager in stato '{self._status}'")
                return False
        self.get_logger().error(f"{what}: nessuna risposta entro {timeout:.0f} s")
        return False

    def shoot_shelf(self, label):
        # spin per ricevere l'eventuale detections latched vecchia PRIMA di contare
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        seq0 = self._det_seq
        self.get_logger().info(f"{label}: scatto shelf_camera (trigger 'shelf')...")
        self.pub_trigger.publish(String(data="shelf"))
        if not self._wait_seq(seq0, self.shelf_wait, label):
            return None
        dets = [d for d in self._dets if d.get("thickness_m", 0) > 0]
        skipped = len(self._dets) - len(dets)
        books = [d for d in dets if d.get("is_book")]
        objs = [d for d in dets if not d.get("is_book")]
        self.get_logger().info(
            f"{label}: {len(books)} libri, {len(objs)} oggetti"
            + (f" ({skipped} senza misure 3D, ignorati)" if skipped else ""))
        for d in dets:
            self.get_logger().info(
                f"   obj {d['id']:>2} {'libro  ' if d.get('is_book') else 'oggetto'} "
                f"y={d['world_y']:+.3f} sp={d['thickness_m']*1000:3.0f}mm "
                f"titolo='{d.get('title','')}' isbn={d.get('isbn','') or '-'}")
        return dets

    def pick(self, det, slot, label):
        x, y = slot
        cmd = ["ros2", "run", "agibot_x2_pkg_py", "pick_test_book", "--ros-args",
               "-p", f"target:={det['id']}", "-p", f"release_x:={x}", "-p", f"release_y:={y}",
               "-p", f"detections_file:={self.det_file}"]
        if self.dry_run:
            cmd += ["-p", "dry_run:=true"]
        self.get_logger().info(f"{label}: obj {det['id']} -> tavolo ({x:+.2f}, {y:+.2f})"
                               + (" [dry_run]" if self.dry_run else ""))
        self.get_logger().info("   $ " + " ".join(cmd))
        t0 = time.time()
        rc = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr).returncode
        ok = rc == 0
        self.get_logger().info(f"{label}: {'OK' if ok else 'FALLITA (exit %d)' % rc} in {time.time()-t0:.0f} s")
        self.results["log"].append({"step": label, "id": det["id"], "slot": [x, y], "ok": ok})
        return ok

    def rephotograph(self, det, label):
        seq0 = self._det_seq
        self.get_logger().info(f"{label}: ri-foto dal tavolo per obj {det['id']} (ISBN)...")
        self.pub_rephoto.publish(String(data=str(det["id"])))
        if self.dry_run:
            # la foto viene fatta comunque (il libro pero' non c'e'): utile per vedere i log
            pass
        if not self._wait_seq(seq0, self.rephoto_wait, label):
            return None
        upd = next((d for d in self._dets if int(d["id"]) == int(det["id"])), None)
        if upd and upd.get("isbn"):
            self.get_logger().info(
                f"{label}: ISBN {upd['isbn']} -> '{upd.get('title')}' {upd.get('author')} {upd.get('year')}")
            return upd
        self.get_logger().warn(f"{label}: nessun ISBN letto per obj {det['id']}")
        return None

    # ─── sequenza ─────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info(
            "PIPELINE: foto -> oggetti sul tavolo -> foto libri -> titoli/Google Books -> "
            "ISBN dal tavolo per i libri non identificati"
            + (" [DRY RUN: prese solo pianificate]" if self.dry_run else ""))
        # 1. prima foto (da lontano: il robot fuori dall'inquadratura)
        if self.photo_from_afar:
            self.get_logger().info(f"1. mi metto a base_x={self.photo_x:.2f} per la foto (fuori dall'inquadratura)")
            if subprocess.run(["ros2", "run", "agibot_x2_pkg_py", "walk_to_shelf", "--ros-args",
                               "-p", f"distance:={self.photo_x}"]).returncode != 0:
                self.get_logger().error("walk_to_shelf (arretramento per la foto) fallito")
                return False
        dets = self.shoot_shelf("1. foto libreria")
        if dets is None:
            return False
        if self.photo_from_afar or self.walk:
            self.get_logger().info("1b. cammino fino alla posa di lavoro")
            if subprocess.run(["ros2", "run", "agibot_x2_pkg_py", "walk_to_shelf"]).returncode != 0:
                self.get_logger().error("walk_to_shelf fallito")
                return False
        objs = [d for d in dets if not d.get("is_book")]

        # 2. oggetti sul tavolo
        if self.skip_objects:
            self.get_logger().info("2. oggetti: saltato (skip_objects)")
        else:
            for i, o in enumerate(objs):
                if i >= len(self.object_slots):
                    self.get_logger().warn(f"2. slot per gli oggetti esauriti ({len(self.object_slots)}): "
                                           f"obj {o['id']} resta sullo scaffale")
                    self.results["unresolved"].append({"id": o["id"], "why": "nessuno slot oggetto libero"})
                    continue
                ok = self.pick(o, self.object_slots[i], f"2.{i+1} oggetto {o['id']}")
                self.results["objects"].append({"id": o["id"], "class": o.get("class"),
                                                "slot": list(self.object_slots[i]), "moved": ok})
                if not ok:
                    self.get_logger().error("presa fallita: mi fermo (l'oggetto potrebbe essere in mano)")
                    return False

        # 3. seconda foto: solo libri (OCR + Google Books dentro library_manager)
        if self.skip_books:
            self.get_logger().info("3-4. libri: saltato (skip_books)")
            self._write()
            return True
        dets2 = self.shoot_shelf("3. foto libri")
        if dets2 is None:
            return False
        books = [d for d in dets2 if d.get("is_book")]
        identified = [b for b in books if b.get("isbn") and not self.force_isbn]
        todo = [b for b in books if b not in identified]
        self.get_logger().info(
            f"3. identificati dal titolo: {[b['id'] for b in identified]}; da portare sul tavolo per "
            f"l'ISBN: {[b['id'] for b in todo]}" + (" (force_isbn)" if self.force_isbn else ""))
        for b in identified:
            self.results["books"].append({"id": b["id"], "title": b.get("title"), "author": b.get("author"),
                                          "year": b.get("year"), "isbn": b.get("isbn"), "how": "ocr+google_books",
                                          "where": "shelf"})

        # 4. libri senza metadati: uno alla volta sotto la camera
        for k, b in enumerate(todo):
            if k >= len(self.book_slots):
                self.get_logger().warn(f"4. slot sotto la camera esauriti ({len(self.book_slots)}): "
                                       f"obj {b['id']} resta sullo scaffale senza metadati")
                self.results["unresolved"].append({"id": b["id"], "why": "nessuno slot libro libero"})
                continue
            slot = self.book_slots[k]
            if not self.pick(b, slot, f"4.{k+1} libro {b['id']}"):
                self.get_logger().error("presa fallita: mi fermo")
                self._write()
                return False
            upd = None if self.dry_run else self.rephotograph(b, f"4.{k+1} libro {b['id']}")
            if upd:
                self.results["books"].append({"id": b["id"], "title": upd.get("title"), "author": upd.get("author"),
                                              "year": upd.get("year"), "isbn": upd.get("isbn"), "how": "isbn_table",
                                              "where": "table", "slot": list(slot)})
            else:
                self.results["unresolved"].append({"id": b["id"], "why": "ISBN non letto" if not self.dry_run else "dry_run",
                                                   "where": "table", "slot": list(slot)})
        self._write()
        return True

    def _write(self):
        with open(self.out_file, "w", encoding="utf-8") as f:
            json.dump(self.results, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f"RISULTATO -> {self.out_file}")
        for b in self.results["books"]:
            self.get_logger().info(f"   libro {b['id']:>2}: '{b['title']}' - {b['author']} ({b['year']}) ISBN {b['isbn']} [{b['how']}]")
        for o in self.results["objects"]:
            self.get_logger().info(f"   oggetto {o['id']:>2}: {'sul tavolo' if o['moved'] else 'NON spostato'} {o['slot']}")
        for u in self.results["unresolved"]:
            self.get_logger().warn(f"   irrisolto {u['id']}: {u['why']}")


def main(args=None):
    rclpy.init(args=args)
    node = LibraryPipeline()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
