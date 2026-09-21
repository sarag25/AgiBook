#!/usr/bin/env python3
"""
ROS 2 node that orchestrates the automatic library pipeline: shelf photo -> objects parked on the table
-> books-only photo with spine OCR + Google Books -> books without metadata held in front of the head for
the ISBN and put back in the same shelf slot -> results in out_file. Each grasp runs `pick_test_book`,
photos go through library_manager_node. Needs simulation, controllers and library_manager_node running.
  ros2 run agibot_x2_pkg_py library_pipeline
  ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p dry_run:=true      # real photos, grasps only planned
  ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p force_isbn:=true   # ignore titles: all books via ISBN
"""

import json
import math
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
    """
    Sequential pipeline driving library_manager_node, walk_to_shelf and pick_test_book
    """

    def __init__(self):
        """
        Declare and read parameters, create library_manager topics
        """
        super().__init__("library_pipeline")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("walk", False)
        # first photo from afar: at the work pose the robot's head hides the globe from shelf_camera
        self.declare_parameter("photo_from_afar", True)
        self.declare_parameter("photo_x", 0.6)
        # shelf distance measured by head_camera depth; 0.666 m = reading at the work pose (base_x 1.5),
        # the photo from afar is photo_gap m further; the last leg is re-measured (~5 cm error from 2 m)
        self.declare_parameter("use_depth", True)
        self.declare_parameter("shelf_distance_work", 0.666)
        self.declare_parameter("photo_gap", 0.9)
        # 'ocr' = up to the JSON with zoom OCR titles; 'full' = also in-hand ISBN for unidentified books
        self.declare_parameter("phase", "ocr")
        self.declare_parameter("zoom", True)
        self.declare_parameter("zoom_wait_s", 2400.0)
        self.declare_parameter("force_isbn", False)
        # books without metadata are shown to the head camera for the barcode (pick_test_book head_isbn)
        self.declare_parameter("head_isbn", True)     # table_camera no longer exists
        self.declare_parameter("skip_objects", False)
        self.declare_parameter("skip_books", False)
        # reuse latched detections: the shelf is only found from afar (0 books from the work pose)
        self.declare_parameter("use_last_detections", False)
        # m past the spine: palm is ~22 mm behind the fingertips, 18 mm keeps it ~4 mm off (25 mm hits)
        self.declare_parameter("book_grasp_depth", 0.018)
        # only_ids "2,3,4": in-hand ISBN only for these books (resume after an error)
        # arm_by_id "2:left,3:right": arm per book (auto may pick an arm that cannot reach the approach)
        self.declare_parameter("only_ids", "")
        self.declare_parameter("arm_by_id", "")
        # grasp_from_top_by_id "3:0.09": grasp height per book (m from the top), when a mid-height grasp
        # exceeds the 20 mm lateral error limit on the retreat
        self.declare_parameter("grasp_from_top_by_id", "")
        # grasp_depth_by_id "3:0.012": per-book book_grasp_depth; with a rotated gripper 18 mm puts the palm inside
        self.declare_parameter("grasp_depth_by_id", "")
        # forwarded to pick_test_book: "" = RRTConnect, "RRTstarkConfigDefault" = experimental
        self.declare_parameter("planner_id", "")
        # TCP release (x, y) per object; table edge at x=+0.05, right-arm IK reach ends near y=-0.51,
        # the two objects stay 14 cm apart along the finger axis (x) so the 10 cm palm does not hit the first
        self.declare_parameter("object_slots_xy", [-0.15, -0.38, -0.29, -0.37])
        # 2 books side by side under the table_camera (x -0.555..-0.025, hfov 0.9)
        self.declare_parameter("book_slots_xy", [-0.43, -0.37, -0.17, -0.37])
        self.declare_parameter("detections_file", "/tmp/x2_detections.json")
        self.declare_parameter("out_file", "/tmp/x2_library.json")
        self.declare_parameter("shelf_wait_s", 900.0)    # SAM3 on CPU takes minutes
        self.declare_parameter("rephoto_wait_s", 180.0)
        g = lambda n: self.get_parameter(n).value
        self.dry_run = bool(g("dry_run"))
        self.force_isbn = bool(g("force_isbn"))
        self.head_isbn = bool(g("head_isbn"))
        self.skip_objects = bool(g("skip_objects"))
        self.skip_books = bool(g("skip_books"))
        self.use_last = bool(g("use_last_detections"))
        self.book_grasp_depth = float(g("book_grasp_depth"))
        self.gft_by_id = {int(k): float(v) for k, v in (kv.split(":") for kv in str(g("grasp_from_top_by_id")).replace(" ", "").split(",") if ":" in kv)}
        self.gd_by_id = {int(k): float(v) for k, v in (kv.split(":") for kv in str(g("grasp_depth_by_id")).replace(" ", "").split(",") if ":" in kv)}
        self.only_ids = [int(x) for x in str(g("only_ids")).replace(" ", "").split(",") if x]
        self.arm_by_id = {int(k): v for k, v in (kv.split(":") for kv in str(g("arm_by_id")).replace(" ", "").split(",") if ":" in kv)}
        self.planner_id = str(g("planner_id"))
        self.walk = bool(g("walk"))
        self.photo_from_afar = bool(g("photo_from_afar"))
        self.photo_x = float(g("photo_x"))
        self.use_depth = bool(g("use_depth"))
        self.d_work = float(g("shelf_distance_work"))
        self.photo_gap = float(g("photo_gap"))
        self.phase = str(g("phase")).strip().lower()
        self.zoom = bool(g("zoom"))
        self.zoom_wait = float(g("zoom_wait_s"))
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
        self.placed = []          # real (x, y) of objects already on the table

    def _det_cb(self, msg):
        """
        Store the latest detections and bump the sequence counter
        """
        try:
            self._dets = json.loads(msg.data)
        except Exception:
            self._dets = None
        self._det_seq += 1

    def _status_cb(self, msg):
        """
        Store the library_manager status
        """
        self._status = msg.data

    def _wait_seq(self, seq0, timeout, what):
        """
        Wait for new detections after sequence seq0; False on timeout or library_manager error
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._det_seq > seq0 and self._dets is not None:
                return True
            if self._status.startswith("error"):
                self.get_logger().error(f"{what}: library_manager in state '{self._status}'")
                return False
        self.get_logger().error(f"{what}: no response within {timeout:.0f} s")
        return False

    def shoot_shelf(self, label):
        """
        Take a 'shelf' photo (or reuse latched detections), return detections with 3D measures
        """
        # receive an old latched detection before counting
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.use_last:
            t0 = time.monotonic()           # latched message arrives after DDS discovery: up to 20 s
            while not self._dets and time.monotonic() - t0 < 20.0:
                rclpy.spin_once(self, timeout_sec=0.2)
            if not self._dets:
                self.get_logger().error(f"{label}: use_last_detections but library_manager has no detections "
                                        "(take the photo from afar first: skip_books:=true)")
                return None
            self.get_logger().info(f"{label}: using existing detections (use_last_detections)")
        else:
            seq0 = self._det_seq
            self.get_logger().info(f"{label}: shooting shelf_camera (trigger 'shelf')...")
            self.pub_trigger.publish(String(data="shelf"))
            if not self._wait_seq(seq0, self.shelf_wait, label):
                return None
        dets = [d for d in self._dets if d.get("thickness_m", 0) > 0]
        skipped = len(self._dets) - len(dets)
        books = [d for d in dets if d.get("is_book")]
        objs = [d for d in dets if not d.get("is_book")]
        self.get_logger().info(
            f"{label}: {len(books)} books, {len(objs)} objects"
            + (f" ({skipped} without 3D measures, ignored)" if skipped else ""))
        for d in dets:
            self.get_logger().info(
                f"   obj {d['id']:>2} {'libro  ' if d.get('is_book') else 'oggetto'} "
                f"y={d['world_y']:+.3f} sp={d['thickness_m']*1000:3.0f}mm "
                f"title='{d.get('title','')}' isbn={d.get('isbn','') or '-'}")
        return dets

    def walk_robot(self, label, shelf_distance=None, base_x=None):
        """
        Run walk_to_shelf to a depth-measured shelf distance or to a given base_x
        """
        cmd = ["ros2", "run", "agibot_x2_pkg_py", "walk_to_shelf", "--ros-args"]
        if shelf_distance is not None:
            cmd += ["-p", f"shelf_distance:={shelf_distance:.3f}"]
            self.get_logger().info(f"{label}: walking to {shelf_distance:.3f} m from the bookshelf (depth)")
        elif base_x is not None:
            cmd += ["-p", f"distance:={base_x:.3f}"]
            self.get_logger().info(f"{label}: walking to base_x={base_x:.2f}")
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            self.get_logger().error(f"{label}: walk_to_shelf failed (exit {rc})")
        return rc == 0

    def zoom_books(self, label):
        """
        library_manager 'zoom' trigger: head on each spine, close-up OCR, title lookup; returns updated detections
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        seq0 = self._det_seq
        self.get_logger().info(f"{label}: zoomed spine photos + OCR (trigger 'zoom')...")
        self.pub_trigger.publish(String(data="zoom"))
        if not self._wait_seq(seq0, self.zoom_wait, label):
            return None
        books = [d for d in self._dets if d.get("is_book")]
        for d in books:
            self.get_logger().info(
                f"   obj {d['id']:>2} book y={d['world_y']:+.3f} title='{d.get('title','')}' "
                f"author='{d.get('author','')}' isbn={d.get('isbn','') or '-'}")
        return self._dets

    def pick(self, det, slot, label, head_isbn=False):
        """
        Run pick_test_book to move `det` to table slot (x, y), return True on success
        """
        x, y = slot
        cmd = ["ros2", "run", "agibot_x2_pkg_py", "pick_test_book", "--ros-args",
               "-p", f"target:={det['id']}", "-p", f"release_x:={x}", "-p", f"release_y:={y}",
               "-p", f"detections_file:={self.det_file}"]
        if self.dry_run:
            cmd += ["-p", "dry_run:=true"]
        if head_isbn:
            cmd += ["-p", "head_isbn:=true"]
        if self.planner_id:
            cmd += ["-p", f"planner_id:={self.planner_id}"]
        self.get_logger().info(f"{label}: obj {det['id']} -> table ({x:+.2f}, {y:+.2f})"
                               + (" [dry_run]" if self.dry_run else ""))
        self.get_logger().info("   $ " + " ".join(cmd))
        t0 = time.time()
        rc = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr).returncode
        ok = rc == 0
        self.get_logger().info(f"{label}: {'OK' if ok else 'FALLITA (exit %d)' % rc} in {time.time()-t0:.0f} s")
        self.results["log"].append({"step": label, "id": det["id"], "slot": [x, y], "ok": ok})
        return ok

    def pick_and_return(self, det, label):
        """
        Hold a book without metadata in front of the head for the ISBN, then put it back in the same slot
        Never goes to the table (pick_test_book put_back:=true reuses the original grasp pose); the ISBN
        result is read from /tmp/x2_head_isbn_obj<id>.json regardless of the exit code.
        """
        cmd = ["ros2", "run", "agibot_x2_pkg_py", "pick_test_book", "--ros-args",
               "-p", f"target:={det['id']}", "-p", f"detections_file:={self.det_file}",
               "-p", "head_isbn:=true", "-p", "put_back:=true", "-p", f"grasp_depth:={self.gd_by_id.get(det['id'], self.book_grasp_depth)}"]
        if self.dry_run:
            cmd += ["-p", "dry_run:=true"]
        if self.planner_id:
            cmd += ["-p", f"planner_id:={self.planner_id}"]
        if det["id"] in self.gft_by_id:
            cmd += ["-p", f"grasp_from_top:={self.gft_by_id[det['id']]}"]
        if det["id"] in self.arm_by_id:
            cmd += ["-p", f"arm:={self.arm_by_id[det['id']]}"]
        self.get_logger().info(f"{label}: obj {det['id']} -> in hand for the ISBN, then back on the shelf"
                               + (" [dry_run]" if self.dry_run else ""))
        self.get_logger().info("   $ " + " ".join(cmd))
        t0 = time.time()
        rc = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr).returncode
        ok = rc == 0
        self.get_logger().info(f"{label}: {'OK' if ok else 'FALLITA (exit %d)' % rc} in {time.time()-t0:.0f} s")
        self.results["log"].append({"step": label, "id": det["id"], "ok": ok})
        return ok

    def rephotograph(self, det, label):
        """
        Ask library_manager to re-photograph `det` for the ISBN, return the updated detection or None
        """
        seq0 = self._det_seq
        self.get_logger().info(f"{label}: re-photo from the table for obj {det['id']} (ISBN)...")
        self.pub_rephoto.publish(String(data=str(det["id"])))
        if self.dry_run:
            # the photo is taken anyway (without the book): useful for the logs
            pass
        if not self._wait_seq(seq0, self.rephoto_wait, label):
            return None
        upd = next((d for d in self._dets if int(d["id"]) == int(det["id"])), None)
        if upd and upd.get("isbn"):
            self.get_logger().info(
                f"{label}: ISBN {upd['isbn']} -> '{upd.get('title')}' {upd.get('author')} {upd.get('year')}")
            return upd
        self.get_logger().warn(f"{label}: no ISBN read for obj {det['id']}")
        return None

    def run(self):
        """
        Run the whole pipeline, return True on success
        """
        self.get_logger().info(
            "PIPELINE: photo -> objects on the table -> books photo -> titles/Google Books -> "
            "unidentified books held (never on the table) in front of the head for the ISBN, then put "
            "back in the same shelf slot"
            + (" [DRY RUN: grasps only planned]" if self.dry_run else ""))
        # 0-1. bookshelf distance (depth) and first photo from afar
        if self.photo_from_afar:
            if self.use_depth:
                ok = self.walk_robot("1. posa per la foto", shelf_distance=self.d_work + self.photo_gap)
            else:
                ok = self.walk_robot("1. posa per la foto", base_x=self.photo_x)
            if not ok:
                return False
        dets = self.shoot_shelf("1. foto libreria")
        if dets is None:
            return False
        if self.photo_from_afar or self.walk:
            if self.use_depth:
                # ~1 cm error from ~1.6 m: a second close pass puts the pelvis within +-1 mm
                ok = (self.walk_robot("1b. posa di lavoro", shelf_distance=self.d_work)
                      and self.walk_robot("1c. correzione fine", shelf_distance=self.d_work))
            else:
                self.get_logger().info("1b. walking to the work pose")
                ok = subprocess.run(["ros2", "run", "agibot_x2_pkg_py", "walk_to_shelf"]).returncode == 0
            if not ok:
                return False
        objs = [d for d in dets if not d.get("is_book")]

        # 2. objects to the table
        if self.skip_objects:
            self.get_logger().info("2. objects: skipped (skip_objects)")
        else:
            for i, o in enumerate(objs):
                if i >= len(self.object_slots):
                    self.get_logger().warn(f"2. no object slots left ({len(self.object_slots)}): "
                                           f"obj {o['id']} stays on the shelf")
                    self.results["unresolved"].append({"id": o["id"], "why": "nessuno slot oggetto libero"})
                    continue
                slot = self._slot_for(i)
                ok = self.pick(o, slot, f"2.{i+1} oggetto {o['id']}")
                self._read_placed(o["id"])
                self.results["objects"].append({"id": o["id"], "class": o.get("class"),
                                                "slot": list(slot), "moved": ok})
                if not ok:
                    self.get_logger().error("grasp failed: stopping (the object may be in hand)")
                    return False

        # 3. second photo: books only (OCR + Google Books inside library_manager)
        if self.skip_books:
            self.get_logger().info("3-4. books: skipped (skip_books)")
            self._write()
            return True
        dets2 = self.shoot_shelf("3. foto libri")
        if dets2 is None:
            return False
        if self.zoom:
            dets3 = self.zoom_books("3b. zoom sui dorsi")
            if dets3 is not None:
                dets2 = dets3
        books = [d for d in dets2 if d.get("is_book")]
        self._books_final = books        # for the JSON "library" section (_write)
        identified = [b for b in books if b.get("isbn") and not self.force_isbn]
        todo = [b for b in books if b not in identified and (not self.only_ids or b["id"] in self.only_ids)]
        self.get_logger().info(
            f"3. identified by title: {[b['id'] for b in identified]}; to handle for "
            f"the ISBN: {[b['id'] for b in todo]}" + (" (force_isbn)" if self.force_isbn else ""))
        for b in identified:
            self.results["books"].append({"id": b["id"], "title": b.get("title"), "author": b.get("author"),
                                          "year": b.get("year"), "isbn": b.get("isbn"), "how": "ocr+google_books",
                                          "where": "shelf"})
        if self.phase != "full":
            for b in todo:
                self.results["unresolved"].append({"id": b["id"], "why": "titolo non identificato dall'OCR (fase ocr)",
                                                   "ocr_title": b.get("title", ""), "where": "shelf"})
            self._write()
            self.get_logger().info("Phase 'ocr' done (phase:=full to continue with the in-hand ISBN)")
            return True

        # 4. books without metadata: one at a time in hand for the ISBN, then back to the same slot
        for k, b in enumerate(todo):
            if not self.pick_and_return(b, f"4.{k+1} libro {b['id']}"):
                self.get_logger().error("grasp failed: stopping")
                self._write()
                return False
            upd = None
            if not self.dry_run:
                hp = f"/tmp/x2_head_isbn_obj{b['id']}.json"
                try:
                    with open(hp) as f:
                        h = json.load(f)
                except Exception as e:
                    h = None
                    self.get_logger().warn(f"4.{k+1} book {b['id']}: head result not found ({e})")
                if h and h.get("isbn"):
                    upd = h
                    self.get_logger().info(f"4.{k+1} book {b['id']}: ISBN from the head {h['isbn']} -> "
                                           f"'{h.get('title')}' {h.get('author')} {h.get('year')}")
                else:
                    self.get_logger().info(f"4.{k+1} book {b['id']}: no ISBN from the head - stays on the shelf without metadata")
            if upd:
                self.results["books"].append({"id": b["id"], "title": upd.get("title"), "author": upd.get("author"),
                                              "year": upd.get("year"), "isbn": upd.get("isbn"), "how": "isbn_head",
                                              "where": "shelf"})
            else:
                self.results["unresolved"].append({"id": b["id"], "why": "ISBN non letto" if not self.dry_run else "dry_run",
                                                   "where": "shelf"})
        self._write()
        return True

    # right-arm fallback slots, IK verified offline (< 4 mm error after the step back)
    ALT_SLOTS = [(-0.29, -0.37), (-0.29, -0.40), (-0.22, -0.38), (-0.15, -0.45), (-0.15, -0.40)]   # no x>-0.15: table edge is at +0.05
    MIN_CLEAR = 0.13      # m between object centers: radii 3.3 + 2.5 cm + half palm (5 cm) + 2 cm

    def _read_placed(self, obj_id):
        """
        Read where the object really ended up (written by pick_test_book)
        """
        try:
            with open(f"/tmp/x2_placed_obj{obj_id}.json", encoding="utf-8") as f:
                d = json.load(f)
            self.placed.append((float(d["x"]), float(d["y"])))
            self.get_logger().info(f"   obj {obj_id}: ended on the table at ({d['x']:+.3f}, {d['y']:+.3f})"
                                   f", planned release ({d['release'][0]:+.2f}, {d['release'][1]:+.2f})")
        except Exception:
            pass

    def _slot_for(self, i):
        """
        Slot for object i: the configured one, or the ALT_SLOTS entry farthest from placed objects
        if one already lies closer than MIN_CLEAR (released objects can roll several cm).
        """
        base = self.object_slots[i]
        if not self.placed:
            return base
        clear = lambda s: min(math.hypot(s[0] - px, s[1] - py) for px, py in self.placed)
        if clear(base) >= self.MIN_CLEAR:
            return base
        best = max([base] + self.ALT_SLOTS, key=lambda s: round(clear(s), 3))
        self.get_logger().warn(
            f"slot {base} at {clear(base) * 100:.1f} cm from an object already on the table (minimum {self.MIN_CLEAR * 100:.0f}): "
            f"using {best} ({clear(best) * 100:.1f} cm)")
        return best

    def _library(self):
        """
        One entry per book seen on the shelf, by decreasing y, with metadata, identification method
        and whether it went back to its slot
        """
        res = {b["id"]: b for b in self.results["books"]}
        unres = {u["id"]: u for u in self.results["unresolved"]}
        picked = {e["id"]: e for e in self.results["log"] if e.get("step", "").startswith("4.")}
        lib = []
        for d in sorted(getattr(self, "_books_final", []), key=lambda d: -float(d.get("world_y", 0.0))):
            r = res.get(d["id"]) or {}
            lib.append({
                "id": d["id"],
                "shelf_x": round(float(d.get("world_x", 0.0)), 3),
                "shelf_y": round(float(d.get("world_y", 0.0)), 3),
                "thickness_mm": round(float(d.get("thickness_m", 0.0)) * 1000.0),
                "title": r.get("title") or d.get("title") or None,
                "author": r.get("author") or d.get("author") or None,
                "year": r.get("year") or d.get("year") or None,
                "isbn": r.get("isbn") or d.get("isbn") or None,
                "identified_by": r.get("how") or "non identificato",
                "ocr_title": d.get("title", ""),
                "read_in_hand": d["id"] in picked,
                "returned_to_shelf": (picked[d["id"]]["ok"] if d["id"] in picked else True),
                "unresolved_reason": (unres.get(d["id"]) or {}).get("why"),
            })
        return lib

    def _write(self):
        """
        Write the results JSON to out_file and log a summary
        """
        self.results["library"] = self._library()
        with open(self.out_file, "w", encoding="utf-8") as f:
            json.dump(self.results, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f"RESULT -> {self.out_file}")
        for b in self.results["books"]:
            self.get_logger().info(f"   book {b['id']:>2}: '{b['title']}' - {b['author']} ({b['year']}) ISBN {b['isbn']} [{b['how']}]")
        for b in self.results["library"]:
            self.get_logger().info(f"   LIBRARY {b['id']:>2} y={b['shelf_y']:+.3f}: '{b['title']}' - {b['author']} "
                                   f"({b['year']}) ISBN {b['isbn']} [{b['identified_by']}]")
        for o in self.results["objects"]:
            self.get_logger().info(f"   object {o['id']:>2}: {'sul tavolo' if o['moved'] else 'NON spostato'} {o['slot']}")
        for u in self.results["unresolved"]:
            self.get_logger().warn(f"   unresolved {u['id']}: {u['why']}")


def main(args=None):
    """
    Run the pipeline once; exit code 0 on success
    """
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
