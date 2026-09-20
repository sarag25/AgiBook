#!/usr/bin/env python3
"""
library_pipeline - orchestratore della pipeline automatica (2026-09-13):

  1. foto della libreria (shelf_camera rgbd) -> libri e oggetti con misure 3D
  2. gli OGGETTI (non libri) vengono presi e parcheggiati sul tavolo
     (uno slot ciascuno, object_slots_xy)
  3. seconda foto, ora solo libri: OCR dei dorsi + ricerca su Google Books
     per titolo (library_manager_node, title_lookup) -> metadati
  4. i libri SENZA metadati (niente ISBN) vengono presi UNO ALLA VOLTA E
     TENUTI IN MANO (mai sul tavolo, 2026-09-18 sera): mostrati alla testa
     ruotati (fino a 3 scatti), ISBN dal codice a barre se si legge, poi
     rimessi nello STESSO slot dello scaffale (pick_test_book
     -p put_back:=true) - nessuno slot sul tavolo, nessuna table_camera
     (rimossa il 2026-09-17).
  5. risultato in out_file (JSON: libri con metadati, oggetti parcheggiati
     sul tavolo, libri irrisolti rimasti sullo scaffale) e riepilogo nel log

Ogni presa e' un processo `pick_test_book -p target:=<id>` (con
`-p release_x/y` per gli oggetti destinati al tavolo, con
`-p put_back:=true` senza release per i libri che tornano nello scaffale -
stessa cosa che si fa a mano), le foto passano da library_manager_node
(/library_manager/trigger 'shelf'/'zoom', risultati su
/library_manager/detections). Dal 2026-09-17 pick_test_book sceglie il
braccio (arm:=auto: sinistro per y>0.30, destro altrimenti, vedi
book_placer.py). Da MoveIt 2 (2026-09-18) i tratti liberi delle prese sono
pianificati con le collisioni se `moveit.launch.py` e' acceso.

Prerequisiti: simulazione su, controller attivi, robot nella posa di
lavoro (walk_to_shelf fatto, oppure walk:=true qui), library_manager_node
acceso dalla radice del repo con detector:=depth (o sam3).

  ros2 run agibot_x2_pkg_py library_pipeline
  ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p dry_run:=true      # foto vere, prese solo pianificate
  ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p force_isbn:=true   # ignora i titoli: tutti i libri via ISBN
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
        # Distanza dalla libreria MISURATA dalla depth della head_camera
        # (2026-09-17, walk_to_shelf -p shelf_distance): il robot non si fida
        # piu' della posizione di spawn. shelf_distance_work = valore letto
        # alla posa di lavoro (base_x 1.5, calibrato dal vivo: 0.666 m,
        # ripetibile al mm); la foto da lontano si fa a photo_gap m in piu'.
        # Da 2 m la misura sbaglia di ~5 cm (finestra su superfici diverse):
        # per questo l'ultimo tratto si ricalcola dalla posa della foto (~1 m).
        self.declare_parameter("use_depth", True)
        self.declare_parameter("shelf_distance_work", 0.666)
        self.declare_parameter("photo_gap", 0.9)
        # Fase (2026-09-17): 'ocr' = fino al JSON con i titoli dallo zoom
        # (distanza -> foto -> oggetti sul tavolo -> foto vicina -> zoom OCR
        # -> JSON); 'full' = anche ISBN in mano per i libri non identificati.
        self.declare_parameter("phase", "ocr")
        self.declare_parameter("zoom", True)
        self.declare_parameter("zoom_wait_s", 2400.0)
        self.declare_parameter("force_isbn", False)
        # ISBN dalla camera della testa (2026-09-14): i libri senza metadati
        # vengono mostrati alla testa durante il trasporto (pick_test_book
        # -p head_isbn:=true); se il barcode si legge, niente ri-foto dal tavolo.
        self.declare_parameter("head_isbn", True)     # 2026-09-17: table_camera rimossa
        self.declare_parameter("skip_objects", False)
        self.declare_parameter("skip_books", False)
        # use_last_detections (2026-09-19): niente scatti 'shelf': usa le rilevazioni gia' pubblicate
        # (latched) da library_manager. Serve alla fase libri: la struttura dello scaffale si trova
        # solo dalla foto DA LONTANO (dalla posa di lavoro il ripiano esce dall'inquadratura e le
        # rilevazioni sono 0 libri), quindi quella foto si fa prima (skip_books:=true, fuori dal
        # video) e la registrazione parte dalla posa di lavoro con lo zoom sui dorsi.
        self.declare_parameter("use_last_detections", False)
        # Profondita' di presa dei LIBRI oltre il dorso (2026-09-19). Il default di pick_test_book
        # (25 mm) e' una scelta per gli oggetti: sui libri il MoveIt check_path (primo giro dal vivo)
        # ha trovato il palmo (right_gripper_base_link) a 2.7 mm DENTRO il libro all'ultimo
        # waypoint: fra il palmo e la punta delle dita ci sono ~22 mm. Con 18 mm il palmo resta
        # a ~4 mm dal dorso. NON e' un margine allentato: si cambia dove afferrare, il controllo
        # delle collisioni resta identico.
        self.declare_parameter("book_grasp_depth", 0.018)
        # only_ids "2,3,4": ISBN in mano solo per questi libri (riprendere dopo un errore);
        # arm_by_id "2:left,3:right": braccio per libro (default: auto di pick_test_book, che per il
        # libro 2 al centro dello scaffale sceglie il destro e non ne raggiunge l'avvicinamento).
        self.declare_parameter("only_ids", "")
        self.declare_parameter("arm_by_id", "")
        # grasp_from_top_by_id "3:0.09": presa piu' in alto sul dorso (m dalla cima) per libro. Il libro 3
        # (IT, 221 mm) a meta' altezza ha la ritirata a 27 mm di errore laterale (limite 20): a 9 cm
        # dalla cima 17 mm, avvicinamento 0.8 mm (dry_run del 2026-09-19).
        self.declare_parameter("grasp_from_top_by_id", "")
        # grasp_depth_by_id "3:0.012": profondita' di presa (m oltre il dorso) per libro, al posto di
        # book_grasp_depth. Il palmo sta ~22 mm dietro la punta delle dita: con la pinza ruotata (IT,
        # 11 gradi) a 18 mm il MoveIt check_path trova il palmo a 1.9 mm DENTRO il libro.
        self.declare_parameter("grasp_depth_by_id", "")
        # planner_id (2026-09-19): inoltrato a pick_test_book -p planner_id:=...
        # ("" = RRTConnect di default; "RRTstarkConfigDefault" = sperimentale,
        # vedi config/ompl_planning.yaml)
        self.declare_parameter("planner_id", "")
        # slot sul tavolo (x, y mondo del TCP al rilascio), verificati con l'IK
        # il 2026-09-13: fascia raggiungibile y -0.37..-0.48, x -0.45..+0.05.
        # x portato da 0.03 a -0.15 il 2026-09-19: il tavolo (planning_scene_builder.
        # TABLE_CENTER/TABLE_SIZE) finisce a x=+0.05 - a x=0.03 il mappamondo
        # (rotondo, poco stabile nella pinza: vedi close_until_contact) aveva
        # solo 2 cm di margine dal bordo ed e' caduto dal tavolo dopo il
        # rilascio. -0.15 da' 20 cm di margine, IK verificato offline per
        # entrambe le braccia (errore < 0.3 mm su entrambi gli slot).
        # y allontanate il 2026-09-19 (richiesto per sicurezza, oggetti
        # ravvicinati sul tavolo): verificato offline con l'IK reale che lo
        # slot del braccio DESTRO (secondo oggetto) non puo' andare molto
        # oltre y~-0.51: dalla posa dopo il passo indietro di 15 cm (robot_x
        # -0.25, quella con cui si ricalcola il rilascio) l'errore IK e' 0.1
        # mm a -0.50 e 2.9 mm a -0.51; da robot_x -0.10 gia' 5 mm a -0.52 e
        # 13-27 mm a -0.53/-0.55 (soffitto di portata reale). Con un passo
        # indietro di 30 cm a -0.51 sarebbe 42 mm (ripiego DROP_ARM): il
        # passo indietro NON va allungato. Primo slot (braccio sinistro)
        # spostato da -0.40 a -0.38 (<0.4 mm di errore per qualunque passo
        # indietro fino a 30 cm) senza avvicinarlo al bordo del tavolo
        # (y=-0.33). Distanza fra i due oggetti: 10 -> 13 cm.
        self.declare_parameter("object_slots_xy", [-0.15, -0.38, -0.15, -0.51])
        # sotto la table_camera (x -0.555..-0.025 con hfov 0.9): 2 libri affiancati
        self.declare_parameter("book_slots_xy", [-0.43, -0.37, -0.17, -0.37])
        self.declare_parameter("detections_file", "/tmp/x2_detections.json")
        self.declare_parameter("out_file", "/tmp/x2_library.json")
        self.declare_parameter("shelf_wait_s", 900.0)    # SAM3 su CPU: minuti
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
        self.placed = []          # (x, y) reali degli oggetti gia' posati sul tavolo

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
        if self.use_last:
            t0 = time.monotonic()           # il messaggio latched arriva dopo la scoperta DDS: fino a 20 s
            while not self._dets and time.monotonic() - t0 < 20.0:
                rclpy.spin_once(self, timeout_sec=0.2)
            if not self._dets:
                self.get_logger().error(f"{label}: use_last_detections ma library_manager non ha rilevazioni "
                                        "(fai prima la foto da lontano: skip_books:=true)")
                return None
            self.get_logger().info(f"{label}: uso le rilevazioni gia' fatte (use_last_detections)")
        else:
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

    def walk_robot(self, label, shelf_distance=None, base_x=None):
        """walk_to_shelf: a una distanza dalla libreria misurata dalla depth
        (shelf_distance) oppure a un base_x dato."""
        cmd = ["ros2", "run", "agibot_x2_pkg_py", "walk_to_shelf", "--ros-args"]
        if shelf_distance is not None:
            cmd += ["-p", f"shelf_distance:={shelf_distance:.3f}"]
            self.get_logger().info(f"{label}: cammino fino a {shelf_distance:.3f} m dalla libreria (depth)")
        elif base_x is not None:
            cmd += ["-p", f"distance:={base_x:.3f}"]
            self.get_logger().info(f"{label}: cammino a base_x={base_x:.2f}")
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            self.get_logger().error(f"{label}: walk_to_shelf fallito (exit {rc})")
        return rc == 0

    def zoom_books(self, label):
        """Trigger 'zoom' di library_manager_node: testa su ogni dorso, OCR
        da vicino, ricerca titolo; ritorna le detection aggiornate."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        seq0 = self._det_seq
        self.get_logger().info(f"{label}: foto zoomate dei dorsi + OCR (trigger 'zoom')...")
        self.pub_trigger.publish(String(data="zoom"))
        if not self._wait_seq(seq0, self.zoom_wait, label):
            return None
        books = [d for d in self._dets if d.get("is_book")]
        for d in books:
            self.get_logger().info(
                f"   obj {d['id']:>2} libro y={d['world_y']:+.3f} titolo='{d.get('title','')}' "
                f"autore='{d.get('author','')}' isbn={d.get('isbn','') or '-'}")
        return self._dets

    def pick(self, det, slot, label, head_isbn=False):
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
        self.get_logger().info(f"{label}: obj {det['id']} -> tavolo ({x:+.2f}, {y:+.2f})"
                               + (" [dry_run]" if self.dry_run else ""))
        self.get_logger().info("   $ " + " ".join(cmd))
        t0 = time.time()
        rc = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr).returncode
        ok = rc == 0
        self.get_logger().info(f"{label}: {'OK' if ok else 'FALLITA (exit %d)' % rc} in {time.time()-t0:.0f} s")
        self.results["log"].append({"step": label, "id": det["id"], "slot": [x, y], "ok": ok})
        return ok

    def pick_and_return(self, det, label):
        """Libro senza metadati (2026-09-18, sera): NON va sul tavolo. Preso
        a meta', mostrato alla testa (rotazione, fino a 3 scatti) per
        l'ISBN, poi rimesso nello STESSO slot dello scaffale da cui e' stato
        preso (pick_test_book -p put_back:=true, nessun release_x/y: la
        rimessa usa la posa di presa originale). Il risultato dell'ISBN si
        legge da /tmp/x2_head_isbn_obj<id>.json a prescindere da come e'
        andata la meccanica del pick (quello lo dice il codice di uscita)."""
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
        self.get_logger().info(f"{label}: obj {det['id']} -> in mano per l'ISBN, poi di nuovo nello scaffale"
                               + (" [dry_run]" if self.dry_run else ""))
        self.get_logger().info("   $ " + " ".join(cmd))
        t0 = time.time()
        rc = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr).returncode
        ok = rc == 0
        self.get_logger().info(f"{label}: {'OK' if ok else 'FALLITA (exit %d)' % rc} in {time.time()-t0:.0f} s")
        self.results["log"].append({"step": label, "id": det["id"], "ok": ok})
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
            "libro in mano (mai sul tavolo) davanti alla testa per l'ISBN, poi rimesso "
            "nello stesso slot dello scaffale, per i libri non identificati"
            + (" [DRY RUN: prese solo pianificate]" if self.dry_run else ""))
        # 0-1. distanza dalla libreria (depth) e prima foto da lontano
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
                # da ~1.6 m la misura sbaglia ~1 cm: seconda passata da vicino
                # (mm) -> il bacino finisce a +-1 mm dalla posa nominale
                ok = (self.walk_robot("1b. posa di lavoro", shelf_distance=self.d_work)
                      and self.walk_robot("1c. correzione fine", shelf_distance=self.d_work))
            else:
                self.get_logger().info("1b. cammino fino alla posa di lavoro")
                ok = subprocess.run(["ros2", "run", "agibot_x2_pkg_py", "walk_to_shelf"]).returncode == 0
            if not ok:
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
                slot = self._slot_for(i)
                ok = self.pick(o, slot, f"2.{i+1} oggetto {o['id']}")
                self._read_placed(o["id"])
                self.results["objects"].append({"id": o["id"], "class": o.get("class"),
                                                "slot": list(slot), "moved": ok})
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
        if self.zoom:
            dets3 = self.zoom_books("3b. zoom sui dorsi")
            if dets3 is not None:
                dets2 = dets3
        books = [d for d in dets2 if d.get("is_book")]
        self._books_final = books        # per la sezione "library" del JSON (_write)
        identified = [b for b in books if b.get("isbn") and not self.force_isbn]
        todo = [b for b in books if b not in identified and (not self.only_ids or b["id"] in self.only_ids)]
        self.get_logger().info(
            f"3. identificati dal titolo: {[b['id'] for b in identified]}; da portare sul tavolo per "
            f"l'ISBN: {[b['id'] for b in todo]}" + (" (force_isbn)" if self.force_isbn else ""))
        for b in identified:
            self.results["books"].append({"id": b["id"], "title": b.get("title"), "author": b.get("author"),
                                          "year": b.get("year"), "isbn": b.get("isbn"), "how": "ocr+google_books",
                                          "where": "shelf"})
        if self.phase != "full":
            for b in todo:
                self.results["unresolved"].append({"id": b["id"], "why": "titolo non identificato dall'OCR (fase ocr)",
                                                   "ocr_title": b.get("title", ""), "where": "shelf"})
            self._write()
            self.get_logger().info("Fase 'ocr' completata (phase:=full per continuare con l'ISBN in mano)")
            return True

        # 4. libri senza metadati (2026-09-18, sera): uno alla volta IN MANO
        # davanti alla testa per l'ISBN, MAI sul tavolo - tornano nello
        # stesso slot dello scaffale (pick_test_book -p put_back:=true).
        for k, b in enumerate(todo):
            if not self.pick_and_return(b, f"4.{k+1} libro {b['id']}"):
                self.get_logger().error("presa fallita: mi fermo")
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
                    self.get_logger().warn(f"4.{k+1} libro {b['id']}: risultato dalla testa non trovato ({e})")
                if h and h.get("isbn"):
                    upd = h
                    self.get_logger().info(f"4.{k+1} libro {b['id']}: ISBN dalla testa {h['isbn']} -> "
                                           f"'{h.get('title')}' {h.get('author')} {h.get('year')}")
                else:
                    self.get_logger().info(f"4.{k+1} libro {b['id']}: dalla testa nessun ISBN - resta sullo scaffale senza metadati")
            if upd:
                self.results["books"].append({"id": b["id"], "title": upd.get("title"), "author": upd.get("author"),
                                              "year": upd.get("year"), "isbn": upd.get("isbn"), "how": "isbn_head",
                                              "where": "shelf"})
            else:
                self.results["unresolved"].append({"id": b["id"], "why": "ISBN non letto" if not self.dry_run else "dry_run",
                                                   "where": "shelf"})
        self._write()
        return True

    # slot alternativi per il braccio DESTRO, tutti con IK verificato offline (errore < 4 mm dalla posa
    # dopo il passo indietro) e con margine dal bordo del tavolo (x_max = +0.05)
    ALT_SLOTS = [(-0.05, -0.40), (-0.05, -0.45), (-0.05, -0.50), (-0.15, -0.45), (-0.15, -0.40)]
    MIN_CLEAR = 0.11      # distanza minima fra i centri di due oggetti sul tavolo (raggi ~3.3 + 2.5 cm + 5 cm di margine)

    def _read_placed(self, obj_id):
        """Dove e' finito DAVVERO l'oggetto (scritto da pick_test_book a fine sequenza)."""
        try:
            with open(f"/tmp/x2_placed_obj{obj_id}.json", encoding="utf-8") as f:
                d = json.load(f)
            self.placed.append((float(d["x"]), float(d["y"])))
            self.get_logger().info(f"   obj {obj_id}: finito sul tavolo a ({d['x']:+.3f}, {d['y']:+.3f})"
                                   f", rilascio previsto ({d['release'][0]:+.2f}, {d['release'][1]:+.2f})")
        except Exception:
            pass

    def _slot_for(self, i):
        """Slot per l'oggetto i: quello configurato, ma se un oggetto e' gia' sul tavolo a meno di
        MIN_CLEAR (2026-09-19: il mappamondo, rilasciato a y=-0.38, e' finito a -0.453: a 5.7 cm dallo
        slot del portapenne) si sceglie fra ALT_SLOTS quello piu' lontano da tutti gli oggetti gia'
        posati (preferendo l'ordine della lista)."""
        base = self.object_slots[i]
        if not self.placed:
            return base
        clear = lambda s: min(math.hypot(s[0] - px, s[1] - py) for px, py in self.placed)
        if clear(base) >= self.MIN_CLEAR:
            return base
        best = max([base] + self.ALT_SLOTS, key=lambda s: round(clear(s), 3))
        self.get_logger().warn(
            f"slot {base} a {clear(base) * 100:.1f} cm da un oggetto gia' sul tavolo (minimo {self.MIN_CLEAR * 100:.0f}): "
            f"uso {best} ({clear(best) * 100:.1f} cm)")
        return best

    def _library(self):
        """Una voce per OGNI libro visto sullo scaffale, in ordine di posizione
        (y decrescente), con tutti i dati disponibili: titolo, autore, anno, ISBN,
        come e' stato identificato, se e' tornato nel suo slot (2026-09-19)."""
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
        self.results["library"] = self._library()
        with open(self.out_file, "w", encoding="utf-8") as f:
            json.dump(self.results, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f"RISULTATO -> {self.out_file}")
        for b in self.results["books"]:
            self.get_logger().info(f"   libro {b['id']:>2}: '{b['title']}' - {b['author']} ({b['year']}) ISBN {b['isbn']} [{b['how']}]")
        for b in self.results["library"]:
            self.get_logger().info(f"   LIBRERIA {b['id']:>2} y={b['shelf_y']:+.3f}: '{b['title']}' - {b['author']} "
                                   f"({b['year']}) ISBN {b['isbn']} [{b['identified_by']}]")
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
