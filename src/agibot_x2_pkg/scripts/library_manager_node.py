#!/usr/bin/env python3
"""
Nodo ROS2 principale: Library Manager.

Orchestrates:
  1. Ricezione immagine (camera RGBD del robot o file utente)
  2. Detection oggetti (YOLOv8)
  3. Analisi colore (KMeans) + OCR (EasyOCR) + Profondità (MiDaS/RGBD)
  4. Parsing struttura scaffale
  5. Ricezione comando utente (testo o voce)
  6. Calcolo piano di riordinamento
  7. Esecuzione sequenza di azioni sul robot X2

Topic ROS2:
  SUB  /camera/image_raw          (sensor_msgs/Image)   immagine RGB per
                                   identificazione (titolo/autore/colore) -
                                   pensata per una foto di qualità (file
                                   trigger sotto, o un'immagine equivalente),
                                   non per il feed live della simulazione:
                                   vedi /head_camera/image sotto per quello.
  SUB  /camera/depth/image_raw    (sensor_msgs/Image)   depth map (opzionale,
                                   stesso discorso sopra)
  SUB  /head_camera/image + depth_image (sensor_msgs/Image) camera della testa,
                                   rgbd a scatto (2026-09-16, vedi Gazebo.md
                                   § "Camere") - usato SOLO per un controllo
                                   veloce di occupazione scaffale (righe/slot
                                   liberi) prima di un PLACE, non per
                                   identificazione: niente OCR/colore qui,
                                   solo detection + shelf_parser. Tenuto
                                   deliberatamente separato da /camera/image_raw
                                   per non mischiare una foto di qualità nota
                                   con un frame in tempo reale a bassa
                                   risoluzione (vedi _refresh_shelf_occupancy).
  SUB  /library_manager/command   (std_msgs/String)     comando testuale utente
  PUB  /library_manager/status    (std_msgs/String)     stato corrente
  PUB  /library_manager/detections (std_msgs/String)    JSON oggetti rilevati

Uso:
  ros2 run x2_description library_manager_node

  # Invia un'immagine:
  ros2 topic pub /library_manager/trigger std_msgs/String "data: '/path/to/shelf.jpg'"

  # Invia il comando:
  ros2 topic pub /library_manager/command std_msgs/String "data: 'ordina per colore'"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import cv2
import threading
import json
import numpy as np
import logging
import os

# Pipeline CV e planning (importa dai moduli locali)
import sys
sys.path.insert(0, os.path.dirname(__file__))

from vision.book_detector import BookDetector
from vision.color_analyzer import ColorAnalyzer
from vision.ocr_reader import OCRReader
from vision.depth_estimator import DepthEstimator
from vision.shelf_parser import ShelfParser
from vision.shelf_geometry import ShelfGeometry, HeadCameraPose, HEAD_CAM_HFOV, HEAD_CAM_SIZE
from sorting.sort_planner import SortPlanner
from sorting.action_sequencer import ActionSequencer
from input.input_handler import InputHandler, SortCriterion

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("library_manager")


class LibraryManagerNode(Node):

    def __init__(self):
        super().__init__("library_manager")

        # ── Parametri ROS2 ──────────────────────────────────────────────
        # detector: "yolo" (default, veloce ma debole sui dorsi sottili -
        # 4/15 libri al primo run reale) oppure "sam3" (SAM3 di Meta con
        # prompt testuale, molto piu' adatto ai libri affiancati - stessa
        # tecnologia della pipeline standalone sorting/detect_sam3.py, vedi
        # Sorting.md; richiede HF_TOKEN e primo download pesante, inferenza
        # CPU lenta ma una tantum per foto). 2026-08-29.
        self.declare_parameter("detector",       "yolo")
        self.declare_parameter("yolo_model",     "yolov8n.pt")
        # Soglia di confidenza YOLO (2026-08-30, era 0.35 fissa nel
        # BookDetector): i dorsi di libro a 320x240 escono con confidenze
        # basse, 0.25 recupera detection valide senza inondare di falsi.
        self.declare_parameter("yolo_conf",      0.25)
        self.declare_parameter("depth_mode",     "midas")
        self.declare_parameter("use_llm",        False)
        self.declare_parameter("ocr_languages",  ["it", "en"])
        self.declare_parameter("default_sort",   "color")
        # plan_only (2026-09-06): True = calcola e pubblica il piano
        # (/library_manager/plan + /tmp/x2_sort_plan.json) ma NON muove il
        # robot. Per provare i criteri di ordinamento sul JSON senza
        # coreografie.
        self.declare_parameter("plan_only",      False)
        # Posa della libreria nel mondo (= shelf_x/shelf_y/shelf_yaw_deg del
        # launch): serve a ShelfGeometry per riproiettare la depth della
        # shelf_camera in coordinate mondo (2026-09-13).
        # Dopo l'OCR dei dorsi cerca i metadati per titolo su Google Books
        # (sorting/extract_isbn.search_book_by_title): un libro con isbn nel
        # JSON e' "identificato"; gli altri vanno sul tavolo per l'ISBN.
        self.declare_parameter("title_lookup",   True)
        self.declare_parameter("shelf_x",        0.40)
        self.declare_parameter("shelf_y",        0.0)
        self.declare_parameter("shelf_yaw_deg",  90.0)
        # Camera per la foto della libreria (2026-09-16, studio "una sola
        # camera" in Gazebo.md): "head" = head_camera sulla testa del robot
        # (rgbd 1920x1440 a scatto; posa nel mondo dai giunti al momento
        # dello scatto, vedi HeadCameraPose), "shelf" = shelf_camera fissa
        # alla libreria (bookshelf.urdf, in via di rimozione).
        self.declare_parameter("camera",         "head")
        # Posa di SPAWN del robot (link 'world' del modello): il launch lo
        # mette a x = ROBOT_SPAWN_X - walk_distance (default -0.1 - 1.5).
        # Con walk_distance:=0 passare robot_spawn_x:=-0.1.
        try:
            from agibot_x2_pkg.book_placer import ROBOT_SPAWN_X, WALK_DISTANCE
            _spawn_x_default = ROBOT_SPAWN_X - WALK_DISTANCE
        except Exception:
            _spawn_x_default = -1.6
        self.declare_parameter("robot_spawn_x",  float(_spawn_x_default))
        self.declare_parameter("robot_spawn_y",  0.0)
        self.declare_parameter("robot_spawn_yaw_deg", 0.0)
        # Posa di sguardo prima dello scatto: l'asse ottico della camera
        # della testa e' 40 gradi sotto l'orizzonte a testa dritta; con
        # head_pitch -0.38 e busto indietro -0.31 diventa orizzontale e da
        # 0.8-1.3 m si vede tutta la libreria (misurato il 2026-09-16).
        self.declare_parameter("look_pose",      True)
        self.declare_parameter("look_head_yaw",  0.0)
        self.declare_parameter("look_head_pitch", -0.38)
        self.declare_parameter("look_waist_pitch", -0.31)

        detector     = self.get_parameter("detector").value
        yolo_model   = self.get_parameter("yolo_model").value
        yolo_conf    = float(self.get_parameter("yolo_conf").value)
        depth_mode   = self.get_parameter("depth_mode").value
        use_llm      = self.get_parameter("use_llm").value
        ocr_langs    = self.get_parameter("ocr_languages").value
        default_sort = self.get_parameter("default_sort").value
        self._plan_only = bool(self.get_parameter("plan_only").value)

        # ── Pipeline CV ─────────────────────────────────────────────────
        self.get_logger().info("Inizializzazione pipeline CV...")
        import math as _math
        self.geometry = ShelfGeometry(
            shelf_x=float(self.get_parameter("shelf_x").value),
            shelf_y=float(self.get_parameter("shelf_y").value),
            shelf_yaw=_math.radians(float(self.get_parameter("shelf_yaw_deg").value)))
        self._shelf_depth: np.ndarray | None = None
        self._shelf_depth_seq = 0
        self._camera = str(self.get_parameter("camera").value).strip().lower()
        self._joints: dict = {}
        self.head_pose = None
        if self._camera == "head":
            spawn = (float(self.get_parameter("robot_spawn_x").value),
                     float(self.get_parameter("robot_spawn_y").value), 0.0)
            try:
                self.head_pose = HeadCameraPose(
                    spawn_xyz=spawn, spawn_yaw=_math.radians(float(self.get_parameter("robot_spawn_yaw_deg").value)))
                self.get_logger().info(
                    f"Camera della libreria: head_camera (rgbd a scatto sulla testa), spawn del robot a "
                    f"x={spawn[0]:.2f} y={spawn[1]:.2f}; posa della camera dai giunti a ogni scatto")
            except Exception as e:
                self.get_logger().error(
                    f"HeadCameraPose non disponibile ({e}): serve agibot_x2_pkg_py compilato. "
                    "Uso la posa della shelf_camera (misure 3D SBAGLIATE con la camera della testa)")
        else:
            self.get_logger().info("Camera della libreria: shelf_camera fissa (bookshelf.urdf)")
        if detector == "none":
            # Nessun detector (2026-09-08): per le prove che non passano
            # dalla foto dello scaffale (ri-foto dal tavolo, barcode/ISBN,
            # comandi di ordinamento su detection gia' note) - risparmia
            # 3-4 GB di RAM e minuti di avvio rispetto a SAM3. Con 8 GB di
            # VM WSL2 e Gazebo server+GUI (2.3 GB l'uno) torch non riusciva
            # nemmeno a importarsi ("Cannot allocate memory").
            self.detector = None
        elif detector == "sam3":
            from vision.sam3_detector import Sam3BookDetector
            self.detector = Sam3BookDetector()
        elif detector == "depth":
            if depth_mode == "midas":
                # niente MiDaS/torch: la profondita' arriva dalla camera
                self.get_logger().info("detector=depth: depth_mode midas -> rgbd (niente torch)")
                depth_mode = "rgbd"
            # Solo geometria dalla depth della shelf_camera (2026-09-13):
            # niente rete neurale, parte in un secondo, non identifica i
            # titoli (colore/OCR restano ai moduli dopo). Vedi
            # vision/depth_detector.py.
            from vision.depth_detector import DepthShelfDetector
            self.detector = DepthShelfDetector(self.geometry)
        else:
            self.detector = BookDetector(model_path=yolo_model,
                                         conf_threshold=yolo_conf)
        self.colorizer = ColorAnalyzer()
        self.ocr       = OCRReader(languages=ocr_langs)
        self.depth_est = DepthEstimator(mode=depth_mode)
        self.shelf_parser = ShelfParser()

        # ── Planning ────────────────────────────────────────────────────
        self.sort_planner  = SortPlanner(color_analyzer=self.colorizer)
        self.action_seq    = ActionSequencer()
        self.input_handler = InputHandler(use_llm=use_llm)

        # ── Stato interno ───────────────────────────────────────────────
        self._current_image: np.ndarray | None = None
        self._depth_image:   np.ndarray | None = None
        # Ultimo frame della camera testa live in Gazebo - separato da
        # _current_image apposta, vedi commento su /head_camera/image
        # nel docstring del modulo.
        self._live_shelf_image: np.ndarray | None = None
        # Foto frontale ad alta risoluzione dei 4 libri (960x720, camera
        # "fototessera" fissa alla libreria - vedi bookshelf.urdf, 2026-09-06):
        # e' la "foto primo scaffale fornita al robot" della PIPELINE in
        # src/TODO, la sorgente giusta per l'OCR dei titoli (la camera testa
        # a 320x240 non li legge). Trigger: data 'shelf'.
        self._shelf_photo_image: np.ndarray | None = None
        # Ultimo frame della camera tavolo (2026-08-14, vedi full_scene.urdf/
        # Gazebo.md § "Camere") - usato SOLO da _rephotograph_on_table per
        # completare title/author/color_name di un libro appena depositato
        # sul tavolo di staging, se la foto di identificazione iniziale
        # (scaffale, più larga/più lontana) non li aveva letti.
        self._table_image: np.ndarray | None = None
        # Camere A SCATTO (2026-09-08): shelf_camera e table_camera sono
        # "triggered" in Gazebo (bookshelf.urdf/table.urdf): nessun frame
        # finche' non si pubblica true su /<camera>/trigger. _capture()
        # scatta e aspetta il frame nuovo (contatore di sequenza).
        self._shelf_seq = 0
        self._table_seq = 0
        if self._camera == "head":
            cam_img, cam_depth, cam_trig = "/head_camera/image", "/head_camera/depth_image", "/head_camera/trigger"
        else:
            cam_img, cam_depth, cam_trig = "/shelf_camera/image", "/shelf_camera/depth_image", "/shelf_camera/trigger"
        self._cam_topics = (cam_img, cam_depth, cam_trig)
        self._pub_shelf_trigger = self.create_publisher(Bool, cam_trig, 10)
        self._pub_table_trigger = self.create_publisher(Bool, "/table_camera/trigger", 10)
        # posa di sguardo (camera della testa): testa e busto prima dello scatto
        self._head_client = ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
        self._waist_client = ActionClient(self, FollowJointTrajectory, "/waist_controller/follow_joint_trajectory")
        self._detected_objects = []
        self._pending_command: str = default_sort

        # ── Sub/Pub ─────────────────────────────────────────────────────
        self.create_subscription(String, "/library_manager/trigger",
                                 self._on_image_trigger, 10)
        self.create_subscription(Image,  "/camera/image_raw",
                                 self._on_camera_image, 10)
        self.create_subscription(Image,  "/camera/depth/image_raw",
                                 self._on_depth_image, 10)
        # Foto della libreria: head_camera (default) o shelf_camera, immagine
        # + profondita' allineata dello stesso scatto (rgbd, 2026-09-13/16).
        # La RGBD 320x240 continua della testa non esiste piu' (2026-09-16):
        # _live_shelf_image e' l'ultimo scatto della camera della libreria.
        self.create_subscription(Image,  cam_img,
                                 self._on_shelf_camera_image, 10)
        self.create_subscription(Image,  cam_depth,
                                 self._on_shelf_depth_image, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Image,  "/table_camera/image",
                                 self._on_table_camera_image, 10)
        self.create_subscription(String, "/library_manager/command",
                                 self._on_user_command, 10)
        # Ri-fotografia MANUALE dal tavolo (2026-08-30): nella sequenza
        # automatica scatta da sola dopo il deposito (on_staging_table), ma
        # quando il libro lo porta l'utente col teleop serve un comando
        # esplicito. data = obj_id del libro (dai log/detections), oppure
        # vuoto = primo libro rilevato con titolo/autore mancanti.
        self.create_subscription(String, "/library_manager/rephotograph",
                                 self._on_rephotograph_cmd, 10)

        # QoS transient_local (2026-08-30): status e detections sono
        # pubblicati "a evento" (una volta a fine analisi) - con QoS volatile
        # un `ros2 topic echo` lanciato DOPO l'analisi resta in attesa per
        # sempre. Con transient_local (depth 1) chi si sottoscrive dopo
        # riceve comunque l'ultimo messaggio.
        from rclpy.qos import QoSProfile, DurabilityPolicy
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_status     = self.create_publisher(String,
                                "/library_manager/status", latched)
        self._pub_detections = self.create_publisher(String,
                                "/library_manager/detections", latched)
        # Piano di ordinamento come JSON (2026-09-06): prima viveva solo nel
        # log (plan.describe()) e non c'era modo di consultarlo dopo.
        self._pub_plan       = self.create_publisher(String,
                                "/library_manager/plan", latched)

        # ── Action clients robot ─────────────────────────────────────────
        self._arm_client     = ActionClient(self, FollowJointTrajectory,
                               "/left_arm_controller/follow_joint_trajectory")
        self._gripper_client = ActionClient(self, FollowJointTrajectory,
                               "/left_gripper_controller/follow_joint_trajectory")
        self._head_client    = ActionClient(self, FollowJointTrajectory,
                               "/head_controller/follow_joint_trajectory")
        self._waist_client   = ActionClient(self, FollowJointTrajectory,
                               "/waist_controller/follow_joint_trajectory")

        # Thread di lavoro per la pipeline (2026-08-29): la pipeline NON
        # puo' girare dentro una callback ROS - _execute_actions aspetta i
        # risultati delle action con spin_until_future_complete, che dentro
        # una callback esplode con "Executor is already spinning" (visto al
        # primo run reale con i controller finalmente attivi). Girando in un
        # thread separato, il thread principale resta libero di processare
        # callback (comprese le camere: i frame live continuano ad
        # aggiornarsi DURANTE l'esecuzione delle azioni, prima erano
        # congelati) e le attese sulle future diventano eventi (_wait_future).
        self._pipeline_thread: threading.Thread | None = None

        self.get_logger().info("LibraryManagerNode pronto.")
        self._publish_status("idle")

    def _start_in_background(self, target):
        """Lancia un passo di pipeline in un thread, una alla volta."""
        if self._pipeline_thread is not None and self._pipeline_thread.is_alive():
            self.get_logger().warn(
                "Pipeline gia' in esecuzione: richiesta ignorata.")
            return
        self._pipeline_thread = threading.Thread(target=target, daemon=True)
        self._pipeline_thread.start()

    def _wait_future(self, future, timeout_sec: float = 60.0):
        """
        Attende una future rclpy da un thread di lavoro SENZA spinnare
        (l'executor gira gia' nel thread principale e completa la future;
        qui basta aspettare l'evento).
        """
        done_evt = threading.Event()
        future.add_done_callback(lambda _f: done_evt.set())
        if not done_evt.wait(timeout_sec):
            self.get_logger().warn("Timeout in attesa di una action")
            return None
        return future.result()

    # ─── Callback immagine da file (simulazione) ──────────────────────────

    def _on_image_trigger(self, msg: String):
        """Carica un'immagine da file path per simulare la camera.

        Valore speciale "head" (2026-08-30): usa l'ULTIMO frame live della
        camera testa in Gazebo come immagine di identificazione - il robot
        "guarda e analizza" quello che ha davvero davanti (es. i libri
        fisici di test), senza passare da un file. A 320x240 l'OCR dei
        titoli sara' spesso parziale: e' il caso d'uso pensato per essere
        completato dalla ri-fotografia sul tavolo (vedi
        /library_manager/rephotograph).
        """
        path = msg.data.strip()
        if path.lower() == "shelf":
            # Foto "fototessera" 960x720 della camera fissa davanti allo
            # scaffale: la via giusta per l'identificazione completa
            # (OCR compreso) nella scena grasp_test.
            def _shoot_and_run():
                img = self._capture("shelf")
                if img is None:
                    return
                self.get_logger().info(
                    f"Analizzo la foto della camera scaffale ({img.shape[1]}x{img.shape[0]})")
                self._current_image = img
                self._run_pipeline()
            self._start_in_background(_shoot_and_run)
            return
        if path.lower() == "zoom":
            # Foto zoomata per libro (2026-09-17): robot alla posa di lavoro,
            # testa puntata su ogni dorso gia' misurato, OCR sul dorso
            # proiettato (40-50 px/cm: si legge anche il sottotitolo, es.
            # "L'alba sulla mietitura" sotto "HUNGER GAMES").
            self._start_in_background(self._zoom_books)
            return
        if path.lower() == "head":
            if self._live_shelf_image is None:
                self.get_logger().error(
                    "Nessun frame dalla camera testa: Gazebo e' attivo?")
                return
            self.get_logger().info("Analizzo il frame live della camera testa")
            self._current_image = self._live_shelf_image.copy()
            self._start_in_background(self._run_pipeline)
            return
        if not os.path.exists(path):
            self.get_logger().error(f"File non trovato: {path}")
            return

        self.get_logger().info(f"Carico immagine: {path}")
        self._current_image = cv2.imread(path)
        if self._current_image is None:
            self.get_logger().error("Impossibile leggere l'immagine")
            return

        self._start_in_background(self._run_pipeline)

    # ─── Callback camera ROS2 ─────────────────────────────────────────────

    @staticmethod
    def _img_msg_to_bgr(msg: Image) -> np.ndarray:
        """
        sensor_msgs/Image -> array BGR (convenzione OpenCV/YOLO/EasyOCR).
        Fix 2026-08-30: le camere Gazebo pubblicano rgb8, ma i byte grezzi
        venivano usati come se fossero BGR -> rosso e blu scambiati (legno
        della libreria blu nell'immagine di debug), e YOLO/colore/OCR
        lavoravano su colori invertiti: dalla camera testa trovava solo
        "mouse" (la pallina) e nessun libro, pur vedendoli chiaramente.
        """
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()
        if enc in ("rgb8", "bgr8"):
            img = arr.reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
        if enc in ("rgba8", "bgra8"):
            img = arr.reshape(msg.height, msg.width, 4)
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR)
        if enc in ("mono8", "8uc1"):
            return cv2.cvtColor(arr.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
        # encoding sconosciuto: comportamento precedente (best effort)
        return arr.reshape(msg.height, msg.width, -1).copy()

    def _on_camera_image(self, msg: Image):
        self._current_image = self._img_msg_to_bgr(msg)

    def _capture(self, camera: str, timeout_s: float = 30.0):
        """Scatta con la camera 'shelf' o 'table' (triggered) e ritorna il
        frame NUOVO, o None se non arriva entro timeout_s (wall). Da chiamare
        SOLO da un thread di lavoro (_start_in_background): qui si aspetta,
        e il frame arriva su un callback dell'executor. Il timeout e' largo:
        a RTF 10-20% un frame a 1 Hz sim impiega 5-10 s reali."""
        import time as _time
        if camera == "shelf":
            pub, seq0, get = self._pub_shelf_trigger, self._shelf_seq, lambda: (self._shelf_seq, self._shelf_photo_image)
        else:
            pub, seq0, get = self._pub_table_trigger, self._table_seq, lambda: (self._table_seq, self._table_image)
        topic = self._cam_topics[2] if camera == "shelf" else "/table_camera/trigger"
        looked = False
        if camera == "shelf" and self._camera == "head":
            if bool(self.get_parameter("look_pose").value):
                self._look_at_shelf()
                looked = True
            self._apply_head_camera_pose()
        self.get_logger().info(f"Scatto {camera} ({topic})...")
        t0 = _time.monotonic()
        pub.publish(Bool(data=True))
        depth_seq0 = self._shelf_depth_seq
        while _time.monotonic() - t0 < timeout_s:
            seq, img = get()
            if seq > seq0 and img is not None:
                if camera == "shelf":
                    # la depth dello stesso scatto arriva a parte: aspettala
                    # (al massimo 5 s; senza rgbd - URDF vecchio - si va avanti
                    # con la sola immagine e niente geometria)
                    t1 = _time.monotonic()
                    while self._shelf_depth_seq <= depth_seq0 and _time.monotonic() - t1 < 5.0:
                        _time.sleep(0.1)
                    if self._shelf_depth_seq <= depth_seq0:
                        self.get_logger().warn(
                            f"Nessuna depth da {self._cam_topics[1]}: la camera non e' rgbd "
                            "(URDF vecchio?) - niente misure 3D dei libri")
                self.get_logger().info(f"Frame {camera} ({img.shape[1]}x{img.shape[0]}) ricevuto in {_time.monotonic()-t0:.1f} s")
                lit = float((img.max(axis=2) > 40).mean()) if img.ndim == 3 else 1.0
                if lit < 0.3 and (_time.monotonic() - t0) < timeout_s - 10:
                    # Frame scuro: e' il PRIMO render di un sensore rgbd
                    # (gz-sensors: la connessione al point cloud colorato
                    # nasce dentro Update(), dopo che Ogre2DepthCamera::
                    # PreRender ha gia' disattivato il pass colore) - il
                    # launch fa uno scatto di riscaldamento all'avvio, qui
                    # resta la rete di sicurezza. Vedi Bugs.md 2026-09-17.
                    self.get_logger().warn(f"Frame {camera} scuro ({lit*100:.0f}% pixel illuminati): riscatto")
                    seq0 = seq
                    pub.publish(Bool(data=True))
                    depth_seq0 = self._shelf_depth_seq
                    _time.sleep(0.5)
                    continue
                out = img.copy()
                if looked:
                    self._look_restore()
                return out
            if (_time.monotonic() - t0) % 10 < 0.25:   # ripeti lo scatto ogni ~10 s
                pub.publish(Bool(data=True))
            _time.sleep(0.2)
        if looked:
            self._look_restore()
        self.get_logger().error(
            f"Nessun frame da {self._cam_topics[0] if camera == 'shelf' else '/table_camera/image'} entro {timeout_s:.0f} s: la scena e' "
            "grasp_test, la simulazione gira e il bridge ha la voce "
            f"{topic}? (camere a scatto: vedi bookshelf.urdf/table.urdf)")
        return None

    def _on_shelf_camera_image(self, msg: Image):
        """Cache l'ultima foto della camera della libreria (testa o fissa)."""
        self._shelf_photo_image = self._img_msg_to_bgr(msg)
        self._live_shelf_image = self._shelf_photo_image
        self._shelf_seq += 1

    def _on_joint_states(self, msg: JointState):
        self._joints = dict(zip(msg.name, msg.position))

    # ─── posa di sguardo (camera della testa) ─────────────────────────────

    def _send_and_wait(self, client, names, positions, duration_s, label, timeout_s=180.0):
        """Goal FollowJointTrajectory dal thread di lavoro: le future vengono
        completate dall'executor nel thread principale, qui si aspetta a
        polling (niente spin da qui). RTF basso: un goal da 2 s sim puo'
        durare un minuto reale."""
        import time as _time
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"{label}: action server non disponibile, salto")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9))
        goal.trajectory.points.append(pt)
        fut = client.send_goal_async(goal)
        t0 = _time.monotonic()
        while not fut.done() and _time.monotonic() - t0 < 30.0:
            _time.sleep(0.1)
        handle = fut.result() if fut.done() else None
        if handle is None or not handle.accepted:
            self.get_logger().warn(f"{label}: goal rifiutato")
            return False
        res = handle.get_result_async()
        while not res.done() and _time.monotonic() - t0 < timeout_s:
            _time.sleep(0.1)
        self.get_logger().info(f"{label}: fatto ({_time.monotonic()-t0:.0f} s)")
        return res.done()

    def _send_many_and_wait(self, goals, timeout_s=180.0):
        """Piu' goal FollowJointTrajectory in PARALLELO (busto e testa
        insieme: a RTF 0.05 due goal da 2 s in sequenza costavano 60-80 s
        reali per libro, 2026-09-17). goals: lista di (client, names,
        positions, durata, label)."""
        import time as _time
        handles = []
        t0 = _time.monotonic()
        for client, names, positions, duration_s, label in goals:
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn(f"{label}: action server non disponibile, salto")
                continue
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in positions]
            pt.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9))
            goal.trajectory.points.append(pt)
            handles.append((client.send_goal_async(goal), label))
        results = []
        for fut, label in handles:
            while not fut.done() and _time.monotonic() - t0 < 30.0:
                _time.sleep(0.05)
            h = fut.result() if fut.done() else None
            if h is None or not h.accepted:
                self.get_logger().warn(f"{label}: goal rifiutato")
                continue
            results.append((h.get_result_async(), label))
        for res, label in results:
            while not res.done() and _time.monotonic() - t0 < timeout_s:
                _time.sleep(0.05)
        self.get_logger().info(f"{' + '.join(l for _, l in results)}: fatto ({_time.monotonic()-t0:.0f} s)")
        return all(r.done() for r, _ in results)

    def _look_at_shelf(self):
        """Testa su e busto indietro (parametri look_*), poi ritorna i giunti
        letti: e' la posa con cui si calcola la camera per lo scatto."""
        import time as _time
        hy, hp = float(self.get_parameter("look_head_yaw").value), float(self.get_parameter("look_head_pitch").value)
        wp = float(self.get_parameter("look_waist_pitch").value)
        wy = float(self._joints.get("waist_yaw_joint", 0.0))
        self._send_many_and_wait([
            (self._head_client, ["head_yaw_joint", "head_pitch_joint"], [hy, hp], 2.0, f"sguardo: testa ({hy:+.2f}, {hp:+.2f})"),
            (self._waist_client, ["waist_yaw_joint", "waist_pitch_joint"], [wy, wp], 2.0, f"sguardo: busto (yaw {wy:+.2f}, pitch {wp:+.2f})")])
        _time.sleep(1.0)     # assestamento + /joint_states aggiornati

    def _look_restore(self):
        wy = float(self._joints.get("waist_yaw_joint", 0.0))
        self._send_many_and_wait([
            (self._head_client, ["head_yaw_joint", "head_pitch_joint"], [0.0, 0.0], 2.0, "sguardo: testa dritta"),
            (self._waist_client, ["waist_yaw_joint", "waist_pitch_joint"], [wy, 0.0], 2.0, "sguardo: busto dritto")])

    def _aim_head_at(self, target, waist_pitch=-0.31):
        """(waist_yaw, head_pitch, errore_rad, distanza) che puntano l'asse
        ottico della head_camera sul punto mondo `target`, con busto
        indietro e testa dritta in yaw (ricerca a griglia sulla FK)."""
        import math as _m
        j = dict(self._joints)
        tgt = np.asarray(target, dtype=float)
        best = None
        for wy in np.linspace(-1.2, 1.2, 97):
            for hp in np.linspace(-0.38, 0.38, 39):
                q = dict(j)
                q.update({"waist_yaw_joint": float(wy), "waist_pitch_joint": waist_pitch,
                          "head_yaw_joint": 0.0, "head_pitch_joint": float(hp)})
                R, t = self.head_pose.world_pose(q)
                v = tgt - t
                dist = float(np.linalg.norm(v))
                ang = _m.acos(max(-1.0, min(1.0, float(R[:, 0] @ (v / dist)))))
                if best is None or ang < best[2]:
                    best = (float(wy), float(hp), ang, dist)
        return best

    def _zoom_books(self):
        """Per ogni libro rilevato: punta la testa sul dorso, scatta, OCR sul
        dorso proiettato con la posa della camera dello scatto, ricerca
        titolo; aggiorna e ripubblica le detection. Il robot deve stare
        vicino allo scaffale (posa di lavoro, ~0.4 m dai dorsi); da 1 m si
        ottiene la stessa risoluzione della foto normale."""
        import math as _m
        import time as _time
        books = [o for o in self._detected_objects if o.is_book and getattr(o, "world_x", 0.0)]
        if self._camera != "head" or self.head_pose is None:
            self.get_logger().error("zoom: serve camera:=head")
            return
        if not books:
            self.get_logger().error("zoom: nessun libro con misure 3D (prima il trigger 'shelf')")
            return
        if "base_x_joint" not in self._joints:
            self.get_logger().error("zoom: nessun /joint_states")
            return
        self._publish_status("zooming")
        n_ok = 0
        for k, obj in enumerate(books, start=1):
            zc = (obj.z_bottom + obj.z_top) / 2.0
            wy, hp, ang, dist = self._aim_head_at((obj.world_x, obj.world_y, zc))
            pxcm = self.geometry.fx / max(dist, 0.05) / 100.0 if self.geometry.fx else 0.0
            self.get_logger().info(
                f"zoom {k}/{len(books)} obj {obj.obj_id}: vita yaw {wy:+.2f}, testa pitch {hp:+.2f}, "
                f"errore {_m.degrees(ang):.1f} gradi, distanza {dist:.2f} m (~{pxcm:.0f} px/cm)")
            if dist > 0.7:
                self.get_logger().warn(f"zoom obj {obj.obj_id}: a {dist:.2f} m non e' uno zoom - avvicina il robot "
                                       "(walk_to_shelf -p distance:=1.5)")
            self._send_many_and_wait([
                (self._waist_client, ["waist_yaw_joint", "waist_pitch_joint"], [wy, -0.31], 2.0, f"zoom obj {obj.obj_id}: busto"),
                (self._head_client, ["head_yaw_joint", "head_pitch_joint"], [0.0, hp], 2.0, f"zoom obj {obj.obj_id}: testa")])
            _time.sleep(1.0)
            if not self._apply_head_camera_pose():
                continue
            # scatto (con controllo del frame scuro, vedi Bugs.md)
            img = None
            for attempt in range(2):
                seq0 = self._shelf_seq
                self._pub_shelf_trigger.publish(Bool(data=True))
                t0 = _time.monotonic()
                while self._shelf_seq <= seq0 and _time.monotonic() - t0 < 90.0:
                    _time.sleep(0.2)
                if self._shelf_seq <= seq0:
                    break
                cand = self._shelf_photo_image
                lit = float((cand.max(axis=2) > 40).mean()) if cand is not None and cand.ndim == 3 else 0.0
                if lit >= 0.3:
                    img = cand.copy()
                    break
                self.get_logger().warn(f"zoom obj {obj.obj_id}: frame scuro, riscatto")
            if img is None:
                self.get_logger().error(f"zoom obj {obj.obj_id}: nessun frame")
                continue
            # dorso proiettato con la posa della camera di QUESTO scatto
            y_a, y_b = obj.world_y - obj.thickness_m / 2.0, obj.world_y + obj.thickness_m / 2.0
            corners = np.array([[obj.world_x, yy, zz] for yy in (y_a, y_b) for zz in (obj.z_bottom, obj.z_top)])
            uu, vv, _dd = self.geometry.world_to_pixels(corners)
            H, W = img.shape[:2]
            bb = (max(0, int(uu.min()) - 6), max(0, int(vv.min()) - 6),
                  min(W, int(uu.max()) + 6), min(H, int(vv.max()) + 6))
            if bb[2] - bb[0] < 10 or bb[3] - bb[1] < 10:
                self.get_logger().warn(f"zoom obj {obj.obj_id}: dorso fuori inquadratura {bb}")
                continue
            self._shot_n = getattr(self, "_shot_n", 0) + 1
            path = f"/tmp/x2_zoom_{self._shot_n}_obj{obj.obj_id}.jpg"
            cv2.imwrite(path, img[bb[1]:bb[3], bb[0]:bb[2]])
            ocr = self.ocr.read_book(img, bb)
            self.get_logger().info(f"zoom obj {obj.obj_id}: dorso {bb[2]-bb[0]}x{bb[3]-bb[1]} px -> {path}; "
                                   f"OCR '{ocr.raw_text}' -> titolo '{ocr.title}' autore '{ocr.author}'")
            if ocr.title and len(ocr.raw_text) > len(obj.ocr_text or ""):
                obj.ocr_text = ocr.raw_text
                if not obj.isbn:
                    obj.title, obj.author, obj.orientation = ocr.title, ocr.author, ocr.orientation
            meta = self._title_lookup(ocr.title, ocr.author) if ocr.title and bool(self.get_parameter("title_lookup").value) else None
            if meta and meta.get("ISBN-13"):
                obj.isbn = meta["ISBN-13"]
                obj.title = meta.get("Title") or obj.title
                if meta.get("Authors"):
                    obj.author = ", ".join(meta["Authors"])
                obj.year = str(meta.get("OriginalYear") or meta.get("Year") or "")
                n_ok += 1
                self.get_logger().info(f"zoom obj {obj.obj_id}: identificato '{obj.title}' {obj.author} {obj.year} ISBN {obj.isbn}")
            else:
                # Identificazione fatta dalla foto da 1 m (spesso il solo nome
                # della serie): la si tiene solo se coerente con il testo
                # letto da vicino, altrimenti via (meglio l'ISBN che un
                # titolo sbagliato).
                if obj.isbn and ocr.raw_text:
                    try:
                        ei = self._extract_isbn_module()
                        coherent = ei.title_matches_ocr({"Title": obj.title, "Authors": [obj.author]}, ocr.raw_text)
                    except Exception:
                        coherent = True
                    if not coherent:
                        self.get_logger().info(
                            f"zoom obj {obj.obj_id}: '{obj.title}' (dalla foto da 1 m) non coerente con il dorso "
                            f"letto da vicino: identificazione rimossa")
                        obj.isbn, obj.year = "", ""
                        obj.title, obj.author = ocr.title, ocr.author
                self.get_logger().info(f"zoom obj {obj.obj_id}: non identificato dal titolo -> ISBN dal codice a barre"
                                       + (f" (tengo '{obj.title}' ISBN {obj.isbn} dalla foto da 1 m)" if obj.isbn else ""))
        self._send_many_and_wait([
            (self._head_client, ["head_yaw_joint", "head_pitch_joint"], [0.0, 0.0], 2.0, "zoom: testa dritta"),
            (self._waist_client, ["waist_yaw_joint", "waist_pitch_joint"], [0.0, 0.0], 2.0, "zoom: busto dritto")])
        det = [o.to_dict() for o in self._detected_objects]
        self._pub_detections.publish(String(data=json.dumps(det, ensure_ascii=False)))
        with open("/tmp/x2_detections.json", "w", encoding="utf-8") as f:
            f.write(json.dumps(det, ensure_ascii=False, indent=2))
        self.get_logger().info(f"zoom: {n_ok}/{len(books)} libri identificati dal titolo; detections ripubblicate + /tmp/x2_detections.json")
        self._publish_status("awaiting_command")

    def _apply_head_camera_pose(self):
        """Posa della head_camera nel mondo dai giunti correnti -> ShelfGeometry."""
        if self.head_pose is None:
            return False
        needed = ("base_x_joint", "waist_yaw_joint", "head_pitch_joint")
        if not all(k in self._joints for k in needed):
            self.get_logger().warn("Nessun /joint_states: non so dove sta la camera della testa, "
                                   "tengo la posa precedente (misure 3D inaffidabili)")
            return False
        R_wc, t_wc = self.head_pose.world_pose(self._joints)
        self.geometry.set_camera(R_wc, t_wc, HEAD_CAM_HFOV, HEAD_CAM_SIZE)
        o = R_wc[:, 0]
        self.get_logger().info(
            f"head_camera a ({t_wc[0]:.3f}, {t_wc[1]:.3f}, {t_wc[2]:.3f}), asse ottico ({o[0]:.2f}, {o[1]:.2f}, {o[2]:.2f}) "
            f"[base_x {self._joints.get('base_x_joint', 0.0):.2f}, head_pitch {self._joints.get('head_pitch_joint', 0.0):+.2f}, "
            f"waist_pitch {self._joints.get('waist_pitch_joint', 0.0):+.2f}]")
        return True

    def _on_shelf_depth_image(self, msg: Image):
        """Depth della shelf_camera: 32FC1 (metri lungo l'asse ottico)."""
        if msg.encoding not in ("32FC1", ""):
            self.get_logger().warn(f"depth shelf_camera con encoding {msg.encoding}: attesa 32FC1")
        arr = np.frombuffer(msg.data, dtype=np.float32)
        self._shelf_depth = arr.reshape(msg.height, msg.width).copy()
        self._shelf_depth_seq += 1

    def _on_table_camera_image(self, msg: Image):
        """Cache l'ultimo frame live della camera tavolo (ri-fotografia libro)."""
        self._table_image = self._img_msg_to_bgr(msg)
        self._table_seq += 1

    def _on_depth_image(self, msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.float32)
        self._depth_image = arr.reshape(msg.height, msg.width)
        self.depth_est.set_depth_map(self._depth_image)

    # ─── Callback comando utente ──────────────────────────────────────────

    def _on_user_command(self, msg: String):
        self._pending_command = msg.data
        self.get_logger().info(f"Ricevuto comando: '{msg.data}'")

        if self._detected_objects:
            # Se abbiamo già la detection, esegui subito il riordinamento
            self._start_in_background(self._execute_sort)
        elif self._current_image is not None:
            self._start_in_background(self._run_pipeline)
        else:
            self.get_logger().warn("Nessuna immagine disponibile. "
                                   "Pubblica su /library_manager/trigger prima.")

    # ─── Pipeline principale ──────────────────────────────────────────────

    def _run_pipeline(self):
        if self._current_image is None:
            return

        self._publish_status("analyzing")
        img = self._current_image

        # depth metrica dello stesso scatto (rgbd): serve gia' al detector 'depth'
        shelf_depth = self._shelf_depth if self._shelf_depth is not None \
            and self._shelf_depth.shape[:2] == img.shape[:2] else None
        # Ogni scatto dello scaffale viene conservato con un suffisso
        # progressivo (2026-09-13: la pipeline fa due foto, da lontano e da
        # vicino, e la seconda sovrascriveva la prima): /tmp/x2_shelf_photo_N.jpg,
        # x2_shelf_depth_N.npy, x2_detections_N.jpg/.json. I file senza
        # suffisso restano l'ULTIMO scatto.
        self._shot_n = getattr(self, "_shot_n", 0) + 1
        n = self._shot_n
        cv2.imwrite(f"/tmp/x2_shelf_photo_{n}.jpg", img)
        cv2.imwrite("/tmp/x2_shelf_photo.jpg", img)
        if shelf_depth is not None:
            self.geometry.set_depth(shelf_depth)
            np.save(f"/tmp/x2_shelf_depth_{n}.npy", shelf_depth)   # per rianalisi offline
            np.save("/tmp/x2_shelf_depth.npy", shelf_depth)
        self.get_logger().info(f"Scatto #{n}: /tmp/x2_shelf_photo_{n}.jpg"
                               + (f" + x2_shelf_depth_{n}.npy" if shelf_depth is not None else ""))

        # 1. Detection oggetti
        if self.detector is None:
            self.get_logger().error(
                "Nessun detector caricato (detector:=none): il trigger richiede "
                "detector:=sam3 o yolo. La ri-foto dal tavolo (rephotograph) funziona comunque.")
            self._publish_status("error: no_detector")
            return
        self.get_logger().info(
            f"[1/5] Rilevamento oggetti ({type(self.detector).__name__})...")
        objects = self.detector.detect(img)

        # 2. Depth: se c'e' la depth METRICA della shelf_camera (rgbd) si usa
        #    quella (stessa vista della foto, niente MiDaS); altrimenti la
        #    stima monoculare di prima.
        self.get_logger().info("[2/5] Stima profondità...")
        if shelf_depth is not None:
            self.depth_est.set_depth_map(shelf_depth)
        elif self._depth_image is None:
            self.depth_est.estimate_depth_map(img)
        for obj in objects:
            obj.depth_m = self.depth_est.get_depth_at_bbox(obj.bbox)

        # 3. Analisi scaffale (righe e slot)
        self.get_logger().info("[3/5] Analisi struttura scaffale...")
        self.shelf_parser.parse(img, objects)
        # Scarta cio' che non poggia su nessun ripiano (shelf_row=-1: testa
        # del robot, tavolo, parete presi dalla passata "object" di SAM3) -
        # non sono cose da ordinare ne' da portare sul tavolo.
        off_shelf = [o for o in objects if o.shelf_row < 0]
        if off_shelf:
            self.get_logger().info(
                f"Scartati {len(off_shelf)} oggetti fuori dallo scaffale: "
                f"{[o.obj_id for o in off_shelf]}")
            objects = [o for o in objects if o.shelf_row >= 0]

        # 3b. Geometria 3D dalla depth (2026-09-13): posizione del dorso,
        #     spessore, altezza, spazio libero ai lati -> e' cio' che
        #     pick_test_book -p target:=<id> usa per la presa automatica.
        if shelf_depth is not None:
            geoms = [self.geometry.measure(o.bbox) for o in objects]
            self.geometry.free_space(geoms)
            for o, ge in zip(objects, geoms):
                if ge is None:
                    self.get_logger().warn(f"obj {o.obj_id}: troppo pochi punti di depth nella bbox")
                    continue
                o.world_x, o.world_y = ge.world_x, ge.world_y
                o.z_bottom, o.z_top = ge.z_bottom, ge.z_top
                o.thickness_m, o.height_m, o.length_m = ge.thickness, ge.height, ge.length
                o.free_plus_m, o.free_minus_m = ge.free_plus, ge.free_minus
                o.world_xyz = (ge.world_x, ge.world_y, (ge.z_bottom + ge.z_top) / 2.0)
                o.depth_m = ge.depth_m
                if o.is_book:
                    # Bbox = proiezione del DORSO misurato in 3D (2026-09-17):
                    # l'estensione dei pixel di depth include anche la
                    # striscia di copertina vista di sbieco a fianco del
                    # dorso, quindi i riquadri partivano sulla copertina del
                    # vicino e si sovrapponevano di ~10 px. Il dorso
                    # (fronte x, y +- spessore/2, z_bottom..z_top) proiettato
                    # da' il riquadro giusto, anche per l'OCR. L'originale
                    # resta in bbox_depth.
                    y_a, y_b = ge.world_y - ge.thickness / 2.0, ge.world_y + ge.thickness / 2.0
                    corners = np.array([[ge.world_x, yy, zz] for yy in (y_a, y_b) for zz in (ge.z_bottom, ge.z_top)])
                    uu, vv, _dd = self.geometry.world_to_pixels(corners)
                    H, W = img.shape[:2]
                    nb = (max(0, int(uu.min())), max(0, int(vv.min())),
                          min(W, int(uu.max()) + 1), min(H, int(vv.max()) + 1))
                    if nb[2] - nb[0] >= 4 and nb[3] - nb[1] >= 4:
                        o.bbox_depth = o.bbox
                        o.bbox = nb
                        o.center = ((nb[0] + nb[2]) // 2, (nb[1] + nb[3]) // 2)
                        o.width_px, o.height_px = nb[2] - nb[0], nb[3] - nb[1]
                self.get_logger().info(
                    f"  obj {o.obj_id} {o.class_name}: dorso x={ge.world_x:.3f} y={ge.world_y:.3f} "
                    f"z={ge.z_bottom:.3f}..{ge.z_top:.3f} spessore {ge.thickness*1000:.0f} mm "
                    f"altezza {ge.height*1000:.0f} mm, liberi +y {ge.free_plus*1000:.0f} / "
                    f"-y {ge.free_minus*1000:.0f} mm")
        else:
            self.get_logger().warn("Niente depth della shelf_camera: JSON senza world_x/thickness_m "
                                   "(la presa automatica per target richiede bookshelf.urdf rgbd)")

        # 4. Colore + OCR per ogni libro
        self.get_logger().info("[4/5] Analisi colore e OCR...")
        for obj in objects:
            if obj.is_book:
                # Colore
                cr = self.colorizer.analyze(img, obj.bbox)
                obj.color_name = cr.name
                obj.color_rgb  = cr.rgb

                # OCR + orientazione
                ocr = self.ocr.read_book(img, obj.bbox)
                obj.ocr_text    = ocr.raw_text
                obj.title       = ocr.title
                obj.author      = ocr.author
                obj.orientation = ocr.orientation
                # metadati dal titolo (Google Books), 2026-09-13
                if obj.title and bool(self.get_parameter("title_lookup").value):
                    meta = self._title_lookup(obj.title, obj.author)
                    if meta and meta.get("ISBN-13"):
                        obj.isbn = meta["ISBN-13"]
                        obj.title = meta.get("Title") or obj.title
                        if meta.get("Authors"):
                            obj.author = ", ".join(meta["Authors"])
                        obj.year = str(meta.get("OriginalYear") or meta.get("Year") or "")
                        self.get_logger().info(
                            f"  obj {obj.obj_id}: Google Books da OCR '{ocr.title}' -> "
                            f"'{obj.title}' {obj.author} {obj.year} ISBN {obj.isbn}")
                    else:
                        self.get_logger().info(
                            f"  obj {obj.obj_id}: OCR '{ocr.title}' non trovato su Google Books "
                            "-> servira' l'ISBN dal tavolo")

        self._detected_objects = objects

        # 5. Pubblica detections come JSON
        det_json = json.dumps([o.to_dict() for o in objects], ensure_ascii=False)
        self._pub_detections.publish(String(data=det_json))
        # Copia su file accanto all'immagine di debug (2026-09-06): e' il
        # JSON "cosa c'e' sul ripiano" da consultare senza ros2 topic echo.
        det_pretty = json.dumps([o.to_dict() for o in objects], ensure_ascii=False, indent=2)
        for path in ("/tmp/x2_detections.json", f"/tmp/x2_detections_{n}.json"):
            with open(path, "w", encoding="utf-8") as f:
                f.write(det_pretty)
        self.get_logger().info(f"[5/5] Rilevati: {len(objects)} oggetti")

        # Visualizzazione debug (salva su file)
        debug_img = self.detector.draw_detections(img, objects)
        cv2.imwrite("/tmp/x2_detections.jpg", debug_img)
        cv2.imwrite(f"/tmp/x2_detections_{n}.jpg", debug_img)
        self.get_logger().info(f"Debug salvato: /tmp/x2_detections.jpg + .json (copia scatto #{n}: "
                               f"/tmp/x2_detections_{n}.jpg/.json)")

        # Esegui il sort se c'è già un comando
        if self._pending_command:
            self._execute_sort()
        else:
            self._publish_status("awaiting_command")

    def _execute_sort(self):
        """Calcola il piano e avvia l'esecuzione."""
        self._publish_status("planning")

        # Parse comando utente
        sort_cmd = self.input_handler.parse(self._pending_command)
        self.get_logger().info(f"Comando: {sort_cmd}")

        if sort_cmd.criterion == SortCriterion.NONE:
            self.get_logger().warn("Criterio non riconosciuto nel comando")
            self._publish_status("error: unknown_criterion")
            return

        # Piano di riordinamento
        plan = self.sort_planner.compute_plan(self._detected_objects, sort_cmd)

        # Controllo occupazione scaffale LIVE (camera testa in Gazebo) prima
        # di reinserire i libri: la shelf_row calcolata sopra viene dalla
        # foto usata per identificazione, che può essere non aggiornata
        # (file trigger) - qui aggiorniamo insertion_order con la riga
        # REALE dove c'è posto adesso. Se non c'è ancora un frame live
        # disponibile, non tocca nulla e restano le righe della foto.
        rows = self._refresh_shelf_occupancy()
        if rows:
            self._assign_target_rows(plan.insertion_order, rows)

        # Pubblica il piano come JSON (latched su /library_manager/plan +
        # copia in /tmp/x2_sort_plan.json): righe/slot di destinazione dei
        # libri nell'ordine finale e oggetti da parcheggiare sul tavolo.
        plan_dict = {
            "criterion": sort_cmd.criterion.value,
            "ascending": sort_cmd.ascending,
            "lifo": plan.lifo_mode,
            "objects_to_table": [
                {"obj_id": o.obj_id, "class": o.class_name,
                 "color": o.color_name}
                for o in plan.obstacles_to_move],
            "removal_order": [b.obj_id for b in plan.removal_order],
            "insertion_order": [
                {"position": i + 1, "obj_id": b.obj_id, "title": b.title,
                 "author": b.author, "color": b.color_name,
                 "target_row": b.shelf_row, "target_slot": i}
                for i, b in enumerate(plan.insertion_order)],
        }
        plan_json = json.dumps(plan_dict, ensure_ascii=False, indent=2)
        self._pub_plan.publish(String(data=plan_json))
        with open("/tmp/x2_sort_plan.json", "w", encoding="utf-8") as f:
            f.write(plan_json)
        self.get_logger().info("Piano salvato: /tmp/x2_sort_plan.json "
                               "(e latched su /library_manager/plan)")

        if self._plan_only:
            self.get_logger().info(
                "plan_only=true: piano pubblicato, nessun movimento del robot.\n"
                + plan.describe())
            self._publish_status("planned")
            return

        # Genera sequenza azioni
        actions = self.action_seq.generate(plan)
        self.get_logger().info(f"Sequenza generata: {len(actions)} azioni")

        # Esegui le azioni sul robot
        self._publish_status("executing")
        self._execute_actions(actions)

        self._publish_status("done")
        self.get_logger().info("Riordinamento completato!")

    # ─── Occupazione scaffale live (camera testa) ─────────────────────────

    def _refresh_shelf_occupancy(self):
        """
        Scatta un check veloce dello stato REALE dello scaffale dall'ultimo
        frame della camera testa in Gazebo (_live_shelf_image, aggiornato da
        _on_shelf_camera_image, ultimo scatto) - solo detection + shelf_parser, niente
        colore/OCR/depth: qui serve solo sapere dove c'è posto, non cosa c'è
        scritto sui libri (quello resta compito della foto di identificazione,
        vedi _run_pipeline). Ritorna None se non è ancora arrivato nessun
        frame dalla camera testa (es. Gazebo non è in esecuzione, o il nodo
        è appena partito) - in quel caso il chiamante deve tenersi le righe
        già assegnate dalla foto di identificazione, non c'è niente di più
        fresco da usare.
        """
        if self._live_shelf_image is None:
            self.get_logger().warn(
                "Nessuno scatto della camera della libreria ancora: "
                "salto il controllo occupazione scaffale, uso le righe "
                "della foto di identificazione."
            )
            return None

        if self.detector is None:
            self.get_logger().warn("detector:=none: salto il controllo occupazione scaffale live")
            return None
        objects = self.detector.detect(self._live_shelf_image)
        rows = self.shelf_parser.parse(self._live_shelf_image, objects)
        occ = self.shelf_parser.occupancy_summary(rows)
        self.get_logger().info(
            f"Occupazione scaffale live: {occ['occupied']}/{occ['total']} "
            f"slot occupati ({occ['empty']} liberi)"
        )
        return rows

    def _assign_target_rows(self, books: list, rows: list):
        """
        Riassegna shelf_row/shelf_slot ai libri da reinserire (insertion_order
        del piano) in base agli slot VUOTI rilevati ORA sullo scaffale live,
        consumandoli uno per uno mano a mano che li assegna (due libri non
        finiscono mai sullo stesso slot). Se gli slot liberi non bastano per
        tutti i libri, quelli in eccesso mantengono la riga originale
        (calcolata dalla foto di identificazione) invece di sparire dal piano.

        Nota: sceglie solo la RIGA (mappa su reach_shelf_low/mid/high in
        action_sequencer.py, gli unici 3 target verticali che il braccio sa
        raggiungere oggi) - la posizione X dello slot vuoto non pilota il
        braccio, questo progetto non ha ancora IK continua per un punto
        preciso sullo scaffale.
        """
        empty = self.shelf_parser.find_empty_slots(rows)
        if not empty:
            self.get_logger().warn(
                "Nessuno slot libero rilevato sullo scaffale live: "
                "mantengo le righe della foto di identificazione."
            )
            return

        assigned = 0
        for book, slot in zip(books, empty):
            book.shelf_row = slot.row_id
            book.shelf_slot = slot.slot_id
            slot.occupied = True
            assigned += 1

        if len(books) > len(empty):
            self.get_logger().warn(
                f"{len(books) - assigned} libri senza slot libero rilevato: "
                "mantengono la riga originale della foto di identificazione."
            )

    # ─── Esecuzione azioni robot ──────────────────────────────────────────

    def _execute_actions(self, actions):
        """Invia ogni azione al controller corrispondente."""
        client_map = {
            "MOVE_ARM": self._arm_client,
            "GRASP":    self._gripper_client,
            "PLACE":    self._arm_client,
            "MOVE_HEAD": self._head_client,
            "ROTATE_WAIST": self._waist_client,
        }

        for i, action in enumerate(actions):
            client = client_map.get(action.action_type, self._arm_client)
            self.get_logger().info(
                f"[{i+1}/{len(actions)}] {action.description}")

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = action.joint_names

            point = JointTrajectoryPoint()
            point.positions = action.joint_positions
            point.time_from_start = Duration(
                sec=int(action.duration_sec),
                nanosec=int((action.duration_sec % 1) * 1e9)
            )
            goal.trajectory.points = [point]

            if not client.wait_for_server(timeout_sec=3.0):
                self.get_logger().warn(
                    f"Controller non disponibile per: {action.action_type}")
                continue

            # _wait_future e non spin_until_future_complete (2026-08-29):
            # questo codice gira nel thread di lavoro (vedi
            # _start_in_background) mentre l'executor spinna nel thread
            # principale - spinnare anche da qui esplodeva con "Executor is
            # already spinning" alla prima azione reale.
            future = client.send_goal_async(goal)
            goal_handle = self._wait_future(future)
            if goal_handle is not None:
                result_future = goal_handle.get_result_async()
                self._wait_future(
                    result_future, timeout_sec=action.duration_sec + 30.0)

            # Il libro è stato appena depositato sul tavolo di staging e il
            # braccio/gripper si sono tolti di mezzo (questa è la ROTATE_WAIST
            # di ritorno verso lo scaffale, vedi on_staging_table in
            # action_sequencer.py): buon momento per uno scatto ravvicinato
            # dal table_camera, senza occlusioni.
            if action.on_staging_table:
                self._rephotograph_on_table(action.target_obj_id)

    def _on_rephotograph_cmd(self, msg: String):
        """Comando manuale: ri-fotografa dal table_camera un libro portato
        sul tavolo (es. col teleop). Vedi commento sulla subscription."""
        txt = msg.data.strip()
        book = None
        if txt:
            try:
                oid = int(txt)
                book = next((o for o in self._detected_objects
                             if o.obj_id == oid), None)
            except ValueError:
                pass
        if book is None and not txt:
            book = next((o for o in self._detected_objects
                         if o.is_book and (not o.title or not o.author)),
                        None)
        if book is None:
            # Nessuna detection in memoria (nodo appena avviato) o id non
            # noto: si crea un segnaposto e si procede lo stesso (2026-09-06)
            # - la foto del tavolo + barcode/ISBN non hanno bisogno della
            # foto dello scaffale. L'id e' quello chiesto, o il prossimo libero.
            from vision.book_detector import DetectedObject
            try:
                oid = int(txt) if txt else max([o.obj_id for o in self._detected_objects], default=-1) + 1
            except ValueError:
                oid = max([o.obj_id for o in self._detected_objects], default=-1) + 1
            book = DetectedObject(obj_id=oid, class_name="book", is_book=True,
                                  bbox=(0, 0, 0, 0), confidence=1.0, center=(0, 0),
                                  width_px=0, height_px=0)
            self._detected_objects.append(book)
            self.get_logger().info(
                f"Nessuna detection per [{oid}]: creo un segnaposto e leggo il libro dal tavolo")
        self.get_logger().info(
            f"Ri-fotografia manuale dal tavolo per libro [{book.obj_id}]")
        oid = book.obj_id
        self._start_in_background(lambda: self._rephotograph_on_table(oid))

    def _rephotograph_on_table(self, obj_id: int):
        """
        Ri-scatta dal table_camera un libro appena depositato sul tavolo di
        staging e ne ricalcola SOLO i campi ancora vuoti (title/author/
        color_name) - non sovrascrive un'identificazione già riuscita sulla
        foto scaffale (più larga/lontana, dove leggere il dorso è più
        difficile). Il libro riempie gran parte dell'inquadratura ravvicinata
        del tavolo (vedi full_scene.urdf, table_camera_link): usa l'intera
        immagine come bbox invece di ri-lanciare YOLO, che su una singola
        foto ravvicinata di un solo oggetto non aggiungerebbe informazione.

        Limite onesto (vedi Sorting.md, segnale "rotate" mai azionato): NON
        ruota fisicamente il libro per esporre una faccia diversa (copertina/
        retro invece del dorso) - servirebbe un vero re-grasping con IK che
        questo progetto non ha ancora. Qui si rifotografa la stessa faccia
        già visibile, ma da vicino e senza le altre entità della scena
        attorno: un passo utile ma parziale verso quel gap.
        """
        book = next((o for o in self._detected_objects if o.obj_id == obj_id), None)
        if book is None:
            return

        img = self._capture("table")
        if img is None:
            self.get_logger().warn("Salto la ri-fotografia sul tavolo (nessun frame).")
            return
        h, w = img.shape[:2]
        bbox = (0, 0, w, h)
        cv2.imwrite("/tmp/x2_table_photo.jpg", img)
        self.get_logger().info(f"Foto tavolo salvata: /tmp/x2_table_photo.jpg ({w}x{h})")

        # ISBN dal codice a barre sul RETRO (2026-09-06): pick_test_book posa
        # il libro a faccia in giu' sotto la table_camera (1920x1440, 55 px/cm).
        # Stessa pipeline standalone di sorting/extract_isbn.py (pyzbar ->
        # OpenLibrary/Google Books): se trova l'ISBN, titolo/autore/anno
        # dai metadati SOVRASCRIVONO l'OCR del dorso, che e' molto piu'
        # inaffidabile. Senza barcode leggibile si prosegue con l'OCR.
        meta = self._isbn_lookup("/tmp/x2_table_photo.jpg")
        if meta:
            # chiavi di sorting/extract_isbn.get_book_info: ISBN-13, Title,
            # Authors (lista), Publisher, Year, Language, OriginalYear
            book.isbn = meta.get("ISBN-13") or book.isbn
            if meta.get("Title"):
                book.title = meta["Title"]
            if meta.get("Authors"):
                a = meta["Authors"]
                book.author = ", ".join(a) if isinstance(a, (list, tuple)) else str(a)
            if meta.get("OriginalYear") or meta.get("Year"):
                book.year = str(meta.get("OriginalYear") or meta.get("Year"))
            self.get_logger().info(
                f"ISBN {book.isbn}: title='{book.title}' author='{book.author}' "
                f"year='{book.year}' (libro [{obj_id}])")
            self._pub_detections.publish(String(data=json.dumps(
                [o.to_dict() for o in self._detected_objects], ensure_ascii=False)))

        if not book.title or not book.author:
            ocr = self.ocr.read_book(img, bbox)
            if not book.title and ocr.title:
                book.title = ocr.title
            if not book.author and ocr.author:
                book.author = ocr.author
            self.get_logger().info(
                f"Ri-identificazione libro [{obj_id}] dal tavolo: "
                f"title='{book.title}' author='{book.author}'"
            )

        if not book.color_name:
            cr = self.colorizer.analyze(img, bbox)
            book.color_name = cr.name
            book.color_rgb = cr.rgb

    def _extract_isbn_module(self):
        """Modulo sorting/extract_isbn.py del repo (cartella `sorting/` dalla
        cwd verso l'alto). Solleva se non trovato."""
        import importlib
        here = os.getcwd()
        for _ in range(6):
            cand = os.path.join(here, "sorting")
            if os.path.isfile(os.path.join(cand, "extract_isbn.py")):
                if cand not in sys.path:
                    sys.path.insert(0, cand)
                return importlib.import_module("extract_isbn")
            here = os.path.dirname(here)
        raise FileNotFoundError("sorting/extract_isbn.py non trovato (lancia dalla radice del repo)")

    def _title_lookup(self, title: str, author: str) -> dict | None:
        """search_book_by_title di sorting/extract_isbn.py (stesso import di _isbn_lookup)."""
        try:
            return self._extract_isbn_module().search_book_by_title(title, author)
        except Exception as e:
            self.get_logger().warn(f"title_lookup '{title}': {e}")
            return None

    def _isbn_lookup(self, image_path: str) -> dict | None:
        """Codice a barre -> ISBN -> metadati, riusando sorting/extract_isbn.py
        del repo (cartella `sorting/` alla radice, fuori dal pacchetto ROS: la
        cerchiamo dalla cwd verso l'alto, come per .env). None se non
        importabile, nessun barcode o nessun metadato."""
        import importlib
        root = os.getcwd()
        while root != os.path.dirname(root) and not os.path.isdir(os.path.join(root, "sorting")):
            root = os.path.dirname(root)
        sorting_dir = os.path.join(root, "sorting")
        if not os.path.isdir(sorting_dir):
            self.get_logger().warn("ISBN: cartella sorting/ non trovata dalla cwd - lancia il nodo dalla radice del repo")
            return None
        if sorting_dir not in sys.path:
            sys.path.insert(0, sorting_dir)
        try:
            ei = importlib.import_module("extract_isbn")
        except Exception as e:
            self.get_logger().warn(f"ISBN: extract_isbn non importabile ({e}); serve pyzbar+isbnlib nel venv")
            return None
        try:
            isbns = ei.extract_isbns_from_barcode(image_path)
        except Exception as e:
            self.get_logger().warn(f"ISBN: decodifica barcode fallita ({e})")
            return None
        if not isbns:
            self.get_logger().info("ISBN: nessun codice a barre leggibile nella foto del tavolo "
                                   "(libro non a faccia in giu', o retro fuori inquadratura)")
            return None
        self.get_logger().info(f"ISBN dal barcode: {isbns}")
        try:
            meta = ei.get_book_info(isbns[0])
        except Exception as e:
            self.get_logger().warn(f"ISBN {isbns[0]}: metadati non recuperati ({e}) - serve rete")
            meta = None
        if not meta:
            return {"ISBN-13": isbns[0]}
        meta.setdefault("ISBN-13", isbns[0])
        return meta

    def _publish_status(self, status: str):
        self._pub_status.publish(String(data=status))
        self.get_logger().info(f"Status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = LibraryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
