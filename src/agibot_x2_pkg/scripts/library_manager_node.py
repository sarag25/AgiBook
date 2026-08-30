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
                                   vedi /rgbd_head_front/image sotto per quello.
  SUB  /camera/depth/image_raw    (sensor_msgs/Image)   depth map (opzionale,
                                   stesso discorso sopra)
  SUB  /rgbd_head_front/image     (sensor_msgs/Image)   feed LIVE della camera
                                   testa in Gazebo (2026-08-14, vedi Gazebo.md
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
from std_msgs.msg import String
from sensor_msgs.msg import Image
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

        detector     = self.get_parameter("detector").value
        yolo_model   = self.get_parameter("yolo_model").value
        yolo_conf    = float(self.get_parameter("yolo_conf").value)
        depth_mode   = self.get_parameter("depth_mode").value
        use_llm      = self.get_parameter("use_llm").value
        ocr_langs    = self.get_parameter("ocr_languages").value
        default_sort = self.get_parameter("default_sort").value

        # ── Pipeline CV ─────────────────────────────────────────────────
        self.get_logger().info("Inizializzazione pipeline CV...")
        if detector == "sam3":
            from vision.sam3_detector import Sam3BookDetector
            self.detector = Sam3BookDetector()
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
        # _current_image apposta, vedi commento su /rgbd_head_front/image
        # nel docstring del modulo.
        self._live_shelf_image: np.ndarray | None = None
        # Ultimo frame della camera tavolo (2026-08-14, vedi full_scene.urdf/
        # Gazebo.md § "Camere") - usato SOLO da _rephotograph_on_table per
        # completare title/author/color_name di un libro appena depositato
        # sul tavolo di staging, se la foto di identificazione iniziale
        # (scaffale, più larga/più lontana) non li aveva letti.
        self._table_image: np.ndarray | None = None
        self._detected_objects = []
        self._pending_command: str = default_sort

        # ── Sub/Pub ─────────────────────────────────────────────────────
        self.create_subscription(String, "/library_manager/trigger",
                                 self._on_image_trigger, 10)
        self.create_subscription(Image,  "/camera/image_raw",
                                 self._on_camera_image, 10)
        self.create_subscription(Image,  "/camera/depth/image_raw",
                                 self._on_depth_image, 10)
        self.create_subscription(Image,  "/rgbd_head_front/image",
                                 self._on_head_camera_image, 10)
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

    def _on_head_camera_image(self, msg: Image):
        """Cache l'ultimo frame live della camera testa (occupazione scaffale)."""
        self._live_shelf_image = self._img_msg_to_bgr(msg)

    def _on_table_camera_image(self, msg: Image):
        """Cache l'ultimo frame live della camera tavolo (ri-fotografia libro)."""
        self._table_image = self._img_msg_to_bgr(msg)

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

        # 1. Detection oggetti
        self.get_logger().info(
            f"[1/5] Rilevamento oggetti ({type(self.detector).__name__})...")
        objects = self.detector.detect(img)

        # 2. Depth estimation
        self.get_logger().info("[2/5] Stima profondità...")
        if self._depth_image is None:
            self.depth_est.estimate_depth_map(img)
        for obj in objects:
            obj.depth_m = self.depth_est.get_depth_at_bbox(obj.bbox)

        # 3. Analisi scaffale (righe e slot)
        self.get_logger().info("[3/5] Analisi struttura scaffale...")
        self.shelf_parser.parse(img, objects)

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

        self._detected_objects = objects

        # 5. Pubblica detections come JSON
        det_json = json.dumps([o.to_dict() for o in objects], ensure_ascii=False)
        self._pub_detections.publish(String(data=det_json))
        self.get_logger().info(f"[5/5] Rilevati: {len(objects)} oggetti")

        # Visualizzazione debug (salva su file)
        debug_img = self.detector.draw_detections(img, objects)
        cv2.imwrite("/tmp/x2_detections.jpg", debug_img)
        self.get_logger().info("Debug immagine salvata: /tmp/x2_detections.jpg")

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
        _on_head_camera_image) - solo detection + shelf_parser, niente
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
                "Nessun frame da /rgbd_head_front/image ricevuto ancora: "
                "salto il controllo occupazione scaffale, uso le righe "
                "della foto di identificazione."
            )
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
        if book is None:
            book = next((o for o in self._detected_objects
                         if o.is_book and (not o.title or not o.author)),
                        None)
        if book is None:
            self.get_logger().warn(
                "Nessun libro candidato per la ri-fotografia (nessuna "
                "detection, o titoli gia' completi). Esegui prima il "
                "trigger ('head' o file).")
            return
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
        if self._table_image is None:
            self.get_logger().warn(
                "Nessun frame da /table_camera/image ricevuto ancora: "
                "salto la ri-fotografia sul tavolo."
            )
            return

        book = next((o for o in self._detected_objects if o.obj_id == obj_id), None)
        if book is None:
            return

        img = self._table_image
        h, w = img.shape[:2]
        bbox = (0, 0, w, h)

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
