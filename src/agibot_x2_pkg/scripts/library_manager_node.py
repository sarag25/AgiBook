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
  SUB  /camera/image_raw          (sensor_msgs/Image)   immagine RGB
  SUB  /camera/depth/image_raw    (sensor_msgs/Image)   depth map (opzionale)
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
        self.declare_parameter("yolo_model",     "yolov8n.pt")
        self.declare_parameter("depth_mode",     "midas")
        self.declare_parameter("use_llm",        False)
        self.declare_parameter("ocr_languages",  ["it", "en"])
        self.declare_parameter("default_sort",   "color")

        yolo_model   = self.get_parameter("yolo_model").value
        depth_mode   = self.get_parameter("depth_mode").value
        use_llm      = self.get_parameter("use_llm").value
        ocr_langs    = self.get_parameter("ocr_languages").value
        default_sort = self.get_parameter("default_sort").value

        # ── Pipeline CV ─────────────────────────────────────────────────
        self.get_logger().info("Inizializzazione pipeline CV...")
        self.detector  = BookDetector(model_path=yolo_model)
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
        self._detected_objects = []
        self._pending_command: str = default_sort

        # ── Sub/Pub ─────────────────────────────────────────────────────
        self.create_subscription(String, "/library_manager/trigger",
                                 self._on_image_trigger, 10)
        self.create_subscription(Image,  "/camera/image_raw",
                                 self._on_camera_image, 10)
        self.create_subscription(Image,  "/camera/depth/image_raw",
                                 self._on_depth_image, 10)
        self.create_subscription(String, "/library_manager/command",
                                 self._on_user_command, 10)

        self._pub_status     = self.create_publisher(String,
                                "/library_manager/status", 10)
        self._pub_detections = self.create_publisher(String,
                                "/library_manager/detections", 10)

        # ── Action clients robot ─────────────────────────────────────────
        self._arm_client     = ActionClient(self, FollowJointTrajectory,
                               "/left_arm_controller/follow_joint_trajectory")
        self._gripper_client = ActionClient(self, FollowJointTrajectory,
                               "/left_gripper_controller/follow_joint_trajectory")
        self._head_client    = ActionClient(self, FollowJointTrajectory,
                               "/head_controller/follow_joint_trajectory")
        self._waist_client   = ActionClient(self, FollowJointTrajectory,
                               "/waist_controller/follow_joint_trajectory")

        self.get_logger().info("LibraryManagerNode pronto.")
        self._publish_status("idle")

    # ─── Callback immagine da file (simulazione) ──────────────────────────

    def _on_image_trigger(self, msg: String):
        """Carica un'immagine da file path per simulare la camera."""
        path = msg.data.strip()
        if not os.path.exists(path):
            self.get_logger().error(f"File non trovato: {path}")
            return

        self.get_logger().info(f"Carico immagine: {path}")
        self._current_image = cv2.imread(path)
        if self._current_image is None:
            self.get_logger().error("Impossibile leggere l'immagine")
            return

        self._run_pipeline()

    # ─── Callback camera ROS2 ─────────────────────────────────────────────

    def _on_camera_image(self, msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        self._current_image = arr.reshape(msg.height, msg.width, -1)

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
            self._execute_sort()
        elif self._current_image is not None:
            self._run_pipeline()
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
        self.get_logger().info("[1/5] Rilevamento oggetti (YOLOv8)...")
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

        # Genera sequenza azioni
        actions = self.action_seq.generate(plan)
        self.get_logger().info(f"Sequenza generata: {len(actions)} azioni")

        # Esegui le azioni sul robot
        self._publish_status("executing")
        self._execute_actions(actions)

        self._publish_status("done")
        self.get_logger().info("Riordinamento completato!")

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

            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                result_future = future.result().get_result_async()
                rclpy.spin_until_future_complete(self, result_future)

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
