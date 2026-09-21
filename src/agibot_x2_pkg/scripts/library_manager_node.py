#!/usr/bin/env python3
"""
ROS 2 node (Library Manager) that photographs the bookshelf, detects and identifies the books
(detector + colour + OCR + depth + title/ISBN lookup), plans the reordering from a user command
and runs the action sequence on the X2.
Identification uses a still photo (/library_manager/trigger: file path, 'shelf', 'head' or 'zoom');
the triggered head/shelf camera (rgbd) is also used for a quick shelf occupancy check before placing.
    ros2 run x2_description library_manager_node
    ros2 topic pub /library_manager/trigger std_msgs/String "data: '/path/to/shelf.jpg'"
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

# CV and planning pipeline (local modules)
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
    """
    Node running the shelf photo -> detection -> identification -> sort plan -> execution pipeline
    """

    def __init__(self):
        """
        Declare parameters, build the CV/planning pipeline and create topics and action clients
        """
        super().__init__("library_manager")

        # detector: "sam3" (Meta SAM3 with a text prompt, best on side-by-side books;
        # needs HF_TOKEN, slow CPU inference once per photo), "depth" (geometry only,
        # no neural net), "yolo" (fast but weak on thin spines) or "none"
        self.declare_parameter("detector",       "sam3")
        self.declare_parameter("yolo_model",     "yolov8n.pt")
        # low-res book spines get low confidences: 0.25 keeps valid ones without many false positives
        self.declare_parameter("yolo_conf",      0.25)
        self.declare_parameter("depth_mode",     "midas")
        self.declare_parameter("use_llm",        False)
        self.declare_parameter("ocr_languages",  ["it", "en"])
        self.declare_parameter("default_sort",   "color")
        # True = compute and publish the plan (/library_manager/plan, /tmp/x2_sort_plan.json) without moving
        self.declare_parameter("plan_only",      False)
        # after spine OCR, look up metadata by title on Google Books; books left without ISBN go to the table
        self.declare_parameter("title_lookup",   True)
        # bookshelf pose in the world (same as the launch), used to reproject depth to world coordinates
        self.declare_parameter("shelf_x",        0.40)
        self.declare_parameter("shelf_y",        0.0)
        self.declare_parameter("shelf_yaw_deg",  90.0)
        # "head" = triggered rgbd head_camera (1920x1440, world pose from the joints at shot time),
        # "shelf" = fixed shelf_camera of bookshelf.urdf (being removed)
        self.declare_parameter("camera",         "head")
        # robot spawn pose: the launch uses x = ROBOT_SPAWN_X - walk_distance; with walk_distance:=0 pass robot_spawn_x:=-0.1
        try:
            from agibot_x2_pkg.book_placer import ROBOT_SPAWN_X, WALK_DISTANCE
            _spawn_x_default = ROBOT_SPAWN_X - WALK_DISTANCE
        except Exception:
            _spawn_x_default = -1.6
        self.declare_parameter("robot_spawn_x",  float(_spawn_x_default))
        self.declare_parameter("robot_spawn_y",  0.0)
        self.declare_parameter("robot_spawn_yaw_deg", 0.0)
        # look pose before the shot: the head camera axis is 40 deg below horizontal with the head straight;
        # head_pitch -0.38 and waist_pitch -0.31 make it horizontal, framing the whole shelf from 0.8-1.3 m
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

        # CV pipeline
        self.get_logger().info("Initialising CV pipeline...")
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
                    f"Shelf camera: head_camera (triggered rgbd on the head), robot spawn at "
                    f"x={spawn[0]:.2f} y={spawn[1]:.2f}; camera pose from the joints at every shot")
            except Exception as e:
                self.get_logger().error(
                    f"HeadCameraPose not available ({e}): agibot_x2_pkg_py must be built. "
                    "Using the shelf_camera pose (3D measurements WRONG with the head camera)")
        else:
            self.get_logger().info("Shelf camera: fixed shelf_camera (bookshelf.urdf)")
        if detector == "none":
            # for tests without the shelf photo (table photo, barcode/ISBN): saves 3-4 GB RAM vs SAM3,
            # needed on an 8 GB WSL2 VM with Gazebo where torch cannot even be imported
            self.detector = None
        elif detector == "sam3":
            from vision.sam3_detector import Sam3BookDetector
            self.detector = Sam3BookDetector()
        elif detector == "depth":
            if depth_mode == "midas":
                # no MiDaS/torch: depth comes from the camera
                self.get_logger().info("detector=depth: depth_mode midas -> rgbd (no torch)")
                depth_mode = "rgbd"
            # geometry only from the camera depth: starts in a second, titles are left to colour/OCR
            from vision.depth_detector import DepthShelfDetector
            self.detector = DepthShelfDetector(self.geometry)
        else:
            self.detector = BookDetector(model_path=yolo_model,
                                         conf_threshold=yolo_conf)
        self.colorizer = ColorAnalyzer()
        self.ocr       = OCRReader(languages=ocr_langs)
        self.depth_est = DepthEstimator(mode=depth_mode)
        self.shelf_parser = ShelfParser()

        # planning
        self.sort_planner  = SortPlanner(color_analyzer=self.colorizer)
        self.action_seq    = ActionSequencer()
        self.input_handler = InputHandler(use_llm=use_llm)

        # internal state
        self._current_image: np.ndarray | None = None
        self._depth_image:   np.ndarray | None = None
        # last shelf camera shot, kept apart from _current_image (identification photo) on purpose
        self._live_shelf_image: np.ndarray | None = None
        # high-res front photo of the shelf (trigger data 'shelf'): the right source for title OCR
        self._shelf_photo_image: np.ndarray | None = None
        # last table camera frame, used only by _rephotograph_on_table to fill missing title/author/colour
        self._table_image: np.ndarray | None = None
        # triggered cameras publish no frame until true is sent on /<camera>/trigger;
        # _capture() waits for a new frame through these sequence counters
        self._shelf_seq = 0
        self._table_seq = 0
        if self._camera == "head":
            cam_img, cam_depth, cam_trig = "/head_camera/image", "/head_camera/depth_image", "/head_camera/trigger"
        else:
            cam_img, cam_depth, cam_trig = "/shelf_camera/image", "/shelf_camera/depth_image", "/shelf_camera/trigger"
        self._cam_topics = (cam_img, cam_depth, cam_trig)
        self._pub_shelf_trigger = self.create_publisher(Bool, cam_trig, 10)
        self._pub_table_trigger = self.create_publisher(Bool, "/table_camera/trigger", 10)
        # look pose (head camera): head and waist before the shot
        self._head_client = ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
        self._waist_client = ActionClient(self, FollowJointTrajectory, "/waist_controller/follow_joint_trajectory")
        self._detected_objects = []
        self._pending_command: str = default_sort

        # subscriptions and publishers
        self.create_subscription(String, "/library_manager/trigger",
                                 self._on_image_trigger, 10)
        self.create_subscription(Image,  "/camera/image_raw",
                                 self._on_camera_image, 10)
        self.create_subscription(Image,  "/camera/depth/image_raw",
                                 self._on_depth_image, 10)
        # shelf photo: image + aligned depth of the same shot (head_camera or shelf_camera)
        self.create_subscription(Image,  cam_img,
                                 self._on_shelf_camera_image, 10)
        self.create_subscription(Image,  cam_depth,
                                 self._on_shelf_depth_image, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Image,  "/table_camera/image",
                                 self._on_table_camera_image, 10)
        self.create_subscription(String, "/library_manager/command",
                                 self._on_user_command, 10)
        # manual table re-photo for books brought by teleop: data = obj_id, or empty = first book missing title/author
        self.create_subscription(String, "/library_manager/rephotograph",
                                 self._on_rephotograph_cmd, 10)

        # latched (transient_local): published once per analysis, late subscribers still get the last message
        from rclpy.qos import QoSProfile, DurabilityPolicy
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_status     = self.create_publisher(String,
                                "/library_manager/status", latched)
        self._pub_detections = self.create_publisher(String,
                                "/library_manager/detections", latched)
        # sort plan as JSON
        self._pub_plan       = self.create_publisher(String,
                                "/library_manager/plan", latched)

        # robot action clients
        self._arm_client     = ActionClient(self, FollowJointTrajectory,
                               "/left_arm_controller/follow_joint_trajectory")
        self._gripper_client = ActionClient(self, FollowJointTrajectory,
                               "/left_gripper_controller/follow_joint_trajectory")
        self._head_client    = ActionClient(self, FollowJointTrajectory,
                               "/head_controller/follow_joint_trajectory")
        self._waist_client   = ActionClient(self, FollowJointTrajectory,
                               "/waist_controller/follow_joint_trajectory")

        # the pipeline runs in a worker thread, not in a callback: spinning from a callback fails with
        # "Executor is already spinning", and the main thread keeps processing callbacks (camera frames too)
        self._pipeline_thread: threading.Thread | None = None

        self.get_logger().info("LibraryManagerNode ready.")
        self._publish_status("idle")

    def _start_in_background(self, target):
        """
        Run a pipeline step in a worker thread, one at a time
        """
        if self._pipeline_thread is not None and self._pipeline_thread.is_alive():
            self.get_logger().warn(
                "Pipeline already running: request ignored.")
            return
        self._pipeline_thread = threading.Thread(target=target, daemon=True)
        self._pipeline_thread.start()

    def _wait_future(self, future, timeout_sec: float = 60.0):
        """
        Wait for an rclpy future from a worker thread without spinning
        The executor in the main thread completes the future; here we just wait for the event.
        """
        done_evt = threading.Event()
        future.add_done_callback(lambda _f: done_evt.set())
        if not done_evt.wait(timeout_sec):
            self.get_logger().warn("Timeout waiting for an action")
            return None
        return future.result()

    def _on_image_trigger(self, msg: String):
        """
        Start the pipeline on an image: a file path, or 'shelf' (take a shelf photo),
        'zoom' (per-book zoomed shots) or 'head' (last camera frame as it is)
        Low-res frames give partial title OCR, to be completed by the table re-photo.
        """
        path = msg.data.strip()
        if path.lower() == "shelf":
            # high-res shelf photo: the right way to a full identification (OCR included)
            def _shoot_and_run():
                """
                Take the shelf photo and run the pipeline on it
                """
                img = self._capture("shelf")
                if img is None:
                    return
                self.get_logger().info(
                    f"Analysing the shelf camera photo ({img.shape[1]}x{img.shape[0]})")
                self._current_image = img
                self._run_pipeline()
            self._start_in_background(_shoot_and_run)
            return
        if path.lower() == "zoom":
            # per-book zoom from the work pose: 40-50 px/cm on the spine, enough to read subtitles too
            self._start_in_background(self._zoom_books)
            return
        if path.lower() == "head":
            if self._live_shelf_image is None:
                self.get_logger().error(
                    "No frame from the head camera: is Gazebo running?")
                return
            self.get_logger().info("Analysing the live head camera frame")
            self._current_image = self._live_shelf_image.copy()
            self._start_in_background(self._run_pipeline)
            return
        if not os.path.exists(path):
            self.get_logger().error(f"File not found: {path}")
            return

        self.get_logger().info(f"Loading image: {path}")
        self._current_image = cv2.imread(path)
        if self._current_image is None:
            self.get_logger().error("Cannot read the image")
            return

        self._start_in_background(self._run_pipeline)

    @staticmethod
    def _img_msg_to_bgr(msg: Image) -> np.ndarray:
        """
        Convert a sensor_msgs/Image to a BGR array (OpenCV/YOLO/EasyOCR convention)
        Gazebo cameras publish rgb8: using the raw bytes as BGR swaps red and blue and breaks detection/colour.
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
        # unknown encoding: best effort
        return arr.reshape(msg.height, msg.width, -1).copy()

    def _on_camera_image(self, msg: Image):
        """
        Store the /camera/image_raw frame as the identification image
        """
        self._current_image = self._img_msg_to_bgr(msg)

    def _capture(self, camera: str, timeout_s: float = 30.0):
        """
        Trigger the 'shelf' or 'table' camera and return the NEW frame, or None after timeout_s (wall time)
        Call only from a worker thread: the frame arrives on an executor callback.
        The timeout is large because at RTF 10-20% a 1 Hz sim frame takes 5-10 real seconds.
        """
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
        self.get_logger().info(f"Shooting {camera} ({topic})...")
        t0 = _time.monotonic()
        pub.publish(Bool(data=True))
        depth_seq0 = self._shelf_depth_seq
        while _time.monotonic() - t0 < timeout_s:
            seq, img = get()
            if seq > seq0 and img is not None:
                if camera == "shelf":
                    # depth of the same shot arrives separately: wait up to 5 s, else go on without geometry
                    t1 = _time.monotonic()
                    while self._shelf_depth_seq <= depth_seq0 and _time.monotonic() - t1 < 5.0:
                        _time.sleep(0.1)
                    if self._shelf_depth_seq <= depth_seq0:
                        self.get_logger().warn(
                            f"No depth from {self._cam_topics[1]}: the camera is not rgbd "
                            "(old URDF?) - no 3D book measurements")
                self.get_logger().info(f"Frame {camera} ({img.shape[1]}x{img.shape[0]}) received in {_time.monotonic()-t0:.1f} s")
                lit = float((img.max(axis=2) > 40).mean()) if img.ndim == 3 else 1.0
                if lit < 0.3 and (_time.monotonic() - t0) < timeout_s - 10:
                    # dark frame = first render of a gz-sensors rgbd camera (colour pass not yet enabled);
                    # the launch takes a warm-up shot, this is the safety net (see Bugs.md)
                    self.get_logger().warn(f"Frame {camera} dark ({lit*100:.0f}% lit pixels): shooting again")
                    seq0 = seq
                    pub.publish(Bool(data=True))
                    depth_seq0 = self._shelf_depth_seq
                    _time.sleep(0.5)
                    continue
                out = img.copy()
                if looked:
                    self._look_restore()
                return out
            if (_time.monotonic() - t0) % 10 < 0.25:   # re-trigger every ~10 s
                pub.publish(Bool(data=True))
            _time.sleep(0.2)
        if looked:
            self._look_restore()
        self.get_logger().error(
            f"No frame from {self._cam_topics[0] if camera == 'shelf' else '/table_camera/image'} within {timeout_s:.0f} s: is the scene "
            "grasp_test, is the simulation running and does the bridge have the entry "
            f"{topic}? (triggered cameras: see bookshelf.urdf/table.urdf)")
        return None

    def _on_shelf_camera_image(self, msg: Image):
        """
        Cache the last shelf camera photo (head or fixed camera)
        """
        self._shelf_photo_image = self._img_msg_to_bgr(msg)
        self._live_shelf_image = self._shelf_photo_image
        self._shelf_seq += 1

    def _on_joint_states(self, msg: JointState):
        """
        Cache the latest joint positions by name
        """
        self._joints = dict(zip(msg.name, msg.position))

    def _send_and_wait(self, client, names, positions, duration_s, label, timeout_s=180.0):
        """
        Send a one-point FollowJointTrajectory goal from the worker thread and poll until done
        The main-thread executor completes the futures; at low RTF a 2 s sim goal can take a real minute.
        """
        import time as _time
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"{label}: action server not available, skipping")
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
            self.get_logger().warn(f"{label}: goal rejected")
            return False
        res = handle.get_result_async()
        while not res.done() and _time.monotonic() - t0 < timeout_s:
            _time.sleep(0.1)
        self.get_logger().info(f"{label}: done ({_time.monotonic()-t0:.0f} s)")
        return res.done()

    def _send_many_and_wait(self, goals, timeout_s=180.0):
        """
        Send several FollowJointTrajectory goals in parallel and wait for all
        Waist and head move together: at RTF 0.05 two sequential 2 s goals cost 60-80 real s per book.
        goals: list of (client, names, positions, duration, label).
        """
        import time as _time
        handles = []
        t0 = _time.monotonic()
        for client, names, positions, duration_s, label in goals:
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn(f"{label}: action server not available, skipping")
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
                self.get_logger().warn(f"{label}: goal rejected")
                continue
            results.append((h.get_result_async(), label))
        for res, label in results:
            while not res.done() and _time.monotonic() - t0 < timeout_s:
                _time.sleep(0.05)
        self.get_logger().info(f"{' + '.join(l for _, l in results)}: done ({_time.monotonic()-t0:.0f} s)")
        return all(r.done() for r, _ in results)

    def _look_at_shelf(self):
        """
        Move head up and waist back (look_* parameters) into the pose used to compute the camera for the shot
        """
        import time as _time
        hy, hp = float(self.get_parameter("look_head_yaw").value), float(self.get_parameter("look_head_pitch").value)
        wp = float(self.get_parameter("look_waist_pitch").value)
        wy = float(self._joints.get("waist_yaw_joint", 0.0))
        self._send_many_and_wait([
            (self._head_client, ["head_yaw_joint", "head_pitch_joint"], [hy, hp], 2.0, f"sguardo: testa ({hy:+.2f}, {hp:+.2f})"),
            (self._waist_client, ["waist_yaw_joint", "waist_pitch_joint"], [wy, wp], 2.0, f"sguardo: busto (yaw {wy:+.2f}, pitch {wp:+.2f})")])
        _time.sleep(1.0)     # settle + fresh /joint_states

    def _look_restore(self):
        """
        Bring head and waist pitch back to straight, keeping the waist yaw
        """
        wy = float(self._joints.get("waist_yaw_joint", 0.0))
        self._send_many_and_wait([
            (self._head_client, ["head_yaw_joint", "head_pitch_joint"], [0.0, 0.0], 2.0, "sguardo: testa dritta"),
            (self._waist_client, ["waist_yaw_joint", "waist_pitch_joint"], [wy, 0.0], 2.0, "sguardo: busto dritto")])

    def _aim_head_at(self, target, waist_pitch=-0.31):
        """
        Grid search on the FK for the waist yaw and head pitch that point the head camera axis at `target`
        Returns (waist_yaw, head_pitch, error_rad, distance), with waist pitched back and head yaw 0.
        """
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
        """
        For each measured book: aim the head at the spine, shoot, OCR the projected spine, look up the title
        and republish the detections
        The robot must be at the work pose (~0.4 m from the spines): from 1 m it is no better than the normal photo.
        """
        import math as _m
        import time as _time
        books = [o for o in self._detected_objects if o.is_book and getattr(o, "world_x", 0.0)]
        if self._camera != "head" or self.head_pose is None:
            self.get_logger().error("zoom: requires camera:=head")
            return
        if not books:
            self.get_logger().error("zoom: no book with 3D measurements (run the 'shelf' trigger first)")
            return
        if "base_x_joint" not in self._joints:
            self.get_logger().error("zoom: no /joint_states")
            return
        self._publish_status("zooming")
        n_ok = 0
        for k, obj in enumerate(books, start=1):
            zc = (obj.z_bottom + obj.z_top) / 2.0
            wy, hp, ang, dist = self._aim_head_at((obj.world_x, obj.world_y, zc))
            pxcm = self.geometry.fx / max(dist, 0.05) / 100.0 if self.geometry.fx else 0.0
            self.get_logger().info(
                f"zoom {k}/{len(books)} obj {obj.obj_id}: waist yaw {wy:+.2f}, head pitch {hp:+.2f}, "
                f"error {_m.degrees(ang):.1f} deg, distance {dist:.2f} m (~{pxcm:.0f} px/cm)")
            if dist > 0.7:
                self.get_logger().warn(f"zoom obj {obj.obj_id}: at {dist:.2f} m this is not a zoom - move the robot closer "
                                       "(walk_to_shelf -p distance:=1.5)")
            self._send_many_and_wait([
                (self._waist_client, ["waist_yaw_joint", "waist_pitch_joint"], [wy, -0.31], 2.0, f"zoom obj {obj.obj_id}: busto"),
                (self._head_client, ["head_yaw_joint", "head_pitch_joint"], [0.0, hp], 2.0, f"zoom obj {obj.obj_id}: testa")])
            _time.sleep(1.0)
            if not self._apply_head_camera_pose():
                continue
            # shot, with dark-frame check (see Bugs.md)
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
                self.get_logger().warn(f"zoom obj {obj.obj_id}: dark frame, shooting again")
            if img is None:
                self.get_logger().error(f"zoom obj {obj.obj_id}: no frame")
                continue
            # spine projected with the camera pose of THIS shot
            y_a, y_b = obj.world_y - obj.thickness_m / 2.0, obj.world_y + obj.thickness_m / 2.0
            corners = np.array([[obj.world_x, yy, zz] for yy in (y_a, y_b) for zz in (obj.z_bottom, obj.z_top)])
            uu, vv, _dd = self.geometry.world_to_pixels(corners)
            H, W = img.shape[:2]
            bb = (max(0, int(uu.min()) - 6), max(0, int(vv.min()) - 6),
                  min(W, int(uu.max()) + 6), min(H, int(vv.max()) + 6))
            if bb[2] - bb[0] < 10 or bb[3] - bb[1] < 10:
                self.get_logger().warn(f"zoom obj {obj.obj_id}: spine out of frame {bb}")
                continue
            self._shot_n = getattr(self, "_shot_n", 0) + 1
            path = f"/tmp/x2_zoom_{self._shot_n}_obj{obj.obj_id}.jpg"
            cv2.imwrite(path, img[bb[1]:bb[3], bb[0]:bb[2]])
            ocr = self.ocr.read_book(img, bb)
            self.get_logger().info(f"zoom obj {obj.obj_id}: spine {bb[2]-bb[0]}x{bb[3]-bb[1]} px -> {path}; "
                                   f"OCR '{ocr.raw_text}' -> title '{ocr.title}' author '{ocr.author}'")
            if ocr.title and len(ocr.raw_text) > len(obj.ocr_text or ""):
                obj.ocr_text = ocr.raw_text
                if not obj.isbn:
                    obj.title, obj.author, obj.orientation = ocr.title, ocr.author, ocr.orientation
            meta = self._title_lookup(ocr.title, ocr.author) if ocr.title and bool(self.get_parameter("title_lookup").value) else None
            if meta and meta.get("ISBN-13") and not self._spine_explained(meta.get("Title"), meta.get("Authors"), ocr.raw_text):
                self.get_logger().info(
                    f"zoom obj {obj.obj_id}: Google Books suggests '{meta.get('Title')}' but the spine says '{ocr.raw_text}': "
                    "the found title does not explain all the text (different series book or edition?) -> discarded")
                meta = None
            if meta and meta.get("ISBN-13"):
                obj.isbn = meta["ISBN-13"]
                obj.title = meta.get("Title") or obj.title
                if meta.get("Authors"):
                    obj.author = ", ".join(meta["Authors"])
                obj.year = str(meta.get("OriginalYear") or meta.get("Year") or "")
                n_ok += 1
                self.get_logger().info(f"zoom obj {obj.obj_id}: identified '{obj.title}' {obj.author} {obj.year} ISBN {obj.isbn}")
            else:
                # keep the identification from the 1 m photo (often just the series name) only if it matches
                # the close-up text: better the ISBN later than a wrong title
                if obj.isbn and ocr.raw_text:
                    try:
                        ei = self._extract_isbn_module()
                        coherent = (ei.title_matches_ocr({"Title": obj.title, "Authors": [obj.author]}, ocr.raw_text)
                                    and self._spine_explained(obj.title, [obj.author], ocr.raw_text))
                    except Exception:
                        coherent = True
                    if not coherent:
                        self.get_logger().info(
                            f"zoom obj {obj.obj_id}: '{obj.title}' (from the 1 m photo) does not match the spine "
                            f"read up close: identification removed")
                        obj.isbn, obj.year = "", ""
                        obj.title, obj.author = ocr.title, ocr.author
                self.get_logger().info(f"zoom obj {obj.obj_id}: not identified by title -> ISBN from the barcode"
                                       + (f" (keeping '{obj.title}' ISBN {obj.isbn} from the 1 m photo)" if obj.isbn else ""))
        self._send_many_and_wait([
            (self._head_client, ["head_yaw_joint", "head_pitch_joint"], [0.0, 0.0], 2.0, "zoom: testa dritta"),
            (self._waist_client, ["waist_yaw_joint", "waist_pitch_joint"], [0.0, 0.0], 2.0, "zoom: busto dritto")])
        det = [o.to_dict() for o in self._detected_objects]
        self._pub_detections.publish(String(data=json.dumps(det, ensure_ascii=False)))
        with open("/tmp/x2_detections.json", "w", encoding="utf-8") as f:
            f.write(json.dumps(det, ensure_ascii=False, indent=2))
        self.get_logger().info(f"zoom: {n_ok}/{len(books)} books identified by title; detections republished + /tmp/x2_detections.json")
        self._publish_status("awaiting_command")

    def _apply_head_camera_pose(self):
        """
        Set the head_camera world pose from the current joints into ShelfGeometry
        """
        if self.head_pose is None:
            return False
        needed = ("base_x_joint", "waist_yaw_joint", "head_pitch_joint")
        if not all(k in self._joints for k in needed):
            self.get_logger().warn("No /joint_states: head camera pose unknown, "
                                   "keeping the previous pose (3D measurements unreliable)")
            return False
        R_wc, t_wc = self.head_pose.world_pose(self._joints)
        self.geometry.set_camera(R_wc, t_wc, HEAD_CAM_HFOV, HEAD_CAM_SIZE)
        o = R_wc[:, 0]
        self.get_logger().info(
            f"head_camera at ({t_wc[0]:.3f}, {t_wc[1]:.3f}, {t_wc[2]:.3f}), optical axis ({o[0]:.2f}, {o[1]:.2f}, {o[2]:.2f}) "
            f"[base_x {self._joints.get('base_x_joint', 0.0):.2f}, head_pitch {self._joints.get('head_pitch_joint', 0.0):+.2f}, "
            f"waist_pitch {self._joints.get('waist_pitch_joint', 0.0):+.2f}]")
        return True

    def _on_shelf_depth_image(self, msg: Image):
        """
        Cache the shelf camera depth: 32FC1, metres along the optical axis
        """
        if msg.encoding not in ("32FC1", ""):
            self.get_logger().warn(f"shelf_camera depth with encoding {msg.encoding}: expected 32FC1")
        arr = np.frombuffer(msg.data, dtype=np.float32)
        self._shelf_depth = arr.reshape(msg.height, msg.width).copy()
        self._shelf_depth_seq += 1

    def _on_table_camera_image(self, msg: Image):
        """
        Cache the last table camera frame (book re-photo)
        """
        self._table_image = self._img_msg_to_bgr(msg)
        self._table_seq += 1

    def _on_depth_image(self, msg: Image):
        """
        Store the /camera/depth/image_raw map and pass it to the depth estimator
        """
        arr = np.frombuffer(msg.data, dtype=np.float32)
        self._depth_image = arr.reshape(msg.height, msg.width)
        self.depth_est.set_depth_map(self._depth_image)

    def _on_user_command(self, msg: String):
        """
        Store the user command and sort now if detections exist, else run the pipeline first
        """
        self._pending_command = msg.data
        self.get_logger().info(f"Command received: '{msg.data}'")

        if self._detected_objects:
            self._start_in_background(self._execute_sort)
        elif self._current_image is not None:
            self._start_in_background(self._run_pipeline)
        else:
            self.get_logger().warn("No image available. "
                                   "Publish on /library_manager/trigger first.")

    def _run_pipeline(self):
        """
        Analyse the current shelf photo: detection, depth, shelf rows, 3D geometry, colour, OCR and
        title lookup; publish and save the detections, then sort if a command is pending
        """
        if self._current_image is None:
            return

        self._publish_status("analyzing")
        img = self._current_image

        # metric depth of the same shot (rgbd), already needed by the 'depth' detector
        shelf_depth = self._shelf_depth if self._shelf_depth is not None \
            and self._shelf_depth.shape[:2] == img.shape[:2] else None
        # every shot is kept with a progressive suffix N (far and close photos); unsuffixed files are the LAST shot
        self._shot_n = getattr(self, "_shot_n", 0) + 1
        n = self._shot_n
        cv2.imwrite(f"/tmp/x2_shelf_photo_{n}.jpg", img)
        cv2.imwrite("/tmp/x2_shelf_photo.jpg", img)
        if shelf_depth is not None:
            self.geometry.set_depth(shelf_depth)
            np.save(f"/tmp/x2_shelf_depth_{n}.npy", shelf_depth)   # for offline re-analysis
            np.save("/tmp/x2_shelf_depth.npy", shelf_depth)
        self.get_logger().info(f"Shot #{n}: /tmp/x2_shelf_photo_{n}.jpg"
                               + (f" + x2_shelf_depth_{n}.npy" if shelf_depth is not None else ""))

        # 1. object detection
        if self.detector is None:
            self.get_logger().error(
                "No detector loaded (detector:=none): the trigger requires "
                "detector:=sam3 or yolo. The table re-photo (rephotograph) still works.")
            self._publish_status("error: no_detector")
            return
        self.get_logger().info(
            f"[1/5] Detecting objects ({type(self.detector).__name__})...")
        objects = self.detector.detect(img)

        # 2. depth: metric rgbd depth of the same view if available, else monocular estimate
        self.get_logger().info("[2/5] Estimating depth...")
        if shelf_depth is not None:
            self.depth_est.set_depth_map(shelf_depth)
        elif self._depth_image is None:
            self.depth_est.estimate_depth_map(img)
        for obj in objects:
            obj.depth_m = self.depth_est.get_depth_at_bbox(obj.bbox)

        # 3. shelf structure (rows and slots)
        self.get_logger().info("[3/5] Analysing shelf structure...")
        self.shelf_parser.parse(img, objects)
        # drop what is on no shelf (shelf_row=-1: robot head, table, wall from the SAM3 "object" pass)
        off_shelf = [o for o in objects if o.shelf_row < 0]
        if off_shelf:
            self.get_logger().info(
                f"Discarded {len(off_shelf)} objects off the shelf: "
                f"{[o.obj_id for o in off_shelf]}")
            objects = [o for o in objects if o.shelf_row >= 0]

        # 3b. 3D geometry from depth (spine position, thickness, height, side clearance),
        #     used by pick_test_book -p target:=<id> for the automatic grasp
        if shelf_depth is not None:
            geoms = [self.geometry.measure(o.bbox) for o in objects]
            self.geometry.free_space(geoms)
            for o, ge in zip(objects, geoms):
                if ge is None:
                    self.get_logger().warn(f"obj {o.obj_id}: too few depth points in the bbox")
                    continue
                o.world_x, o.world_y = ge.world_x, ge.world_y
                o.z_bottom, o.z_top = ge.z_bottom, ge.z_top
                o.thickness_m, o.height_m, o.length_m = ge.thickness, ge.height, ge.length
                o.free_plus_m, o.free_minus_m = ge.free_plus, ge.free_minus
                o.width_profile = list(getattr(ge, "width_profile", []) or [])
                o.world_xyz = (ge.world_x, ge.world_y, (ge.z_bottom + ge.z_top) / 2.0)
                o.depth_m = ge.depth_m
                if o.is_book:
                    # bbox = projection of the 3D-measured spine: the depth pixels also include the obliquely
                    # seen cover strip, so boxes overlapped the neighbour; the original stays in bbox_depth
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
                    f"  obj {o.obj_id} {o.class_name}: spine x={ge.world_x:.3f} y={ge.world_y:.3f} "
                    f"z={ge.z_bottom:.3f}..{ge.z_top:.3f} thickness {ge.thickness*1000:.0f} mm "
                    f"height {ge.height*1000:.0f} mm, free +y {ge.free_plus*1000:.0f} / "
                    f"-y {ge.free_minus*1000:.0f} mm")
        else:
            self.get_logger().warn("No shelf_camera depth: JSON without world_x/thickness_m "
                                   "(automatic grasp by target requires the rgbd bookshelf.urdf)")

        # 4. colour + OCR for every book
        self.get_logger().info("[4/5] Colour analysis and OCR...")
        for obj in objects:
            if obj.is_book:
                # colour
                cr = self.colorizer.analyze(img, obj.bbox)
                obj.color_name = cr.name
                obj.color_rgb  = cr.rgb

                # OCR + orientation
                ocr = self.ocr.read_book(img, obj.bbox)
                obj.ocr_text    = ocr.raw_text
                obj.title       = ocr.title
                obj.author      = ocr.author
                obj.orientation = ocr.orientation
                # metadata from the title (Google Books)
                if obj.title and bool(self.get_parameter("title_lookup").value):
                    meta = self._title_lookup(obj.title, obj.author)
                    if meta and meta.get("ISBN-13"):
                        obj.isbn = meta["ISBN-13"]
                        obj.title = meta.get("Title") or obj.title
                        if meta.get("Authors"):
                            obj.author = ", ".join(meta["Authors"])
                        obj.year = str(meta.get("OriginalYear") or meta.get("Year") or "")
                        self.get_logger().info(
                            f"  obj {obj.obj_id}: Google Books from OCR '{ocr.title}' -> "
                            f"'{obj.title}' {obj.author} {obj.year} ISBN {obj.isbn}")
                    else:
                        self.get_logger().info(
                            f"  obj {obj.obj_id}: OCR '{ocr.title}' not found on Google Books "
                            "-> ISBN from the table will be needed")

        self._detected_objects = objects

        # 5. publish detections as JSON
        det_json = json.dumps([o.to_dict() for o in objects], ensure_ascii=False)
        self._pub_detections.publish(String(data=det_json))
        # file copy next to the debug image, readable without ros2 topic echo
        det_pretty = json.dumps([o.to_dict() for o in objects], ensure_ascii=False, indent=2)
        for path in ("/tmp/x2_detections.json", f"/tmp/x2_detections_{n}.json"):
            with open(path, "w", encoding="utf-8") as f:
                f.write(det_pretty)
        self.get_logger().info(f"[5/5] Detected: {len(objects)} objects")

        # debug image
        debug_img = self.detector.draw_detections(img, objects)
        cv2.imwrite("/tmp/x2_detections.jpg", debug_img)
        cv2.imwrite(f"/tmp/x2_detections_{n}.jpg", debug_img)
        self.get_logger().info(f"Debug saved: /tmp/x2_detections.jpg + .json (copy of shot #{n}: "
                               f"/tmp/x2_detections_{n}.jpg/.json)")

        # sort now if a command is pending
        if self._pending_command:
            self._execute_sort()
        else:
            self._publish_status("awaiting_command")

    def _execute_sort(self):
        """
        Parse the pending command, compute and publish the sort plan, then execute it (unless plan_only)
        """
        self._publish_status("planning")

        sort_cmd = self.input_handler.parse(self._pending_command)
        self.get_logger().info(f"Command: {sort_cmd}")

        if sort_cmd.criterion == SortCriterion.NONE:
            self.get_logger().warn("Criterion not recognised in the command")
            self._publish_status("error: unknown_criterion")
            return

        plan = self.sort_planner.compute_plan(self._detected_objects, sort_cmd)

        # the identification photo may be stale: move insertion_order to rows that have room NOW
        # on the live shelf; without a live frame the photo rows are kept
        rows = self._refresh_shelf_occupancy()
        if rows:
            self._assign_target_rows(plan.insertion_order, rows)

        # plan JSON: target rows/slots in final order and objects to park on the table
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
        self.get_logger().info("Plan saved: /tmp/x2_sort_plan.json "
                               "(and latched on /library_manager/plan)")

        if self._plan_only:
            self.get_logger().info(
                "plan_only=true: plan published, no robot motion.\n"
                + plan.describe())
            self._publish_status("planned")
            return

        actions = self.action_seq.generate(plan)
        self.get_logger().info(f"Sequence generated: {len(actions)} actions")

        self._publish_status("executing")
        self._execute_actions(actions)

        self._publish_status("done")
        self.get_logger().info("Reordering completed!")

    def _refresh_shelf_occupancy(self):
        """
        Quick check of the REAL shelf state on the last camera shot: detection + shelf_parser only
        (no colour/OCR/depth, only where there is room)
        Returns the rows, or None without a frame or detector: the caller then keeps the photo rows.
        """
        if self._live_shelf_image is None:
            self.get_logger().warn(
                "No shelf camera shot yet: "
                "skipping the shelf occupancy check, using the rows "
                "of the identification photo."
            )
            return None

        if self.detector is None:
            self.get_logger().warn("detector:=none: skipping the live shelf occupancy check")
            return None
        objects = self.detector.detect(self._live_shelf_image)
        rows = self.shelf_parser.parse(self._live_shelf_image, objects)
        occ = self.shelf_parser.occupancy_summary(rows)
        self.get_logger().info(
            f"Live shelf occupancy: {occ['occupied']}/{occ['total']} "
            f"slots occupied ({occ['empty']} free)"
        )
        return rows

    def _assign_target_rows(self, books: list, rows: list):
        """
        Reassign shelf_row/shelf_slot of the books to insert to the EMPTY slots seen now, one slot per book
        Books beyond the free slots keep their photo row. Only the ROW matters (reach_shelf_low/mid/high
        in action_sequencer.py): the slot X does not drive the arm, there is no continuous IK yet.
        """
        empty = self.shelf_parser.find_empty_slots(rows)
        if not empty:
            self.get_logger().warn(
                "No free slot detected on the live shelf: "
                "keeping the rows of the identification photo."
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
                f"{len(books) - assigned} books without a detected free slot: "
                "they keep the original row of the identification photo."
            )

    def _execute_actions(self, actions):
        """
        Send every action to its controller and wait for it; re-photograph books left on the staging table
        """
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
                    f"Controller not available for: {action.action_type}")
                continue

            # _wait_future, not spin_until_future_complete: this runs in the worker thread while the
            # main executor spins ("Executor is already spinning")
            future = client.send_goal_async(goal)
            goal_handle = self._wait_future(future)
            if goal_handle is not None:
                result_future = goal_handle.get_result_async()
                self._wait_future(
                    result_future, timeout_sec=action.duration_sec + 30.0)

            # book just placed on the staging table and the arm is out of the way (return ROTATE_WAIST):
            # close-up table_camera shot without occlusions
            if action.on_staging_table:
                self._rephotograph_on_table(action.target_obj_id)

    def _on_rephotograph_cmd(self, msg: String):
        """
        Manual command: re-photograph with the table_camera a book brought to the table (e.g. by teleop)
        data = obj_id, or empty = first book missing title/author.
        """
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
            # unknown id or no detections: create a placeholder (requested id or next free one),
            # the table photo + barcode/ISBN do not need the shelf photo
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
                f"No detection for [{oid}]: creating a placeholder and reading the book from the table")
        self.get_logger().info(
            f"Manual table re-photo for book [{book.obj_id}]")
        oid = book.obj_id
        self._start_in_background(lambda: self._rephotograph_on_table(oid))

    def _rephotograph_on_table(self, obj_id: int):
        """
        Re-shoot with the table_camera a book placed on the staging table: barcode ISBN metadata
        override title/author/year, otherwise OCR fills only the empty fields
        The book fills most of the close-up frame, so the whole image is the bbox (no detector).
        The book is not flipped to show another face: that would need re-grasping with IK.
        """
        book = next((o for o in self._detected_objects if o.obj_id == obj_id), None)
        if book is None:
            return

        img = self._capture("table")
        if img is None:
            self.get_logger().warn("Skipping the table re-photo (no frame).")
            return
        h, w = img.shape[:2]
        bbox = (0, 0, w, h)
        cv2.imwrite("/tmp/x2_table_photo.jpg", img)
        self.get_logger().info(f"Table photo saved: /tmp/x2_table_photo.jpg ({w}x{h})")

        # ISBN from the barcode on the back (pick_test_book places the book face down under the
        # 1920x1440 table_camera, 55 px/cm): metadata override the much less reliable spine OCR
        meta = self._isbn_lookup("/tmp/x2_table_photo.jpg")
        if meta:
            # extract_isbn.get_book_info keys: ISBN-13, Title, Authors (list), Publisher, Year, Language, OriginalYear
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
                f"year='{book.year}' (book [{obj_id}])")
            self._pub_detections.publish(String(data=json.dumps(
                [o.to_dict() for o in self._detected_objects], ensure_ascii=False)))

        if not book.title or not book.author:
            ocr = self.ocr.read_book(img, bbox)
            if not book.title and ocr.title:
                book.title = ocr.title
            if not book.author and ocr.author:
                book.author = ocr.author
            self.get_logger().info(
                f"Re-identification of book [{obj_id}] from the table: "
                f"title='{book.title}' author='{book.author}'"
            )

        if not book.color_name:
            cr = self.colorizer.analyze(img, bbox)
            book.color_name = cr.name
            book.color_rgb = cr.rgb

    def _extract_isbn_module(self):
        """
        Import the repo's book_identification/extract_isbn.py, searching upwards from the cwd; raise if missing
        """
        import importlib
        here = os.getcwd()
        for _ in range(6):
            cand = os.path.join(here, "book_identification")
            if os.path.isfile(os.path.join(cand, "extract_isbn.py")):
                if cand not in sys.path:
                    sys.path.insert(0, cand)
                return importlib.import_module("extract_isbn")
            here = os.path.dirname(here)
        raise FileNotFoundError("book_identification/extract_isbn.py not found (launch from the repo root)")

    @staticmethod
    def _spine_explained(title, authors, raw_text, min_ratio=0.75) -> bool:
        """
        Check that the found title/author explain the spine text
        At least min_ratio of the spine words (>= 4 letters, fuzzy difflib >= 0.75) must appear in them:
        catches another book of the same series, whose title words all appear in the OCR.
        """
        import difflib
        import re

        def words(txt):
            """
            Lowercase words of at least 4 letters
            """
            return [w.lower() for w in re.findall(r"[A-Za-zÀ-ÿ]+", txt or "") if len(w) >= 4]
        known = words(title)
        for a in authors or []:
            known += words(a)
        spine = words(raw_text)
        if not spine:
            return True                     # no text to compare: title_matches_ocr decides
        ok = sum(1 for w in spine if any(difflib.SequenceMatcher(None, w, k).ratio() >= 0.75 for k in known))
        return ok / len(spine) >= min_ratio

    def _title_lookup(self, title: str, author: str) -> dict | None:
        """
        Look up book metadata by title with extract_isbn.search_book_by_title, None on error
        """
        try:
            return self._extract_isbn_module().search_book_by_title(title, author)
        except Exception as e:
            self.get_logger().warn(f"title_lookup '{title}': {e}")
            return None

    def _isbn_lookup(self, image_path: str) -> dict | None:
        """
        Barcode -> ISBN -> metadata with the repo's book_identification/extract_isbn.py
        The folder is outside the ROS package, so it is searched upwards from the cwd.
        Returns None if not importable or no barcode; only the ISBN if no metadata.
        """
        import importlib
        root = os.getcwd()
        while root != os.path.dirname(root) and not os.path.isdir(os.path.join(root, "book_identification")):
            root = os.path.dirname(root)
        bookid_dir = os.path.join(root, "book_identification")
        if not os.path.isdir(bookid_dir):
            self.get_logger().warn("ISBN: book_identification/ folder not found from the cwd - launch the node from the repo root")
            return None
        if bookid_dir not in sys.path:
            sys.path.insert(0, bookid_dir)
        try:
            ei = importlib.import_module("extract_isbn")
        except Exception as e:
            self.get_logger().warn(f"ISBN: extract_isbn cannot be imported ({e}); pyzbar+isbnlib needed in the venv")
            return None
        try:
            isbns = ei.extract_isbns_from_barcode(image_path)
        except Exception as e:
            self.get_logger().warn(f"ISBN: barcode decoding failed ({e})")
            return None
        if not isbns:
            self.get_logger().info("ISBN: no readable barcode in the table photo "
                                   "(book not face down, or back out of frame)")
            return None
        self.get_logger().info(f"ISBN from the barcode: {isbns}")
        try:
            meta = ei.get_book_info(isbns[0])
        except Exception as e:
            self.get_logger().warn(f"ISBN {isbns[0]}: metadata not retrieved ({e}) - network needed")
            meta = None
        if not meta:
            return {"ISBN-13": isbns[0]}
        meta.setdefault("ISBN-13", isbns[0])
        return meta

    def _publish_status(self, status: str):
        """
        Publish the node status (latched) and log it
        """
        self._pub_status.publish(String(data=status))
        self.get_logger().info(f"Status: {status}")


def main(args=None):
    """
    Start the node and spin until Ctrl+C
    """
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
