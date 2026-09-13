"""
pick_test_book: presa e trasporto AUTOMATICI di un libro fisico di test,
stile demo MOGI-ROS (avvicinamento -> chiusura -> attach del DetachableJoint
-> sollevamento -> trasporto -> rilascio), con le pose del braccio calcolate
dalla cinematica inversa (arm_kinematics.py) a partire dalla posizione nota
del libro (book_placer.TEST_BOOKS) - niente angoli hardcoded.

Uso (con rviz_gaz_control.launch.py attivo e controller su):
    ros2 run agibot_x2_pkg_py pick_test_book                 # libro IT (scena grasp_test, default)
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=hunger   # hunger | it | ballata | alba | pen | globe
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p scene:=full -p book:=emma
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p dry_run:=true   # solo IK, niente movimento

Sequenza (rivista 2026-09-06: apertura di avvicinamento calcolata, IK con
vita libera in yaw per le decorazioni, rilascio via IK DENTRO il tavolo -
prima il libro cadeva sul bordo; vedi PickAndPlace.md):
  1. apre il gripper destro QUANTO BASTA per entrare ai lati dell'oggetto
     senza toccare i vicini (arm_kinematics.approach_opening: i libri sono
     a 2 cm l'uno dall'altro, a tutta apertura le dita li urtavano)
  2. IK: grasp (dorso + grasp_depth), poi pre-grasp/lift/retreat seminati
     dalla soluzione grasp (continuita' nello spazio giunti)
  3. vita+braccio a pre-grasp, braccio a grasp
  4. chiude le dita allo spessore del libro, ATTACH
  5. sfila il libro all'indietro in linea retta finche' e' TUTTO fuori dal
     fronte della libreria: braccio e vita si muovono INSIEME (2026-09-08,
     move_both) - il busto ruota verso il tavolo e si inclina mentre il
     braccio arretra, cosi' la pinza (e il libro) non ruota mai in pianta
     (con la vita ferma ruotava fino a 49 gradi e spazzava i vicini)
  6. vita a -90 gradi (verso il tavolo), busto in avanti, braccio sul punto
     di rilascio calcolato dall'IK 7 cm dentro il piano del tavolo (bordo
     vicino a y=-0.33), 3 cm sopra -> DETACH, apre -> il libro si posa
  7. torna a casa

Se l'IK del rilascio non converge (> 2 cm) si ripiega su DROP_ARM (braccio
teso sul bordo: il libro cade da 19 cm). Vedi PickAndPlace.md.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty, String
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts

from agibot_x2_pkg.book_placer import (catalog_entry, collision_size, free_space_sides,
                                       test_entities, ROBOT_SPAWN_X, SHELF_SURFACES_Z,
                                       BOOK_COLLISION_SIDE_MARGIN)
from agibot_x2_pkg_py.arm_kinematics import (
    ArmKinematics, ARM_JOINTS, WAIST_JOINTS, GRIPPER_OPEN, grasp_opening,
    approach_opening)

GRIPPER_JOINTS = ["right_gripper_left_finger_joint", "right_gripper_right_finger_joint"]
# Dal 2026-09-06 le dita sono DENTRO right_arm_controller (7 giunti,
# x2_controllers.yaml): niente piu' right_gripper_controller. Ogni
# traiettoria al braccio deve elencare tutti e 7 i giunti.
ARM7_JOINTS = ARM_JOINTS + GRIPPER_JOINTS

# Posa di trasporto (braccio raccolto davanti al petto): TCP a ~0.28 m dal
# busto, sotto il fronte dello scaffale anche a vita ruotata - verificato
# con la FK (arm_kinematics) il 2026-08-30.
CARRY_ARM = [-0.5, 0.0, 0.0, -2.2, 0.0]
# Rilascio sul tavolo: vita a -90 gradi, busto in avanti al limite. Il
# punto di rilascio viene dall'IK (2026-09-06): TABLE_RELEASE_Y dentro il
# piano (bordo vicino del tavolo a y=-0.33 dal launch), quota = piano +
# meta' altezza oggetto + TABLE_RELEASE_AIR. DROP_ARM (braccio teso
# orizzontale, TCP a y=-0.447 z=0.94 -> 19 cm di caduta) resta solo come
# ripiego se l'IK non converge.
DROP_WAIST = [-1.57, 0.31]
DROP_ARM = [-1.57, 0.0, 0.0, 0.0, 0.0]
TABLE_TOP_Z = 0.75           # table.urdf: piano a z=0.7275 +- 0.0225
# y del TCP al rilascio: -0.37 = 4 cm dentro il bordo vicino (-0.33, launch
# lx=-0.93). Scansione IK a z=0.80 (libro coricato): -0.35 -> 0.3 mm,
# -0.38 -> 7-10 mm, -0.41 -> 30 mm: piu' dentro di cosi' il braccio non
# arriva alla quota bassa. Il libro coricato si estende dal dorso (TCP +
# 2.5 cm verso il robot) al taglio (13 cm oltre): tutto sul piano.
TABLE_RELEASE_Y = -0.37
TABLE_RELEASE_X_OFFSET = -0.19   # rispetto a robot_x (spalla destra ruotata)
TABLE_RELEASE_AIR = 0.03
# Fronte della libreria (bookshelf.urdf: caso a x=0.40, interno profondo
# 0.278 -> pannelli laterali/ripiani finiscono a x=0.261). L'uscita
# rettilinea deve portare l'oggetto TUTTO oltre questo piano (con margine)
# prima di ruotare la vita: con 14 cm fissi IT (largo 15.5 cm) restava
# dentro di 2.5 cm e la rotazione lo faceva spazzare sui vicini
# (2026-09-06: "travolge gli altri libri").
SHELF_FRONT_X = 0.261
SHELF_EXIT_MARGIN = 0.03
# Quota guadagnata durante l'uscita (sopra IT ci sono 8 cm liberi, sotto 2 mm)
RETREAT_LIFT = 0.03
# Normale della COPERTINA nel world allo spawn (libri con yaw pi: copertina
# locale +Y -> world -y). Serve per posare il libro a FACCIA IN GIU' sul
# tavolo (retro con l'ISBN verso la table_camera).
BOOK_COVER_NORMAL_WORLD = np.array([0.0, -1.0, 0.0])
HOME_ARM = [0.0] * 5
HOME_WAIST = [0.0, 0.0]


class PickTestBook(Node):

    def __init__(self):
        super().__init__("pick_test_book")
        self.declare_parameter("book", "it")
        self.declare_parameter("scene", "grasp_test")   # grasp_test (default) | full, come il launch
        self.declare_parameter("robot_x", ROBOT_SPAWN_X)
        self.declare_parameter("robot_y", 0.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("grasp_depth", 0.025)   # quanto entrare oltre il dorso
        self.declare_parameter("approach_back", 0.08)  # pre-grasp: arretrato di tanto
        self.declare_parameter("retreat", 0.14)
        # Errore IK oltre il quale si ritenta con la vita libera anche in
        # yaw (necessario per portapenne e mappamondo, a x=0.40 e y>-0.1:
        # la spalla destra non arriva verso il centro del corpo).
        self.declare_parameter("yaw_retry_mm", 5.0)
        # PRESA DA PERCEZIONE (2026-09-13): target = id dell'oggetto nel JSON
        # delle detection (/tmp/x2_detections.json, scritto da
        # library_manager_node dopo il trigger 'shelf'), oppure 'auto' =
        # primo libro misurato, oppure una parola del titolo. Posizione del
        # dorso, spessore, altezza e spazio libero vengono dalla depth della
        # shelf_camera (vision/shelf_geometry.py): niente pose/misure note.
        # L'attach passa dal GraspManager (/gripper/right/attach con nome
        # vuoto = l'entita' che il dito sta toccando): il nome Gazebo
        # dell'oggetto non serve saperlo.
        # dynamic_typing: -p target:=3 arriva come INTEGER, 'auto' come STRING
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter("target", "", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("detections_file", "/tmp/x2_detections.json")
        # Chiusura: 'contact' = le dita si chiudono piano finche' il sensore
        # del dito non tocca l'oggetto (niente spessore hardcoded), 'fixed' =
        # posizione calcolata dallo spessore, 'auto' = contact se target
        # altrimenti fixed (comportamento storico dei test da catalogo).
        self.declare_parameter("close_mode", "auto")
        self.declare_parameter("close_speed", 0.005)   # m/s per dito (tempo simulato)
        # Slot di rilascio sul tavolo (2026-09-13, pipeline automatica): x/y
        # mondo del TCP al rilascio; default = punto sotto la table_camera.
        # Gli oggetti vengono parcheggiati altrove, i libri da fotografare
        # nei 2 slot sotto la camera (vedi library_pipeline.py).
        self.declare_parameter("release_x", ROBOT_SPAWN_X + TABLE_RELEASE_X_OFFSET)
        self.declare_parameter("release_y", TABLE_RELEASE_Y)

        self.book = self.get_parameter("book").value
        self.target = str(self.get_parameter("target").value).strip()
        mode = str(self.get_parameter("close_mode").value)
        self.contact_close = (mode == "contact") or (mode == "auto" and bool(self.target))
        self.close_speed = max(0.001, float(self.get_parameter("close_speed").value))
        self._contact_model = None
        self._finger_pos = None
        self.robot_x = float(self.get_parameter("robot_x").value)
        self.robot_y = float(self.get_parameter("robot_y").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.kin = ArmKinematics.from_package()

        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       "/right_arm_controller/follow_joint_trajectory")
        self.waist_client = ActionClient(self, FollowJointTrajectory,
                                         "/waist_controller/follow_joint_trajectory")
        # stato corrente per comporre i messaggi a 7 giunti
        self._arm_now = [0.0] * 5
        self._grip_now = 0.0

        scene = self.get_parameter("scene").value
        self.scene = scene
        if self.target:
            self._load_target(self.target, str(self.get_parameter("detections_file").value))
            self.attach_pub = self.create_publisher(String, "/gripper/right/attach", 10)
            self.detach_pub = self.create_publisher(Empty, "/gripper/right/detach", 10)
        else:
            entities = test_entities(scene)
            entry = next((e for e in entities if e[0].endswith(self.book) or e[1].startswith(self.book)), None)
            if entry is None:
                raise RuntimeError(f"'{self.book}' non nella scena {scene}: {[e[0] for e in entities]}")
            self.entity, self.key, self.kind, self.bx, self.by = entry
            sx, sy, sz = catalog_entry(self.kind, self.key)["size"]
            self.spine_x = self.bx - sx / 2.0
            self.thick_mesh = sy                                   # per l'apertura di avvicinamento
            self.thick_close = collision_size(self.kind, self.key)[1]   # su cui chiudere (fixed)
            self.height = sz
            self.length = sx
            self.z_center = SHELF_SURFACES_Z[3] + sz / 2.0 + 0.002
            self.free = free_space_sides(self.entity, self.scene)
            self.attach_pub = self.create_publisher(Empty, f"/{self.entity}/attach", 10)
            self.detach_pub = self.create_publisher(Empty, f"/{self.entity}/detach", 10)
        self.create_subscription(Contacts, "/contact_right_tcp", self._contact_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

    def _load_target(self, target: str, path: str):
        import json
        try:
            with open(path, encoding="utf-8") as f:
                dets = json.load(f)
        except Exception as e:
            raise RuntimeError(f"target '{target}': non leggo {path} ({e}). Prima il trigger "
                               "'shelf' di library_manager_node (detector:=depth o sam3).")
        measured = [d for d in dets if d.get("thickness_m", 0) > 0]
        if not measured:
            raise RuntimeError(f"{path}: nessun oggetto con misure 3D (thickness_m): la shelf_camera "
                               "e' rgbd e la depth e' arrivata? (log 'dorso x=...')")
        if target == "auto":
            books = [d for d in measured if d.get("is_book")]
            d = (books or measured)[0]
        elif target.isdigit():
            d = next((x for x in measured if int(x["id"]) == int(target)), None)
        else:
            d = next((x for x in measured if target.lower() in str(x.get("title", "")).lower()), None)
        if d is None:
            raise RuntimeError(f"target '{target}' non trovato fra {[(x['id'], x.get('title')) for x in measured]}")
        self.entity = f"obj{d['id']}"
        self.key = d.get("title") or d.get("class", "?")
        self.kind = "book" if d.get("is_book") else "decoration"
        self.spine_x = float(d["world_x"])
        self.by = float(d["world_y"])
        self.bx = self.spine_x          # sconosciuta la profondita' del libro: non serve
        self.thick_mesh = float(d["thickness_m"])
        # in Gazebo la collision dei libri e' piu' stretta della mesh
        # (BOOK_COLLISION_SIDE_MARGIN per lato): valore di fallback per la
        # chiusura 'fixed'; con 'contact' non serve.
        self.thick_close = self.thick_mesh - (2 * BOOK_COLLISION_SIDE_MARGIN if self.kind == "book" else 0.0)
        self.height = float(d["height_m"])
        # profondita' dorso->taglio: misurata dalla faccia superiore se
        # visibile, altrimenti 0.16 m (conservativa: uscita piu' lunga)
        self.length = float(d.get("length_m") or 0.0) or 0.16
        self.z_center = (float(d["z_bottom"]) + float(d["z_top"])) / 2.0
        self.free = (float(d.get("free_plus_m", 0.0)), float(d.get("free_minus_m", 0.0)))
        self.get_logger().info(
            f"Target dal JSON: obj {d['id']} ({self.kind}, '{d.get('title','')}' {d.get('color','')}): "
            f"dorso x={self.spine_x:.3f} y={self.by:.3f} z={self.z_center:.3f}, spessore "
            f"{self.thick_mesh*1000:.0f} mm, altezza {self.height*1000:.0f} mm, profondita' {self.length*1000:.0f} mm, liberi "
            f"+{self.free[0]*1000:.0f}/-{self.free[1]*1000:.0f} mm")

    # ─── contatti / stato dita ───────────────────────────────────────────

    def _contact_cb(self, msg):
        for c in msg.contacts:
            for ent in (c.collision1, c.collision2):
                model = ent.name.split("::")[0]
                if model and model not in ("mogi_arm", "bookshelf", "table", "ground_plane", "video_camera"):
                    self._contact_model = model
                    return

    def _js_cb(self, msg):
        if GRIPPER_JOINTS[0] in msg.name:
            self._finger_pos = msg.position[msg.name.index(GRIPPER_JOINTS[0])]

    def close_until_contact(self, p_from, label):
        """Chiude le dita a close_speed finche' il sensore del dito destro
        non tocca un oggetto della scena; poi cancella il goal (il JTC tiene
        la posizione raggiunta). Ritorna la posizione del dito, o None."""
        T = max(0.5, p_from / self.close_speed)
        self.get_logger().info(f"{label}: chiusura a contatto da {p_from*1000:.1f} mm/dito, "
                               f"{self.close_speed*1000:.0f} mm/s ({T:.0f} s sim)")
        if self.dry_run:
            self._grip_now = grasp_opening(self.thick_close)
            return self._grip_now
        self._contact_model = None
        full = list(self._arm_now) + [0.0, 0.0]
        goal = self._goal(ARM7_JOINTS, [full], T)
        fut = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: goal rifiutato")
            return None
        res = handle.get_result_async()
        t0 = time.time()
        while not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._contact_model is not None:
                handle.cancel_goal_async()
                break
            if time.time() - t0 > 20 * T + 30:      # RTF bassissimo: non restare appesi
                break
        if self._contact_model is None:
            self.get_logger().error(f"{label}: dita chiuse senza contatto - oggetto non fra le dita")
            return None
        # lascia assestare e leggi dove si e' fermato il dito
        t1 = time.time()
        while time.time() - t1 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        pos = self._finger_pos if self._finger_pos is not None else self._grip_now
        self._grip_now = float(pos)
        self.get_logger().info(f"{label}: contatto con {self._contact_model} a {pos*1000:.1f} mm/dito")
        return self._grip_now

    def _attach(self):
        if self.attach_pub.msg_type is String:
            self.attach_pub.publish(String(data=""))    # GraspManager: l'entita' toccata
        else:
            self.attach_pub.publish(Empty())

    # ─── esecuzione traiettorie ──────────────────────────────────────────

    def _move(self, client, names, positions, duration, label):
        """positions: una configurazione, oppure una LISTA di configurazioni
        (waypoint equispaziati nel tempo fino a `duration`)."""
        waypoints = positions if isinstance(positions[0], (list, tuple, np.ndarray)) else [positions]
        self.get_logger().info(
            f"{label}: {len(waypoints)} waypoint, ultimo {np.round(waypoints[-1], 3).tolist()} ({duration:.1f}s)")
        if self.dry_run:
            return True
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"{label}: action server non disponibile")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        for i, wp in enumerate(waypoints, start=1):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in wp]
            t_i = duration * i / len(waypoints)
            pt.time_from_start = Duration(sec=int(t_i), nanosec=int((t_i % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: goal rifiutato")
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return True

    def _goal(self, names, waypoints, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        for i, wp in enumerate(waypoints, start=1):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in wp]
            t_i = duration * i / len(waypoints)
            pt.time_from_start = Duration(sec=int(t_i), nanosec=int((t_i % 1) * 1e9))
            goal.trajectory.points.append(pt)
        return goal

    def move_both(self, arm_wps, waist_wps, duration, label):
        """Braccio (7 giunti) e vita (2) insieme, waypoint a coppie con gli
        stessi istanti: e' cio' che rende l'uscita dallo scaffale un moto
        rettilineo del libro (2026-09-08) - il busto ruota/si inclina mentre
        il braccio arretra, e la pinza resta allineata."""
        arm_full = [list(wp) + [self._grip_now, self._grip_now] for wp in arm_wps]
        self.get_logger().info(
            f"{label}: {len(arm_wps)} waypoint braccio+vita, vita finale "
            f"{np.round(waist_wps[-1], 2).tolist()} ({duration:.1f}s)")
        if self.dry_run:
            self._arm_now = list(arm_wps[-1])
            return True
        for c, n in ((self.arm_client, "braccio"), (self.waist_client, "vita")):
            if not c.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f"{label}: action server {n} non disponibile")
                return False
        f_arm = self.arm_client.send_goal_async(self._goal(ARM7_JOINTS, arm_full, duration))
        f_waist = self.waist_client.send_goal_async(self._goal(WAIST_JOINTS, waist_wps, duration))
        rclpy.spin_until_future_complete(self, f_arm)
        rclpy.spin_until_future_complete(self, f_waist)
        handles = [f_arm.result(), f_waist.result()]
        if any(h is None or not h.accepted for h in handles):
            self.get_logger().error(f"{label}: goal rifiutato")
            return False
        results = [h.get_result_async() for h in handles]
        for r in results:
            rclpy.spin_until_future_complete(self, r)
        self._arm_now = list(arm_wps[-1])
        return True

    def arm(self, q, duration, label):
        """q: config braccio (5) o lista di config; le dita restano
        all'apertura corrente (_grip_now) in ogni waypoint."""
        waypoints = q if isinstance(q[0], (list, tuple, np.ndarray)) else [q]
        full = [list(wp) + [self._grip_now, self._grip_now] for wp in waypoints]
        ok = self._move(self.arm_client, ARM7_JOINTS, full, duration, label)
        if ok:
            self._arm_now = list(waypoints[-1])
        return ok

    def waist(self, q, duration, label):
        return self._move(self.waist_client, WAIST_JOINTS, q, duration, label)

    def gripper(self, opening, duration, label):
        """Muove solo le dita, tenendo il braccio dov'e' (_arm_now)."""
        full = list(self._arm_now) + [opening, opening]
        ok = self._move(self.arm_client, ARM7_JOINTS, full, duration, label)
        if ok:
            self._grip_now = float(opening)
        return ok

    def _pause(self, sec):
        if not self.dry_run:
            time.sleep(sec)

    # ─── sequenza ────────────────────────────────────────────────────────

    def run(self):
        sz = self.height
        sx = self.length
        sy = self.thick_mesh
        z_center = self.z_center
        spine_x = self.spine_x
        g = np.array([spine_x + float(self.get_parameter("grasp_depth").value), self.by, z_center])
        rel = lambda p: np.asarray(p) - np.array([self.robot_x, self.robot_y, 0.0])
        back = float(self.get_parameter("approach_back").value)
        retreat = float(self.get_parameter("retreat").value)

        self.get_logger().info(
            f"Libro {self.entity} ({self.key}): centro world=({self.bx:.3f},{self.by:.3f},{z_center:.3f}) "
            f"dorso x={spine_x:.3f}, punto di presa world={np.round(g,3).tolist()}, robot a x={self.robot_x}")

        # Apertura di avvicinamento: quanto basta per entrare ai lati
        # dell'oggetto senza urtare i vicini (vedi approach_opening).
        free_plus, free_minus = self.free
        p_app, p_min, p_max = approach_opening(sy, free_plus, free_minus)
        self.get_logger().info(
            f"Spazio libero ai lati: +y {free_plus*1000:.0f} mm, -y {free_minus*1000:.0f} mm -> "
            f"apertura di avvicinamento {p_app*1000:.1f} mm/dito "
            f"(ammessa {p_min*1000:.1f}..{p_max*1000:.1f})")
        if p_max < p_min:
            self.get_logger().error(
                f"Non c'e' spazio per le dita (1 cm) accanto a {self.entity}: "
                f"servono {p_min*1000:.1f} mm/dito, ce ne stanno {p_max*1000:.1f}. "
                "Allontana i vicini in book_placer.GRASP_TEST_ENTITIES. Mi fermo.")
            return

        t0 = time.time()

        def orient_deg(q, w):
            _p, R = self.kin.fk_arm(q, tuple(w))
            return math.degrees(math.acos(min(1.0, max(-1.0, float(-R[:, 2] @ np.array([1.0, 0, 0]))))))

        def free_path(p0, p1, n, q0, w0, lift, mode):
            """Moto RETTILINEO del TCP con la vita libera (mode: 'pitch' o True =
            yaw+pitch), asse delle dita vincolato forte (w_finger 40: il libro
            non ruota in pianta ne' rolla), direzione di avvicinamento quasi
            libera (w_approach 0.05: l'avambraccio puo' inclinarsi), posizione
            stretta in y (50) e lenta in x/z (30/15) con `lift` di quota lungo
            il percorso. Misurato il 2026-09-08 su IT: con la vita fissa la
            pinza ruotava fino a 49 gradi durante l'uscita (polso al fine
            corsa) e il libro spazzava i vicini; cosi' yaw/roll restano < 1
            grado per tutti i 19 cm, il busto ruota verso il tavolo (-0.7 rad).
            Ritorna (arm_wps, waist_wps, dy_max, ang_max, x_end)."""
            qq, ww = list(q0), tuple(w0)
            arm_wps, waist_wps = [], []
            dy_max = ang_max = 0.0
            for i in range(1, n + 1):
                p = p0 + (p1 - p0) * i / n + np.array([0.0, 0.0, lift]) * i / n
                qq, ww, _e = self.kin.ik(p, q0=qq, waist=ww, optimize_waist=mode, restarts=0,
                                         w_pos=np.array([30.0, 50.0, 15.0]),
                                         w_approach=0.05, w_finger=40.0)
                pos, R = self.kin.fk_arm(qq, tuple(ww))
                fy = R[:, 1]
                yaw_b = math.degrees(math.atan2(abs(fy[0]), abs(fy[1])))
                roll_b = math.degrees(math.atan2(abs(fy[2]), abs(fy[1])))
                dy_max = max(dy_max, abs(float(pos[1] - p[1])))
                ang_max = max(ang_max, yaw_b, roll_b)
                arm_wps.append(list(qq)); waist_wps.append(list(ww))
            x_end = float(pos[0])
            return arm_wps, waist_wps, dy_max, ang_max, x_end

        def plan_paths(q_g, w_g, retreat_m):
            """pre-grasp + avvicinamento (pitch libero) + uscita (yaw+pitch
            liberi). Ritorna (q_pre, path_in, path_out, errs, angs): path_* sono
            coppie (arm_wps, waist_wps); errs in metri (errore del pre-grasp,
            errore laterale massimo dei due tratti, quanto manca alla fine
            dell'uscita); angs in gradi (rotazione massima della pinza)."""
            q_p, _w, e_p = self.kin.ik(rel(g - [back, 0, 0]), q0=q_g, waist=w_g)
            a_in, w_in, dy_in, ang_in, _x = free_path(rel(g - [back, 0, 0]), rel(g), 4, q_p, w_g, 0.0, "pitch")
            a_out, w_out, dy_out, ang_out, x_end = free_path(rel(g), rel(g + [-retreat_m, 0, 0]), 10,
                                                             a_in[-1], w_in[-1], RETREAT_LIFT, True)
            x_short = max(0.0, x_end - float(rel(g + [-retreat_m, 0, 0])[0]))
            errs = dict(pre=e_p, approach=dy_in, retreat=dy_out, uscita_incompleta=x_short)
            angs = dict(approach=ang_in, retreat=ang_out)
            return q_p, (a_in, w_in), (a_out, w_out), errs, angs

        # Uscita: almeno quanto basta perche' l'oggetto sia TUTTO fuori dal
        # fronte della libreria (poi si ruota la vita: il libro non deve
        # piu' trovarsi fra i vicini).
        retreat_needed = (spine_x + sx) - (SHELF_FRONT_X - SHELF_EXIT_MARGIN)
        if retreat_needed > retreat:
            self.get_logger().info(
                f"Uscita allungata da {retreat*100:.0f} a {retreat_needed*100:.0f} cm: "
                f"l'oggetto e' largo {sx*100:.1f} cm e deve superare il fronte (x={SHELF_FRONT_X})")
            retreat = retreat_needed

        # 1) presa a busto dritto (solo pitch libero): continuita' con i libri
        #    davanti alla spalla. 2) se la posizione o l'ORIENTAMENTO non
        #    tornano (con 5 giunti + pitch la posa 6D non e' sempre
        #    raggiungibile: Alba a 1 mm ma pinza ruotata di 19 gradi ->
        #    il libro attaccato spazza i vicini a 2 cm), candidati con lo
        #    yaw della vita libero e con yaw intermedi. Vince il candidato
        #    con avvicinamento/uscita fattibili (<= 2 cm) e orientamento
        #    minimo; se nessuno e' perfetto si prende il migliore e si avvisa.
        yaw_retry = float(self.get_parameter("yaw_retry_mm").value) / 1000.0
        q_a, w_a, e_a = self.kin.ik(rel(g), optimize_waist="pitch")
        cands = [("busto dritto", q_a, w_a, e_a)]
        o_a = orient_deg(q_a, w_a)
        if e_a > yaw_retry or o_a > 3.0:
            self.get_logger().info(
                f"IK a busto dritto: {e_a*1000:.1f} mm, pinza ruotata di {o_a:.1f} gradi -> "
                "provo con la vita libera in yaw")
            q_b, w_b, e_b = self.kin.ik(rel(g), optimize_waist=True)
            cands.append((f"yaw libero {w_b[0]:+.2f}", q_b, w_b, e_b))
            for frac in (0.5, 0.25):
                yaw_c = float(w_b[0]) * frac
                q_c, w_c, e_c = self.kin.ik(rel(g), waist=(yaw_c, float(w_a[1])),
                                            optimize_waist="pitch", restarts=3)
                cands.append((f"yaw {yaw_c:+.2f}", q_c, w_c, e_c))
        best = None
        for label, q_c, w_c, e_c in cands:
            if e_c > 0.02:
                self.get_logger().info(f"  candidato {label}: presa {e_c*1000:.1f} mm -> scartato")
                continue
            o_c = orient_deg(q_c, w_c)
            q_p, p_in, p_out, errs_c, angs_c = plan_paths(q_c, w_c, retreat)
            worst = max(errs_c.values())
            worst_ang = max(angs_c.values())
            self.get_logger().info(
                f"  candidato {label}: presa {e_c*1000:.1f} mm, orientamento {o_c:.1f} gradi, "
                f"percorsi: laterale max {worst*1000:.1f} mm, pinza ruotata max {worst_ang:.1f} gradi")
            feasible = worst <= 0.02 and worst_ang <= 6.0
            score = (0 if feasible else 1, o_c if feasible else worst)
            if best is None or score < best[0]:
                best = (score, label, q_c, w_c, e_c, o_c, q_p, p_in, p_out, errs_c)
            if feasible and o_c <= 3.0:
                break
        _score, label, q_grasp, w_grasp, e1, o_best, q_pre, path_in, path_out, errs_p = best
        e2, e3, e5 = errs_p["pre"], errs_p["approach"], max(errs_p["retreat"], errs_p["uscita_incompleta"])
        self.get_logger().info(f"Presa: {label} (orientamento {o_best:.1f} gradi)")
        if o_best > 5.0:
            self.get_logger().warn(
                f"Pinza ruotata di {o_best:.1f} gradi rispetto all'avvicinamento: con 2 cm fra i "
                "libri l'oggetto potrebbe toccare i vicini uscendo")
        # 4) rilascio DENTRO il piano del tavolo (2026-09-06, prima DROP_ARM
        #    fisso lasciava cadere il libro sul bordo): vita a -90 gradi,
        #    gripper che punta -y world (davanti al robot ruotato), dita
        #    lungo x. Verificato: 0.3 mm a z=0.85.
        # chiude sullo spessore della COLLISION (piu' stretta della mesh per i libri)
        opening = grasp_opening(self.thick_close)
        face_down = self.kind == "book"
        if face_down:
            # Libro posato di PIATTO, copertina in giu': dita verticali, il
            # dito inferiore (faccia esterna a TCP - 0.025 - opening) deve
            # restare sopra il piano -> TCP a 5 cm + opening dal piano.
            z_rel = TABLE_TOP_Z + 0.05 + opening
        else:
            z_rel = TABLE_TOP_Z + sz / 2.0 + TABLE_RELEASE_AIR
        p_rel = np.array([float(self.get_parameter("release_x").value),
                          float(self.get_parameter("release_y").value), z_rel])
        # Prima la soluzione con le dita orizzontali (libro in piedi): il
        # polso ruota attorno all'asse di avvicinamento (= asse Z del
        # gripper), quindi +-pi/2 sul polso corica il libro senza spostare
        # il TCP; il segno lo decide la normale della copertina.
        q_rel, w_rel, e_rel = self.kin.ik(rel(p_rel), approach=(0, -1, 0), finger_axis=(1, 0, 0),
                                          waist=tuple(DROP_WAIST), optimize_waist="pitch")
        if e_rel > 0.01:
            # slot fuori dalla linea della vita a -90 gradi (release_x/y della
            # pipeline): yaw libero. Gli slot verificati il 2026-09-13 arrivano
            # tutti sotto 2 mm cosi' (vedi library_pipeline.py).
            q2, w2, e2 = self.kin.ik(rel(p_rel), approach=(0, -1, 0), finger_axis=(1, 0, 0),
                                     waist=tuple(DROP_WAIST), optimize_waist=True)
            if e2 < e_rel:
                q_rel, w_rel, e_rel = q2, w2, e2
                self.get_logger().info(
                    f"Rilascio con la vita libera in yaw ({w_rel[0]:+.2f} rad): {e_rel*1000:.1f} mm")
        if face_down and e_rel <= 0.02:
            _pos, R_grasp = self.kin.fk_arm(q_grasp, tuple(w_grasp))
            n_g = R_grasp.T @ BOOK_COVER_NORMAL_WORLD      # normale copertina nel frame gripper
            best, best_z = None, 1.0
            for dw in (math.pi / 2, -math.pi / 2):
                q_alt = list(q_rel); q_alt[4] += dw
                if not (-2.556 <= q_alt[4] <= 2.556):
                    continue
                _p, R_alt = self.kin.fk_arm(q_alt, tuple(w_rel))
                nz = float((R_alt @ n_g)[2])
                if nz < best_z:
                    best, best_z = q_alt, nz
            if best is not None and best_z < -0.5:
                q_rel = best
                self.get_logger().info(
                    f"Rilascio a FACCIA IN GIU': polso ruotato a {q_rel[4]:+.2f} rad, "
                    f"normale copertina z={best_z:+.2f} (retro con l'ISBN verso la camera)")
            else:
                self.get_logger().warn(
                    f"Non riesco a coricare il libro copertina in giu' (limiti polso): "
                    f"lo poso in piedi (normale z={best_z:+.2f})")
        errs = dict(grasp=e1, pre=e2, approach=e3, retreat=e5)
        self.get_logger().info(
            f"IK in {time.time()-t0:.1f}s - errori max: " +
            ", ".join(f"{k} {v*1000:.1f}mm" for k, v in errs.items()) +
            f"; rilascio {e_rel*1000:.1f}mm a {np.round(p_rel, 3).tolist()}")
        if max(errs.values()) > 0.02:
            self.get_logger().error("IK con errore > 2 cm: libro fuori portata da questa posizione, mi fermo.")
            return
        if e_rel > 0.02:
            self.get_logger().warn(
                f"Rilascio via IK non converge ({e_rel*1000:.0f} mm): uso DROP_ARM (caduta dal bordo)")
            q_rel, w_rel = DROP_ARM, DROP_WAIST
        w_pre = w_grasp

        ok = (
            self.gripper(p_app, 1.0, f"1. apri gripper a {p_app*1000:.1f} mm/dito (avvicinamento)")
            and self.waist(list(w_grasp), 2.0, "2. vita (pitch) per la presa")
            and self.arm(q_pre, 3.0, "3. braccio pre-grasp")
            and self.move_both(path_in[0], path_in[1], 3.0, "4-5. avvicinamento rettilineo fino al grasp (braccio+vita)")
        )
        if not ok:
            return
        if self.contact_close:
            ok = self.close_until_contact(p_app, "6. chiudo le dita fino al contatto") is not None
        else:
            ok = self.gripper(opening, 1.5, f"6. chiudo le dita a {opening*1000:.1f} mm/lato")
        if not ok:
            return
        self._pause(0.5)
        self.get_logger().info(f"7. ATTACH ({self.attach_pub.topic_name})")
        if not self.dry_run:
            self._attach()
        self._pause(0.5)

        # Niente "posa di trasporto" in giunti prima della rotazione (2026-09-06):
        # era un movimento nello spazio giunti che faceva spazzare il libro
        # sui vicini. L'oggetto e' gia' tutto fuori dal fronte: si ruota la
        # vita con il braccio com'e' (il libro si allontana dallo scaffale
        # lungo un arco), poi si va al punto di rilascio sopra il tavolo.
        ok = (
            self.move_both(path_out[0], path_out[1], 6.0,
                           "8-9. sfilo l'oggetto all'indietro (rettilineo, braccio+vita, tutto fuori dallo scaffale)")
            and self.waist([DROP_WAIST[0], float(path_out[1][-1][1])], 3.0, "10-11. vita verso il tavolo (braccio fermo)")
        )
        if not ok:
            return
        # Slot con la vita poco ruotata (yaw > -1.0: slot degli oggetti a
        # x~0, o il secondo slot dei libri): tornando da -1.57 con il braccio
        # ancora teso la punta spazzerebbe di nuovo il fronte dello
        # scaffale (r~0.41 m dall'asse della vita: x > 0.25 per |yaw| < 0.64).
        # Prima si raccoglie il braccio (oggetto gia' tutto fuori dallo
        # scaffale e sopra il tavolo), poi si ruota, poi si stende sullo slot.
        if w_rel[0] > -1.0:
            ok = self.arm(CARRY_ARM, 2.5, "11b. braccio raccolto (slot con busto poco ruotato)")
            if not ok:
                return
        ok = (
            self.waist(list(w_rel), 2.0, "12. busto verso lo slot")
            and self.arm(list(q_rel), 3.0, "13. braccio sul punto di rilascio dentro il tavolo"
                         + (" (libro coricato)" if face_down else ""))
        )
        if not ok:
            return
        self.get_logger().info(f"14. DETACH ({self.detach_pub.topic_name}) + apro il gripper")
        if not self.dry_run:
            self.detach_pub.publish(Empty())
        self._pause(0.3)
        self.gripper(GRIPPER_OPEN, 1.0, "15. apri gripper")
        self._pause(1.0)
        self.arm(CARRY_ARM, 2.5, "16. braccio raccolto")
        self.waist(HOME_WAIST, 3.0, "17. vita a casa")
        self.arm(HOME_ARM, 2.5, "18. braccio a casa")
        self.get_logger().info("Sequenza completata.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickTestBook()
    try:
        ok = node.run() is True      # False/None = fermato prima del rilascio (exit 1 per library_pipeline)
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
