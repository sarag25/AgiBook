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
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty, String, Bool
from sensor_msgs.msg import JointState, Image
from ros_gz_interfaces.msg import Contacts

from agibot_x2_pkg.book_placer import (catalog_entry, collision_size, free_space_sides,
                                       test_entities, ROBOT_SPAWN_X, SHELF_SURFACES_Z,
                                       BOOK_COLLISION_SIDE_MARGIN)
from agibot_x2_pkg_py.arm_kinematics import (
    ArmKinematics, ARM_JOINTS, WAIST_JOINTS, GRIPPER_OPEN, GRIPPER_MIN_GAP, grasp_opening,
    approach_opening, _rpy_matrix)

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
# Bordo anteriore REALE dei ripiani (bookshelf.urdf: box profondo 0.278
# centrato a y_locale=+0.011 -> fronte a 0.150 dal centro, 0.40-0.15=0.25)
# e ingombro dell'avambraccio (right_elbow_link: cilindro r=0.03). Trovato
# il 2026-09-13 con il mappamondo: presa a 6 cm dal ripiano -> gomito a
# z=0.966, avambraccio 1.5 cm SOTTO il piano del ripiano (0.993): il braccio
# si e' incastrato sotto il bordo, il TCP e' finito 11 cm piu' in basso e le
# dita si sono chiuse nel vuoto ("dita chiuse senza contatto").
PLANK_FRONT_X = 0.25
FOREARM_RADIUS = 0.03
PLANK_CLEARANCE_MIN = 0.01      # quota minima dell'avambraccio sopra il ripiano
OBJECT_GRASP_FROM_TOP = 0.025   # oggetti (non libri): presa vicino alla cima
OBJECT_EXTRA_OPENING = 0.010    # oggetti: apertura piu' larga (facce non piane)
# Normale della COPERTINA nel world allo spawn (libri con yaw pi: copertina
# locale +Y -> world -y). Serve per posare il libro a FACCIA IN GIU' sul
# tavolo (retro con l'ISBN verso la table_camera).
BOOK_COVER_NORMAL_WORLD = np.array([0.0, -1.0, 0.0])
HOME_ARM = [0.0] * 5
HOME_WAIST = [0.0, 0.0]
# ISBN dalla camera della testa (2026-09-14): head_camera in
# control_file.gazebo (rgbd 1920x1440 a scatto, hfov 1.0 dal 2026-09-16:
# e' la stessa camera che fotografa la libreria).
HEAD_JOINTS = ["head_yaw_joint", "head_pitch_joint"]
HEAD_ISBN_HFOV = 1.0                       # = <horizontal_fov> del sensore
HEAD_ISBN_ASPECT = 1440.0 / 1920.0
HEAD_SENSOR_R = _rpy_matrix(-1.5707963, -1.5707963, 0.0)   # <pose> del sensore nel link
HEAD_ISBN_TORSO_CLEAR = 0.13   # distanza minima degli spigoli del libro dall'asse del busto
HEAD_ISBN_FOV_MARGIN = 0.92    # spigoli entro il 92% del semi-angolo di vista
# Con hfov 1.0 (2026-09-16) un libro di 23 cm non entra tutto in verticale a
# 30 cm (campo 0.25 m) e piu' lontano l'IK peggiora; la rotazione del libro
# nel suo piano non e' comandabile (nessun giunto ruota attorno alla
# normale della copertina). Quindi per ogni copertina si scattano fino a 3
# foto con la testa a pitch -/+ HEAD_ISBN_PITCH_SWEEP: il libro non si
# muove, la camera inquadra il centro, poi le due meta'.
HEAD_ISBN_PITCH_SWEEP = 0.15


class PickTestBook(Node):

    def __init__(self):
        super().__init__("pick_test_book")
        self.declare_parameter("book", "it")
        self.declare_parameter("scene", "grasp_test")   # grasp_test (default) | full, come il launch
        self.declare_parameter("robot_x", ROBOT_SPAWN_X)
        self.declare_parameter("robot_y", 0.0)
        self.declare_parameter("dry_run", False)
        # ISBN dalla testa (2026-09-14): dopo l'uscita dallo scaffale il libro
        # viene portato davanti alla camera della testa con una copertina
        # rivolta alla camera, scatto + barcode; se non si legge, si gira il
        # polso di 180 gradi e si riprova con l'altra copertina.
        self.declare_parameter("head_isbn", False)
        # put_back (2026-09-17): dopo l'uscita (e l'eventuale lettura ISBN
        # dalla testa) il libro TORNA AL SUO POSTO sullo scaffale invece di
        # andare sul tavolo: stessi percorsi di uscita e avvicinamento
        # percorsi all'indietro, stacco e apertura nella posa di presa.
        self.declare_parameter("put_back", False)
        # Libri: dorso preso a questa distanza dalla CIMA invece che a meta'
        # altezza (2026-09-17): con la presa a meta' il dito copriva l'angolo
        # in basso a destra del retro, dove sta il codice a barre (IT), e la
        # foto dalla testa non lo vedeva. 0 = a meta' altezza come prima.
        # Default 0 (2026-09-17): con 4.5 cm l'IK sceglieva un altro ramo
        # (pinza ruotata di 180 gradi sull'asse di avvicinamento, X del
        # gripper verso il BASSO) e la rimessa a posto ha travolto i vicini;
        # il codice a barre si libera invece alzando la posa di mostra.
        self.declare_parameter("grasp_from_top", 0.0)
        self.declare_parameter("head_isbn_dist", 0.25)     # distanza camera-copertina di partenza (m)
        self.declare_parameter("head_yaw", -0.35)          # testa girata verso il braccio destro
        # head_pitch -0.30 (2026-09-17, era +0.20): con la testa in giu' il
        # libro stava davanti al petto e il busto nascondeva la meta' bassa
        # della copertina (dove sta il codice a barre); con -0.30 il libro
        # sta a z~1.07, a 27 cm dall'asse del busto, tutto visibile.
        self.declare_parameter("head_pitch", -0.10)
        # Libro DRITTO davanti alla camera (2026-09-17): la rotazione nel
        # piano non e' piu' libera, si chiede che l'alto del libro coincida
        # con l'alto dell'immagine, con peso HEAD_ISBN_W_UPRIGHT (compromesso
        # a 5 gradi di liberta': ~9 gradi di inclinazione e ~11 dalla
        # perpendicolare invece dei 30-40 di prima, che facevano fallire
        # pyzbar). 0 = rotazione libera come prima.
        self.declare_parameter("head_isbn_w_upright", 2.0)
        self.declare_parameter("head_isbn_wait_s", 120.0)  # attesa del frame (RTF basso)
        # Il lato +Y della pinza e' il retro del libro (copertine con normale
        # -y allo spawn): si fotografa solo quello, max 3 scatti (centro,
        # basso, cima). True = se non legge, gira il libro e riprova.
        self.declare_parameter("head_isbn_both_sides", True)
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
        # 'auto' = a contatto SEMPRE (2026-09-17, era solo con target): la
        # chiusura fissa sulla larghezza della collision non genera contatto
        # se il libro non e' perfettamente centrato (Hunger Games: dita a
        # 16 mm = box 62 mm, nessun contatto, presa abortita). A contatto le
        # dita si chiudono finche' il sensore tocca, qualunque sia la posizione.
        self.contact_close = (mode == "contact") or (mode == "auto")
        self.close_speed = max(0.001, float(self.get_parameter("close_speed").value))
        self._contact_model = None
        self._finger_pos = None
        self.robot_x = float(self.get_parameter("robot_x").value)
        self.robot_y = float(self.get_parameter("robot_y").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.kin = ArmKinematics.from_package()
        self.head_isbn = bool(self.get_parameter("head_isbn").value)
        self.put_back = bool(self.get_parameter("put_back").value)
        self._head_image = None
        if self.head_isbn:
            self.head_kin = ArmKinematics.from_package(tip="rgbd_head_front_link")
            self.head_client = ActionClient(self, FollowJointTrajectory,
                                            "/head_controller/follow_joint_trajectory")
            self.head_trigger_pub = self.create_publisher(Bool, "/head_camera/trigger", 10)
            self.create_subscription(Image, "/head_camera/image", self._head_image_cb, 1)

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
        # entrambe le dita (2026-09-17): la chiusura si ferma al PRIMO contatto,
        # cosi' il dito senza sensore non spinge il libro contro il vicino
        self.create_subscription(Contacts, "/contact_right_tcp_b", self._contact_cb, 10)
        # Auto-attach del GraspManager SPENTO mentre questo nodo lavora
        # (2026-09-17: durante la chiusura su Alba il dito con il sensore ha
        # toccato la Ballata vicina, agganciata e portata via insieme). Gli
        # attach/detach qui sono espliciti. Riacceso alla fine (main).
        self.auto_attach_pub = self.create_publisher(Bool, "/gripper/auto_attach", 10)
        self.detach_all_pub = self.create_publisher(Empty, "/gripper/right/detach", 10)
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
        self._js = dict(zip(msg.name, msg.position))

    def _wait_converged(self, names, targets, label, tol=0.012, timeout_s=120.0):
        """Aspetta che i giunti reali (/joint_states) siano entro `tol` rad
        dal comando (2026-09-17: la JTC dichiara "goal reached" allo scadere
        del tempo della traiettoria anche se i giunti sono ancora indietro
        di decine di gradi - misurato 17 gradi sulla spalla e 8 sulla vita
        al pre-grasp di Hunger Games; l'avvicinamento partiva con il braccio
        fuori posto e le dita finivano contro la libreria). Con RTF 0.05 un
        secondo di assestamento sono 20 s reali: timeout largo."""
        if self.dry_run:
            return True
        t0 = time.time()
        worst, worst_name = None, ""
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            js = getattr(self, "_js", {})
            errs = [(abs(float(js[n]) - float(t)), n) for n, t in zip(names, targets) if n in js]
            if len(errs) == len(names):
                worst, worst_name = max(errs)
                if worst <= tol:
                    return True
        self.get_logger().warn(f"{label}: giunti non assestati dopo {timeout_s:.0f} s "
                               f"({worst_name} a {math.degrees(worst or 0):.1f} gradi dal comando)")
        return False

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
        self._wait_converged(names, waypoints[-1], label)
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
        self._wait_converged(ARM7_JOINTS, arm_full[-1], label)
        self._wait_converged(WAIST_JOINTS, waist_wps[-1], label)
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

    # ─── ISBN dalla camera della testa (2026-09-14) ──────────────────────

    def _head_image_cb(self, msg):
        self._head_image = msg

    def head(self, q, duration, label):
        return self._move(self.head_client, HEAD_JOINTS, q, duration, label)

    def head_camera(self, waist, head_q):
        """Posizione e assi (ottico, alto, destra immagine) della camera della
        testa nel frame del robot, per vita e testa date."""
        q = {WAIST_JOINTS[0]: float(waist[0]), WAIST_JOINTS[1]: float(waist[1]),
             HEAD_JOINTS[0]: float(head_q[0]), HEAD_JOINTS[1]: float(head_q[1])}
        p, R = self.head_kin.fk_link(q)
        Rc = R @ HEAD_SENSOR_R
        return p, Rc[:, 0], Rc[:, 2], -Rc[:, 1]

    def plan_show(self, waist, head_q, q_seed=None):
        """Posa del braccio che porta la copertina del libro davanti alla
        camera della testa: normale della copertina (+-Y del gripper) verso la
        camera, rotazione del libro nel suo piano libera (5 giunti: la posa 6D
        non e' raggiungibile, e per il barcode l'orientamento non conta).
        Cerca la distanza (da head_isbn_dist in su) alla quale l'IK converge,
        il libro sta tutto nell'inquadratura e i suoi spigoli restano lontani
        dal busto. Ritorna (q_A, q_B, dist) - q_B mostra l'altra copertina -
        oppure None."""
        p, o, up, right = self.head_camera(waist, head_q)
        n = -o
        sx, sz = self.length, self.height
        half_h = HEAD_ISBN_HFOV / 2.0 * HEAD_ISBN_FOV_MARGIN
        half_v = math.atan(HEAD_ISBN_ASPECT * math.tan(HEAD_ISBN_HFOV / 2.0)) * HEAD_ISBN_FOV_MARGIN + HEAD_ISBN_PITCH_SWEEP
        d0 = float(self.get_parameter("head_isbn_dist").value)
        book_up_g = getattr(self, "_book_up_g", np.array([1.0, 0.0, 0.0]))
        book_dz = getattr(self, "_book_dz", 0.0)
        gd = float(self.get_parameter("grasp_depth").value)
        w_upr = float(self.get_parameter("head_isbn_w_upright").value)
        for d in [d0 + 0.025 * k for k in range(5)]:
            c = p + d * o
            # alto del libro = alto dell'immagine: X gripper = up_cam (o il suo
            # opposto se in questo ramo la X punta in basso), Y = n, quindi
            # Z = X x Y e l'avvicinamento (-Z) = n x X
            x_des = up * (1.0 if book_up_g[0] >= 0 else -1.0)
            a_des = np.cross(n, x_des); a_des /= np.linalg.norm(a_des)
            a = a_des.copy()
            up_b = np.array([0.0, 0.0, 1.0]); up_b -= (up_b @ o) * o; up_b /= np.linalg.norm(up_b)
            # Seme = posa corrente del braccio (2026-09-17): la soluzione piu'
            # vicina a dove sta gia' il braccio, cosi' dal libro appena
            # uscito si va DRITTI davanti alla camera (e' il polso che ruota
            # il libro con il retro verso la camera), senza raccogliere il
            # braccio ne' incrociarlo davanti al petto.
            q0 = list(q_seed) if q_seed is not None else list(CARRY_ARM)
            q = None
            for it in range(2):          # il TCP dipende dalla rotazione trovata: 2 passate
                tcp = c - a * (sx / 2.0 - gd) - up_b * book_dz
                q, wq, e = self.kin.ik(tcp, approach=a_des, finger_axis=n, waist=tuple(waist),
                                       optimize_waist=False, w_finger=20.0, w_approach=w_upr,
                                       finger_signed=True, q0=q0, restarts=6 if it == 0 else 0)
                _pos, R = self.kin.fk_arm(q, tuple(wq))
                a = -R[:, 2]
                up_b = R @ book_up_g
                q0 = q
            pos, R = self.kin.fk_arm(q, tuple(wq))
            ang_n = math.degrees(math.acos(max(-1.0, min(1.0, float(R[:, 1] @ n)))))
            up_b = R @ book_up_g
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up_b @ up)))))
            cc = pos + a * (sx / 2.0 - gd) + up_b * book_dz
            hax = up_b
            in_fov, torso, zmin = True, 9.0, 9.0
            for s1 in (-1, 1):
                for s2 in (-1, 1):
                    k = cc + (sx / 2.0) * a * s1 + (sz / 2.0) * hax * s2
                    v = k - p; v /= np.linalg.norm(v)
                    if abs(math.atan2(v @ right, v @ o)) > half_h or abs(math.atan2(v @ up, v @ o)) > half_v:
                        in_fov = False
                    torso = min(torso, math.hypot(k[0], k[1]))
                    zmin = min(zmin, float(k[2]))
            ok = e < 0.012 and ang_n < 15.0 and in_fov and torso > HEAD_ISBN_TORSO_CLEAR and zmin > TABLE_TOP_Z + 0.05
            self.get_logger().info(
                f"  mostra alla testa d={d:.3f}: IK {e*1000:.1f} mm, copertina->camera {ang_n:.0f} gradi, "
                f"libro inclinato {tilt:.0f} gradi, in inquadratura {in_fov}, spigoli a {torso:.2f} m dal busto, z min {zmin:.2f}"
                + (" -> OK" if ok else ""))
            if not ok:
                continue
            # altra copertina: polso girato di 180 gradi (stessa posa del TCP)
            lo, hi = self.kin.limits[ARM_JOINTS[4]]
            q_b = None
            for dw in (math.pi, -math.pi):
                if lo <= q[4] + dw <= hi:
                    q_b = list(q); q_b[4] = q[4] + dw
                    break
            if q_b is None:
                q_b, _w, e_b = self.kin.ik(tcp, approach=a, finger_axis=-n, waist=tuple(waist),
                                           optimize_waist=False, w_finger=20.0, w_approach=0.0,
                                           finger_signed=True, q0=q, restarts=6)
                if e_b > 0.01:
                    q_b = None
            return list(q), q_b, d
        return None

    def read_isbn_head(self, label):
        """Scatto con head_camera e decodifica del codice a barre
        (sorting/extract_isbn.py, come library_manager_node._isbn_lookup).
        Ritorna il dict dei metadati (almeno ISBN-13) o None."""
        if self.dry_run:
            self.get_logger().info(f"{label}: dry_run, nessuno scatto")
            return None
        import cv2
        wait = float(self.get_parameter("head_isbn_wait_s").value)
        img = None
        for attempt in range(2):
            self._head_image = None
            self.head_trigger_pub.publish(Bool(data=True))
            t0 = time.time()
            while self._head_image is None and time.time() - t0 < wait:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self._head_image is None:
                self.get_logger().error(f"{label}: nessun frame da /head_camera/image in {wait:.0f} s "
                                        "(bridge image_bridge e sensore head_camera nel launch?)")
                return None
            msg = self._head_image
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == "rgb8":
                img = img[:, :, ::-1]
            lit = float((img.max(axis=2) > 40).mean())
            if lit >= 0.3:
                break
            # Primo render di un sensore rgbd = senza colore (gz-sensors,
            # vedi Bugs.md 2026-09-17): il launch scalda la camera all'avvio,
            # qui la rete di sicurezza: riscatto una volta.
            self.get_logger().warn(f"{label}: frame scuro ({lit*100:.0f}% pixel illuminati), riscatto")
        self._head_shot = getattr(self, "_head_shot", 0) + 1
        path = f"/tmp/x2_head_isbn_{self.entity}_{self._head_shot}.jpg"
        cv2.imwrite(path, img)
        self.get_logger().info(f"{label}: foto {msg.width}x{msg.height} salvata in {path}")
        # sorting/extract_isbn.py: dalla cwd verso l'alto (come library_manager_node)
        import importlib, sys, os
        root = os.getcwd()
        while root != os.path.dirname(root) and not os.path.isdir(os.path.join(root, "sorting")):
            root = os.path.dirname(root)
        sdir = os.path.join(root, "sorting")
        if not os.path.isdir(sdir):
            self.get_logger().warn(f"{label}: cartella sorting/ non trovata dalla cwd (lancia dalla radice del repo)")
            return None
        if sdir not in sys.path:
            sys.path.insert(0, sdir)
        # `ros2 run` esegue lo script installato con /usr/bin/python3 (shebang
        # scritto da colcon), quindi il venv attivo NON conta: pyzbar e
        # isbnlib si prendono dai site-packages del .venv del repo, aggiunti
        # al path (2026-09-17). Con il venv attivo o meno, il risultato e' lo
        # stesso; il messaggio e' informativo, non un errore.
        import glob
        venv_sp = sorted(glob.glob(os.path.join(root, ".venv", "lib", "python3*", "site-packages")))
        try:
            importlib.import_module("pyzbar")
        except Exception:
            if venv_sp and venv_sp[-1] not in sys.path:
                sys.path.append(venv_sp[-1])
                self.get_logger().info(f"{label}: pyzbar dal .venv del repo ({venv_sp[-1]})")
        try:
            ei = importlib.import_module("extract_isbn")
        except Exception as e:
            self.get_logger().error(f"{label}: extract_isbn non importabile ({e}): nel .venv servono pyzbar e isbnlib")
            return None
        isbns = []
        try:
            isbns = ei.extract_isbns_from_barcode(path)
        except Exception as e:
            self.get_logger().warn(f"{label}: decodifica barcode fallita ({e})")
        if not isbns:
            try:
                isbns = ei.extract_isbns_from_ocr(path)
                if isbns:
                    self.get_logger().info(f"{label}: nessun barcode, ISBN dall'OCR: {isbns}")
            except Exception as e:
                self.get_logger().info(f"{label}: OCR non disponibile ({e})")
        if not isbns:
            self.get_logger().info(f"{label}: nessun ISBN leggibile in {path}")
            return None
        self.get_logger().info(f"{label}: ISBN {isbns}")
        meta = None
        try:
            meta = ei.get_book_info(isbns[0])
        except Exception as e:
            self.get_logger().warn(f"{label}: metadati non recuperati ({e}) - serve rete")
        meta = dict(meta or {})
        meta.setdefault("ISBN-13", isbns[0])
        meta["image"] = path
        return meta

    def show_to_head_and_read(self, waist):
        """Sequenza completa: braccio raccolto -> posa di mostra (copertina A)
        -> testa -> scatto -> se serve copertina B -> scatto. Scrive
        /tmp/x2_head_isbn_<entita'>.json. La testa torna dritta; il braccio
        resta nella posa di mostra (il chiamante passa da CARRY_ARM)."""
        head_q = [float(self.get_parameter("head_yaw").value), float(self.get_parameter("head_pitch").value)]
        self.get_logger().info("H0. ISBN dalla testa: cerco la posa di mostra")
        plan = self.plan_show(waist, head_q, q_seed=list(self._arm_now))
        if plan is None:
            self.get_logger().warn("H0. nessuna posa di mostra valida: salto la lettura dalla testa")
            return None
        q_a, q_b, d = plan
        meta = None
        lo_h, hi_h = -0.3838, 0.3838     # head_pitch_joint (x2_hand_gazebo.urdf)

        def aim_head(target):
            """(head_yaw, head_pitch) che puntano l'asse ottico della camera
            sul punto `target` (frame robot), vita ferma a `waist`."""
            best = None
            for hy in np.linspace(-0.36, 0.36, 25):
                for hp in np.linspace(lo_h, hi_h, 39):
                    p, o, _u, _r = self.head_camera(waist, [hy, hp])
                    v = np.asarray(target) - p
                    ang = math.acos(max(-1.0, min(1.0, float(o @ v) / float(np.linalg.norm(v)))))
                    if best is None or ang < best[0]:
                        best = (ang, float(hy), float(hp))
            return best[1], best[2], best[0]

        def sweep(label, q_side):
            """Fino a 3 scatti sulla stessa copertina: centro, poi parte bassa
            (dove di solito sta il codice a barre, angolo vicino al dorso), poi
            cima - puntando la camera dalla posa reale del libro in mano. Ci si
            ferma al primo scatto che legge un ISBN."""
            pos, R = self.kin.fk_arm(q_side, tuple(waist))
            a_dir = -R[:, 2]
            up = R @ getattr(self, "_book_up_g", np.array([1.0, 0.0, 0.0]))
            c = (pos + a_dir * (self.length / 2.0 - float(self.get_parameter("grasp_depth").value))
                 + up * getattr(self, "_book_dz", 0.0))
            # Prima UNA foto al centro (tutta la copertina: nella prova del
            # 2026-09-17 e' quella che ha letto il codice), poi solo se serve
            # la parte bassa e la cima; l'altra copertina solo dopo.
            targets = (("centro", c),
                       ("basso", c - up * (self.height / 2.0 - 0.035)),
                       ("cima", c + up * (self.height / 2.0 - 0.035)))
            for k, (zone, tgt) in enumerate(targets):
                hy, hp, err = aim_head(tgt)
                if not self.head([hy, hp], 1.0, f"{label}: testa su '{zone}' (yaw {hy:+.2f}, pitch {hp:+.2f}, {math.degrees(err):.1f} gradi)"):
                    continue
                self._pause(0.5)
                m = self.read_isbn_head(f"{label}: scatto {k+1} ({zone})")
                if m is not None:
                    return m
            return None

        both = bool(self.get_parameter("head_isbn_both_sides").value)
        ok = (self.head(head_q, 1.5, f"H2. testa verso il libro {np.round(head_q, 2).tolist()}")
              and self.arm(q_a, 4.0, f"H3. retro del libro davanti alla camera (d={d:.2f} m)"))
        if ok:
            self._pause(1.0)
            meta = sweep("H4. retro", q_a)
            if meta is None and both and q_b is not None:
                if self.head(head_q, 1.0, "H5. testa al centro") and self.arm(q_b, 2.5, "H5. giro il libro: altra copertina"):
                    self._pause(1.0)
                    meta = sweep("H6. altra copertina", q_b)
        self.head([0.0, 0.0], 1.5, "H7. testa dritta")
        out = {"entity": self.entity, "isbn": (meta or {}).get("ISBN-13"), "title": (meta or {}).get("Title"),
               "author": ", ".join((meta or {}).get("Authors", []) or []) if isinstance((meta or {}).get("Authors"), list) else (meta or {}).get("Authors"),
               "year": (meta or {}).get("Year"), "image": (meta or {}).get("image"), "dist": d}
        import json
        path = f"/tmp/x2_head_isbn_{self.entity}.json"
        if not self.dry_run:
            with open(path, "w") as f:
                json.dump(out, f, indent=2, ensure_ascii=False)
            # metadati anche nel JSON delle detection (2026-09-17): in
            # modalita' target l'entita' e' "obj<id>", in modalita' catalogo
            # si cerca il libro per posizione (world_y del dorso).
            if meta:
                try:
                    dp = "/tmp/x2_detections.json"
                    dets = json.load(open(dp)) if os.path.isfile(dp) else []
                    hit = None
                    if self.entity.startswith("obj"):
                        hit = next((d for d in dets if str(d.get("id")) == self.entity[3:]), None)
                    else:
                        hit = next((d for d in dets if d.get("is_book") and abs(float(d.get("world_y", 9)) - self.by) < 0.02), None)
                    if hit is not None:
                        hit.update({"isbn": out["isbn"], "title": out["title"] or hit.get("title", ""),
                                    "author": out["author"] or hit.get("author", ""), "year": out["year"] or hit.get("year", ""),
                                    "isbn_image": out["image"], "how": "isbn_head"})
                        with open(dp, "w", encoding="utf-8") as f:
                            json.dump(dets, f, indent=2, ensure_ascii=False)
                        self.get_logger().info(f"Metadati salvati anche in {dp} (obj {hit.get('id')})")
                except Exception as e:
                    self.get_logger().warn(f"detections JSON non aggiornato: {e}")
        if meta:
            self.get_logger().info(f"ISBN dalla testa: {out['isbn']} '{out['title']}' {out['author']} {out['year']} -> {path}")
        else:
            self.get_logger().warn(f"ISBN dalla testa: non letto (risultato in {path})")
        return meta

    def put_back_on_shelf(self, q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app, shown):
        """Rimette l'oggetto dov'era (2026-09-17): braccio raccolto, busto
        alla vita di fine uscita, braccio alla posa di fine uscita, percorso
        di uscita ALL'INDIETRO fino alla presa (il libro rientra fra i vicini
        lungo la stessa retta da cui e' uscito), DETACH, dita all'apertura di
        avvicinamento, percorso di avvicinamento all'indietro fino al
        pre-grasp, casa. Le mosse in giunti (raccolto <-> fine uscita) avvengono
        con il libro TUTTO fuori dal fronte dello scaffale, come all'andata."""
        a_out, w_out = path_out
        a_in, w_in = path_in
        w_end = [float(w_out[-1][0]), float(w_out[-1][1])]
        # R1 SEMPRE (2026-09-17): dalla posa di mostra direttamente alla posa
        # di fine uscita, l'interpolazione nei giunti faceva spazzare il
        # libro sui vicini (Hunger Games: gomito bloccato 19 gradi indietro,
        # tre libri a terra). Raccolto prima, poi busto, poi fine uscita.
        ok = self.arm(CARRY_ARM, 2.5, "R1. braccio raccolto (libro in mano)")
        # R3 in due tempi (2026-09-17): dal braccio raccolto DIRETTAMENTE alla
        # posa di fine uscita l'interpolazione nei giunti fa "sporgere" la
        # mano oltre la posa finale a meta' corsa, e la punta del libro (20 cm
        # oltre le dita) entrava nello scaffale spazzando i vicini (Hunger
        # Games: IT e Ballata a terra). Ora: R3a posa SICURA = fine uscita
        # arretrata di 12 cm lungo l'asse del libro e alzata di 5 cm (mossa
        # in giunti, lontano dallo scaffale), R3b avvicinamento RETTILINEO
        # da li' alla posa di fine uscita (IK a vita ferma, come l'uscita).
        q_end = list(a_out[-1])
        p_end, R_end = self.kin.fk_arm(q_end, tuple(w_end))
        a_end = -R_end[:, 2]
        p_safe = p_end - a_end * 0.12 + np.array([0.0, 0.0, 0.05])
        q_safe, _w, e_safe = self.kin.ik(p_safe, approach=a_end, finger_axis=R_end[:, 1], waist=tuple(w_end),
                                         optimize_waist=False, q0=q_end, restarts=0, w_approach=2.0, w_finger=20.0)
        if e_safe > 0.02:
            q_safe, _w, e_safe = self.kin.ik(p_safe, approach=a_end, finger_axis=R_end[:, 1], waist=tuple(w_end),
                                             optimize_waist=False, q0=q_end, restarts=6, w_approach=2.0, w_finger=20.0)
        line = []
        qq = list(q_safe)
        for i in range(1, 5):
            pt = p_safe + (p_end - p_safe) * i / 4.0
            qq, _w, _e = self.kin.ik(pt, approach=a_end, finger_axis=R_end[:, 1], waist=tuple(w_end),
                                     optimize_waist=False, q0=qq, restarts=0, w_approach=2.0, w_finger=20.0)
            line.append(list(qq))
        line[-1] = q_end            # chiude esattamente sulla posa di fine uscita
        self.get_logger().info(f"R3. posa sicura a {e_safe*1000:.1f} mm, poi 4 waypoint rettilinei fino alla fine uscita")
        # R4/R7 ripianificati (2026-09-17): linea retta dalla fine uscita
        # alla presa, TRASLATA dell'offset del libro nella pinza, cosi' il
        # libro (non il TCP) torna nel suo slot; vita libera come all'uscita.
        off = np.asarray(getattr(self, "_book_off", np.zeros(3)), dtype=float)
        p_g, R_g = self.kin.fk_arm(q_grasp, tuple(w_grasp))
        a_g = -R_g[:, 2]
        p_pre_tcp, _R = self.kin.fk_arm(q_pre, tuple(w_pre))
        def straight(p0, p1, n, q0, w0, mode):
            qq, ww, arm_w, waist_w = list(q0), tuple(w0), [], []
            for i in range(1, n + 1):
                pt = p0 + (p1 - p0) * i / n
                qq, ww, _e = self.kin.ik(pt, approach=a_g, finger_axis=R_g[:, 1], q0=qq, waist=ww,
                                         optimize_waist=mode, restarts=0, w_pos=np.array([30.0, 50.0, 15.0]),
                                         w_approach=0.05, w_finger=40.0)
                arm_w.append(list(qq)); waist_w.append(list(ww))
            return arm_w, waist_w
        p_end_off = p_end + off
        p_g_off = p_g + off
        r4_arm, r4_w = straight(p_end_off, p_g_off, 10, q_end, w_end, True)
        r7_arm, r7_w = straight(p_g_off, p_pre_tcp + off, 4, r4_arm[-1], r4_w[-1], "pitch")
        if np.linalg.norm(off) > 0.002:
            self.get_logger().info(f"R4/R7 traslati di {np.round(off*1000, 1).tolist()} mm (libro fuori centro nella pinza)")
        ok = (ok
              and self.waist(w_end, 3.0, f"R2. busto alla vita di fine uscita {np.round(w_end, 2).tolist()}")
              and self.arm(list(q_safe), 3.0, "R3a. braccio alla posa sicura (12 cm piu' indietro, 5 cm piu' su)")
              and self.arm(line, 4.0, "R3b. avvicinamento rettilineo alla posa di fine uscita")
              and self.move_both(r4_arm, r4_w, 6.0,
                                 "R4. rientro rettilineo fino alla posa di presa (con l'offset del libro)"))
        if not ok:
            return False
        self._pause(0.5)
        self.set_auto_attach(False)
        self.get_logger().info(f"R5. DETACH ({self.detach_pub.topic_name} + /gripper/right/detach)")
        if not self.dry_run:
            self.detach_pub.publish(Empty())
            self.detach_all_pub.publish(Empty())    # anche cio' che il manager avesse agganciato da solo
        self._pause(0.5)
        ok = (self.gripper(p_app, 1.0, f"R6. apro le dita a {p_app*1000:.1f} mm/dito (avvicinamento)")
              and self.move_both(r7_arm, r7_w, 3.0,
                                 "R7. arretro fino al pre-grasp (linea retta, con l'offset)")
              and self.arm(CARRY_ARM, 2.5, "R8. braccio raccolto")
              and self.waist(HOME_WAIST, 3.0, "R9. vita a casa")
              and self.arm(HOME_ARM, 2.5, "R10. braccio a casa"))
        if ok:
            self.get_logger().info("Oggetto rimesso al suo posto. Sequenza completata.")
        return ok

    def fingers_open_check(self, p_app, label, tol=0.002):
        """Entrambe le dita entro tol dall'apertura di avvicinamento; se no
        rimanda il comando fino a 3 volte e poi si ferma (meglio non entrare
        fra i libri con un dito chiuso)."""
        if self.dry_run:
            return True
        for attempt in range(3):
            js = getattr(self, "_js", {})
            pos = [float(js.get(j, -1.0)) for j in GRIPPER_JOINTS]
            if all(abs(p - p_app) <= tol for p in pos):
                self.get_logger().info(f"{label}: dita a {[round(p*1000, 1) for p in pos]} mm")
                return True
            self.get_logger().warn(f"{label}: dita a {[round(p*1000, 1) for p in pos]} mm, attese {p_app*1000:.1f}: rimando il comando")
            self.gripper(p_app, 1.0, f"{label}: riapro")
        js = getattr(self, "_js", {})
        pos = [float(js.get(j, -1.0)) for j in GRIPPER_JOINTS]
        if all(abs(p - p_app) <= tol for p in pos):
            return True
        self.get_logger().error(f"{label}: un dito non si apre ({[round(p*1000, 1) for p in pos]} mm): mi fermo senza toccare lo scaffale")
        self.arm(HOME_ARM, 2.5, "braccio a casa")
        return False

    def _pause(self, sec):
        if not self.dry_run:
            time.sleep(sec)

    # ─── sequenza ────────────────────────────────────────────────────────

    def set_auto_attach(self, on: bool):
        if self.dry_run:
            return
        # Aspetta che il GraspManager sia sottoscritto (2026-09-17: i
        # messaggi pubblicati prima del match DDS andavano persi e l'OFF
        # arrivava... all'inizio della presa successiva), poi invia piu' volte.
        t0 = time.time()
        while self.auto_attach_pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.auto_attach_pub.get_subscription_count() == 0:
            self.get_logger().warn("GraspManager non in ascolto su /gripper/auto_attach (grasp_manager:=false?)")
            return
        for _ in range(3):
            self.auto_attach_pub.publish(Bool(data=bool(on)))
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"GraspManager auto_attach {'ON' if on else 'OFF'}")

    def run(self):
        self.set_auto_attach(False)
        sz = self.height
        sx = self.length
        sy = self.thick_mesh
        z_center = self.z_center
        z_top = self.z_center + sz / 2.0
        gft = float(self.get_parameter("grasp_from_top").value)
        if self.kind == "book" and gft > 0.0 and z_top - gft > z_center:
            z_center = z_top - gft
            self.get_logger().info(f"Libro: presa del dorso a {gft*100:.1f} cm dalla cima (z={z_center:.3f}, "
                                   "codice a barre libero)")
        # centro del libro rispetto al punto di presa, lungo l'altezza (0 se
        # presa a meta'): serve alla mostra alla testa
        self._book_dz = float(self.z_center - z_center) if self.kind == "book" else 0.0
        if self.kind != "book" and z_top - OBJECT_GRASP_FROM_TOP > z_center:
            # Oggetti bassi (mappamondo 11 cm, portapenne 9 cm): presi a meta'
            # altezza l'avambraccio finisce sotto il bordo del ripiano (vedi
            # PLANK_FRONT_X). Presa vicino alla cima: la collision e' un box
            # alto quanto l'oggetto, le dita stringono comunque.
            z_center = z_top - OBJECT_GRASP_FROM_TOP
            self.get_logger().info(f"Oggetto basso: presa alzata da z={self.z_center:.3f} a {z_center:.3f} "
                                   f"(cima a {z_top:.3f})")
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
        if self.kind != "book" and p_max >= p_min:
            # Lo "spessore" misurato dalla depth e' la larghezza della faccia
            # frontale: per un oggetto tondo (mappamondo: 66 mm misurati,
            # box di collision 80) e' meno della larghezza massima -> con
            # p_min+2 mm le dita urtavano il fronte. Se c'e' spazio, 1 cm in piu'.
            p_app = max(0.0, min(p_max, p_min + OBJECT_EXTRA_OPENING))
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
            p_pre = rel(g - [back, 0, 0])
            # (a) come sempre: pre-grasp con ripartenze (seme q_g), poi percorso
            #     pre -> presa. E' la versione verificata dal vivo sui libri.
            q_p, _w, e_p = self.kin.ik(p_pre, q0=q_g, waist=w_g)
            a_in, w_in, dy_in, ang_in, _x = free_path(p_pre, rel(g), 4, q_p, w_g, 0.0, "pitch")
            w_p = list(w_g)
            clr_f = plank_clearance(q_g, w_g, (a_in, w_in))
            if clr_f < PLANK_CLEARANCE_MIN:
                # (b) 2026-09-13: l'IK del pre-grasp con le ripartenze casuali
                #     sceglieva un ramo "gomito basso" (portapenne: gomito a
                #     z=0.950, avambraccio 3.7 cm sotto il ripiano) anche con la
                #     posa di presa a posto. Percorso pianificato ALL'INDIETRO
                #     dalla presa e rovesciato: stesso ramo di q_grasp e finisce
                #     esattamente nella posa di presa. Solo se serve, per non
                #     cambiare le prese dei libri gia' verificate.
                a_bk, w_bk, dy_b, ang_b, _x = free_path(rel(g), p_pre, 4, q_g, w_g, 0.0, "pitch")
                q_b, w_b = list(a_bk[-1]), list(w_bk[-1])
                a_b = [list(q) for q in reversed(a_bk[:-1])] + [list(q_g)]
                w_bi = [list(w) for w in reversed(w_bk[:-1])] + [list(w_g)]
                clr_b = plank_clearance(q_g, w_g, (a_b, w_bi))
                self.get_logger().info(
                    f"    avvicinamento in avanti: avambraccio {clr_f*1000:+.0f} mm dal ripiano -> "
                    f"pianificato all'indietro dalla presa: {clr_b*1000:+.0f} mm")
                if clr_b > clr_f:
                    pos_b, _R = self.kin.fk_arm(q_b, tuple(w_b))
                    q_p, w_p, e_p = q_b, w_b, float(np.linalg.norm(pos_b - p_pre))
                    a_in, w_in, dy_in, ang_in = a_b, w_bi, dy_b, ang_b
            a_out, w_out, dy_out, ang_out, x_end = free_path(rel(g), rel(g + [-retreat_m, 0, 0]), 10,
                                                             a_in[-1], w_in[-1], RETREAT_LIFT, True)
            x_short = max(0.0, x_end - float(rel(g + [-retreat_m, 0, 0])[0]))
            errs = dict(pre=e_p, approach=dy_in, retreat=dy_out, uscita_incompleta=x_short)
            angs = dict(approach=ang_in, retreat=ang_out)
            return (q_p, w_p), (a_in, w_in), (a_out, w_out), errs, angs

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
        # Ripiano sotto la presa: l'avambraccio (gomito->polso) e l'attacco
        # della pinza devono restare sopra il suo piano quando stanno oltre
        # il bordo anteriore (2026-09-13, mappamondo incastrato sotto il bordo).
        surface_z = max([z_ for z_ in SHELF_SURFACES_Z if z_ <= z_center + 1e-3] or [SHELF_SURFACES_Z[0]])

        def plank_clearance(q_c, w_c, p_in):
            """Quota minima (m) sopra il ripiano di avambraccio e attacco
            pinza, nella posa di presa e lungo l'avvicinamento; 1.0 se
            nessun punto sta sopra il ripiano."""
            off = np.array([self.robot_x, self.robot_y, 0.0])
            worst_c = 1.0
            for qq, ww in [(q_c, w_c)] + list(zip(p_in[0], p_in[1])):
                pts = self.kin.fk_joints(qq, tuple(ww))
                el = pts["right_elbow_joint"] + off
                wr = pts["right_wrist_yaw_joint"] + off
                mo = pts["right_gripper_mount_joint"] + off
                for s_ in np.linspace(0.0, 1.0, 11):
                    pt = el + (wr - el) * s_
                    if pt[0] >= PLANK_FRONT_X - FOREARM_RADIUS:
                        worst_c = min(worst_c, pt[2] - FOREARM_RADIUS - surface_z)
                if mo[0] >= PLANK_FRONT_X - FOREARM_RADIUS:
                    worst_c = min(worst_c, mo[2] - FOREARM_RADIUS - surface_z)
            return worst_c

        def search_grasp():
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
                (q_p, w_p), p_in, p_out, errs_c, angs_c = plan_paths(q_c, w_c, retreat)
                worst = max(errs_c.values())
                worst_ang = max(angs_c.values())
                self.get_logger().info(
                    f"  candidato {label}: presa {e_c*1000:.1f} mm, orientamento {o_c:.1f} gradi, "
                    f"percorsi: laterale max {worst*1000:.1f} mm, pinza ruotata max {worst_ang:.1f} gradi")
                clr = plank_clearance(q_c, w_c, p_in)
                self.get_logger().info(f"    avambraccio sopra il ripiano: {clr*1000:+.0f} mm (min {PLANK_CLEARANCE_MIN*1000:.0f})")
                feasible = worst <= 0.02 and worst_ang <= 6.0 and clr >= PLANK_CLEARANCE_MIN
                score = (0 if feasible else 1, o_c if feasible else worst + max(0.0, PLANK_CLEARANCE_MIN - clr))
                if best is None or score < best[0]:
                    best = (score, label, q_c, w_c, e_c, o_c, q_p, p_in, p_out, errs_c, clr, w_p)
                if feasible and o_c <= 3.0:
                    break
            return best

        z_max = z_top - 0.015
        while True:
            best = search_grasp()
            clr_best = best[10]
            if clr_best >= PLANK_CLEARANCE_MIN:
                break
            z_new = float(g[2]) + 0.01
            if z_new > z_max + 1e-6:
                self.get_logger().warn(
                    f"Avambraccio a {clr_best*1000:+.0f} mm dal ripiano (min {PLANK_CLEARANCE_MIN*1000:.0f}) e "
                    f"nessuna quota piu' alta disponibile (cima {z_top:.3f}): procedo, rischio di incastro")
                break
            self.get_logger().info(
                f"Avambraccio a {clr_best*1000:+.0f} mm dal ripiano: alzo la presa a z={z_new:.3f}")
            g[2] = z_new
        _score, label, q_grasp, w_grasp, e1, o_best, q_pre, path_in, path_out, errs_p, _clr, w_pre_plan = best
        # Direzione "alto del libro" nel frame del gripper (2026-09-17): a
        # seconda del ramo IK la X del gripper punta verso l'alto O verso il
        # basso; il libro e' rigido con la pinza, quindi si memorizza qui e
        # si riusa nella mostra alla testa (up = R @ _book_up_g).
        _pg, R_g = self.kin.fk_arm(q_grasp, tuple(w_grasp))
        self._book_up_g = R_g.T @ np.array([0.0, 0.0, 1.0])
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
        w_pre = list(w_pre_plan)     # vita del pre-grasp come pianificata (percorso all'indietro)

        # Ordine (2026-09-17): PRIMA il braccio raccolto (mano libera davanti
        # al petto), POI le dita: a casa la mano sta contro il fianco e il
        # dito esterno non si apriva (restava a 1 mm per minuti: entrava fra
        # i libri chiuso e spingeva Hunger Games contro IT). Poi si verifica
        # che ENTRAMBE le dita siano aperte prima di avvicinarsi.
        ok = (
            self.arm(CARRY_ARM, 2.5, "1. braccio raccolto")
            and self.gripper(p_app, 1.0, f"1b. apri gripper a {p_app*1000:.1f} mm/dito (avvicinamento)")
            and self.fingers_open_check(p_app, "1c. verifica dita aperte")
            and self.waist(list(w_pre), 2.0, "2. vita (pitch) per il pre-grasp")
            and self.arm(q_pre, 3.0, "3. braccio pre-grasp")
            and self.move_both(path_in[0], path_in[1], 3.0, "4-5. avvicinamento rettilineo fino al grasp (braccio+vita)")
        )
        if not ok:
            return
        if self.contact_close:
            ok = self.close_until_contact(p_app, "6. chiudo le dita fino al contatto") is not None
            # Offset del libro rispetto al TCP lungo l'asse delle dita
            # (2026-09-17): il dito con il sensore tocca a p_actual; con il
            # libro centrato toccherebbe a p_nom = (collision - gap_min)/2.
            # Il libro viene incollato dov'e', quindi alla rimessa a posto
            # il percorso va traslato di questo offset (Hunger Games: 8 mm,
            # il libro rientrava contro IT).
            self._book_off = np.zeros(3)
            if ok and not self.dry_run:
                p_nom = max(0.0, (self.thick_close - GRIPPER_MIN_GAP) / 2.0)
                d_off = p_nom - float(self._grip_now)
                _pg, R_gg = self.kin.fk_arm(q_grasp, tuple(w_grasp))
                self._book_off = R_gg[:, 1] * d_off
                self.get_logger().info(f"   libro fuori centro di {d_off*1000:+.1f} mm lungo le dita "
                                       f"(contatto a {self._grip_now*1000:.1f} mm, atteso {p_nom*1000:.1f})")
        else:
            ok = self.gripper(opening, 1.5, f"6. chiudo le dita a {opening*1000:.1f} mm/lato")
            self._book_off = np.zeros(3)
        if not ok:
            return
        self._pause(0.5)
        # Prima di incollare: il dito con il sensore deve toccare PROPRIO
        # questo oggetto (2026-09-17: Hunger Games, dita deviate dalla
        # parete, chiuse fuori posto e libro incollato sollevato e storto).
        # (con la chiusura a contatto il tocco e' gia' verificato da
        # close_until_contact; la finestra qui e' in tempo REALE: a RTF 0.05
        # 2 s reali sono 0.1 s sim, troppo pochi per un nuovo messaggio del
        # sensore - 2026-09-17, Hunger Games abortito subito dopo un contatto
        # valido)
        if not self.dry_run and not self.contact_close:
            self._contact_model = None
            t_c = time.time()
            while time.time() - t_c < 20.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._contact_model == self.entity:
                    break
            if self._contact_model != self.entity:
                self.get_logger().error(
                    f"7. le dita non toccano {self.entity} (contatto: {self._contact_model or 'nessuno'}): "
                    "presa fuori posto, NON incollo. Apro e torno a casa.")
                # apertura di AVVICINAMENTO, non tutta: le dita sono ancora
                # fra i libri vicini (2026-09-17: con GRIPPER_OPEN spingeva
                # IT e la Ballata fuori dallo scaffale)
                self.gripper(p_app, 1.0, f"apri le dita a {p_app*1000:.1f} mm/dito")
                self.move_both([list(q) for q in reversed(path_in[0][:-1])] + [list(q_pre)],
                               [list(w) for w in reversed(path_in[1][:-1])] + [list(w_pre)], 3.0,
                               "arretro fino al pre-grasp")
                # PRIMA raccolto, POI casa (2026-09-17): dal pre-grasp l'interpolazione
                # diretta verso [0,0,0,0,0] passa con il braccio orizzontale dentro
                # la libreria (spalla bloccata a -1.44 rad contro i libri).
                self.arm(CARRY_ARM, 2.5, "braccio raccolto")
                self.waist(HOME_WAIST, 3.0, "vita a casa")
                self.arm(HOME_ARM, 2.5, "braccio a casa")
                return False
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
        shown = False
        if self.head_isbn and self.kind == "book":
            self.show_to_head_and_read([DROP_WAIST[0], float(path_out[1][-1][1])])
            shown = True
        if self.put_back:
            return self.put_back_on_shelf(q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app, shown)
        if w_rel[0] > -1.0 or shown:
            ok = self.arm(CARRY_ARM, 2.5, "11b. braccio raccolto (slot con busto poco ruotato)"
                          if not shown else "11b. braccio raccolto (dopo la mostra alla testa)")
            if not ok:
                return
        ok = (
            self.waist(list(w_rel), 2.0, "12. busto verso lo slot")
            and self.arm(list(q_rel), 3.0, "13. braccio sul punto di rilascio dentro il tavolo"
                         + (" (libro coricato)" if face_down else ""))
        )
        if not ok:
            return
        self.get_logger().info(f"14. DETACH ({self.detach_pub.topic_name} + /gripper/right/detach) + apro il gripper")
        if not self.dry_run:
            self.detach_pub.publish(Empty())
            self.detach_all_pub.publish(Empty())
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
        try:
            node.set_auto_attach(True)       # il teleop di Cate lo ritrova acceso
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
