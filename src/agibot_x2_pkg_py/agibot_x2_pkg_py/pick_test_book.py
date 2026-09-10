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
from std_msgs.msg import Empty

from agibot_x2_pkg.book_placer import (catalog_entry, collision_size, free_space_sides,
                                       test_entities, ROBOT_SPAWN_X, SHELF_SURFACES_Z)
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

        self.book = self.get_parameter("book").value
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
        entities = test_entities(scene)
        entry = next((e for e in entities if e[0].endswith(self.book) or e[1].startswith(self.book)), None)
        if entry is None:
            raise RuntimeError(f"'{self.book}' non nella scena {scene}: {[e[0] for e in entities]}")
        self.entity, self.key, self.kind, self.bx, self.by = entry
        self.attach_pub = self.create_publisher(Empty, f"/{self.entity}/attach", 10)
        self.detach_pub = self.create_publisher(Empty, f"/{self.entity}/detach", 10)

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
        sx, sy, sz = catalog_entry(self.kind, self.key)["size"]
        z_center = SHELF_SURFACES_Z[3] + sz / 2.0 + 0.002
        spine_x = self.bx - sx / 2.0
        g = np.array([spine_x + float(self.get_parameter("grasp_depth").value), self.by, z_center])
        rel = lambda p: np.asarray(p) - np.array([self.robot_x, self.robot_y, 0.0])
        back = float(self.get_parameter("approach_back").value)
        retreat = float(self.get_parameter("retreat").value)

        self.get_logger().info(
            f"Libro {self.entity} ({self.key}): centro world=({self.bx:.3f},{self.by:.3f},{z_center:.3f}) "
            f"dorso x={spine_x:.3f}, punto di presa world={np.round(g,3).tolist()}, robot a x={self.robot_x}")

        # Apertura di avvicinamento: quanto basta per entrare ai lati
        # dell'oggetto senza urtare i vicini (vedi approach_opening).
        free_plus, free_minus = free_space_sides(self.entity, self.scene)
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
        opening = grasp_opening(collision_size(self.kind, self.key)[1])
        face_down = self.kind == "book"
        if face_down:
            # Libro posato di PIATTO, copertina in giu': dita verticali, il
            # dito inferiore (faccia esterna a TCP - 0.025 - opening) deve
            # restare sopra il piano -> TCP a 5 cm + opening dal piano.
            z_rel = TABLE_TOP_Z + 0.05 + opening
        else:
            z_rel = TABLE_TOP_Z + sz / 2.0 + TABLE_RELEASE_AIR
        p_rel = np.array([self.robot_x + TABLE_RELEASE_X_OFFSET, TABLE_RELEASE_Y, z_rel])
        # Prima la soluzione con le dita orizzontali (libro in piedi): il
        # polso ruota attorno all'asse di avvicinamento (= asse Z del
        # gripper), quindi +-pi/2 sul polso corica il libro senza spostare
        # il TCP; il segno lo decide la normale della copertina.
        q_rel, w_rel, e_rel = self.kin.ik(rel(p_rel), approach=(0, -1, 0), finger_axis=(1, 0, 0),
                                          waist=tuple(DROP_WAIST), optimize_waist="pitch")
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
            and self.gripper(opening, 1.5, f"6. chiudo le dita a {opening*1000:.1f} mm/lato")
        )
        if not ok:
            return
        self._pause(0.5)
        self.get_logger().info(f"7. ATTACH /{self.entity}/attach")
        if not self.dry_run:
            self.attach_pub.publish(Empty())
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
            and self.waist(list(w_rel), 1.5, "12. busto in avanti")
            and self.arm(list(q_rel), 3.0, "13. braccio sul punto di rilascio dentro il tavolo"
                         + (" (libro coricato)" if face_down else ""))
        )
        if not ok:
            return
        self.get_logger().info(f"14. DETACH /{self.entity}/detach + apro il gripper")
        if not self.dry_run:
            self.detach_pub.publish(Empty())
        self._pause(0.3)
        self.gripper(GRIPPER_OPEN, 1.0, "15. apri gripper")
        self._pause(1.0)
        self.arm(CARRY_ARM, 2.5, "16. braccio raccolto")
        self.waist(HOME_WAIST, 3.0, "17. vita a casa")
        self.arm(HOME_ARM, 2.5, "18. braccio a casa")
        self.get_logger().info("Sequenza completata.")


def main(args=None):
    rclpy.init(args=args)
    node = PickTestBook()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
