#!/usr/bin/env python3
"""
walk_to_shelf - camminata CINEMATICA del robot dallo spawn alla posa di
lavoro davanti allo scaffale (2026-09-13, vedi Gazebo.md "Camminata").

Come funziona
-------------
Il pelvis non e' piu' inchiodato al mondo: fra `world` e `pelvis` ci sono 4
giunti virtuali (base_x/y/z prismatici + base_yaw, x2_hand_gazebo.urdf)
comandati in posizione dal JTC `base_controller`. Il robot quindi si
SPOSTA "scivolando" sul mondo, mentre `legs_controller` (12 giunti) anima
le gambe con un passo generato qui, in modo che i piedi non strisciano:
durante l'appoggio il piede arretra rispetto all'anca esattamente alla
velocita' con cui il corpo avanza (contatto "a rotolamento" ideale), durante
l'oscillazione si solleva di step_height e torna avanti. Non c'e' nessun
equilibrio da mantenere (ZMP, IMU, PID sulle gambe): e' una camminata
d'animazione, deterministica e leggera per la CPU - la fisica resta valida
per braccia, libri e oggetti. Un controller di camminata dinamica vero e'
fuori scopo del progetto (e non girerebbe a RTF 10-30% su 8 GB).

Geometria della gamba (sagittale, dall'URDF): anca_pitch -> ginocchio
0.320 m (THIGH), ginocchio -> caviglia 0.282 m (SHANK), caviglia -> pianta
0.060 m (FOOT). Gamba dritta: caviglia a 0.602 m sotto l'anca, pianta a
0.662 = quota di spawn (piedi a terra). Per ogni istante si calcola il punto
caviglia desiderato rispetto all'anca e si risolve l'IK planare a 2
segmenti (ginocchio in avanti, come un umano); la caviglia mantiene il
piede orizzontale (ankle = -(hip + knee)). Segni URDF: hip/knee/ankle
pitch positivi = piede/pianta INDIETRO; ginocchio 0 = dritto, positivo =
flesso (limiti 0..2.41).

Durante il passo la base si abbassa di `crouch` (le ginocchia restano un
po' piegate, come un umano che cammina) e i piedi in appoggio stanno
`clearance` sopra il pavimento: nessun contatto piede-terra, quindi
nessuna forza spuria sul corpo comandato in posizione. Tutto e' moltiplicato
da un inviluppo e(s) che parte da 0 e torna a 0 alla fine: primo e ultimo
passo "a mezzo passo", e alla fine gambe DRITTE a 0 e base esattamente a
(distance, 0, 0, 0) -> il pelvis torna dove la cinematica del braccio lo
aspetta (book_placer.ROBOT_SPAWN_X).

Braccia (arm_swing, default true): oscillazione controlaterale - braccio
destro avanti quando la gamba sinistra e' avanti - di arm_amplitude sul
pitch della spalla, gomito leggermente flesso (elbow_flex). Nella realta'
serve a compensare il momento angolare delle gambe attorno alla verticale
ed e' energeticamente conveniente; qui e' puramente estetico ma
"umano". Un braccio che tiene un oggetto (/gripper/<lato>/attached non
vuoto) NON oscilla, come farebbe una persona con un libro in mano; con
arm_swing:=false le braccia restano ferme dove sono.

Uso:
  ros2 run agibot_x2_pkg_py walk_to_shelf                    # fino a WALK_DISTANCE
  ros2 run agibot_x2_pkg_py walk_to_shelf --ros-args -p distance:=0.5 -p arm_swing:=false
  ros2 run agibot_x2_pkg_py walk_to_shelf --ros-args -p dry_run:=true   # solo traiettoria + log
`distance` e' il valore ASSOLUTO di base_x_joint da raggiungere (= metri
percorsi dallo spawn, default book_placer.WALK_DISTANCE = quello del
launch). Se il robot e' gia' li' non fa nulla. Va lanciato PRIMA di
pick_test_book: la presa presuppone la posa di lavoro.
"""

import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from agibot_x2_pkg.book_placer import WALK_DISTANCE

BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_z_joint", "base_yaw_joint"]
LEG_JOINTS = {
    "left": ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
             "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint"],
    "right": ["right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
              "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"],
}
LEGS_JOINTS = LEG_JOINTS["left"] + LEG_JOINTS["right"]   # ordine di legs_controller
ARM_JOINTS = {
    s: [f"{s}_shoulder_pitch_joint", f"{s}_shoulder_roll_joint", f"{s}_shoulder_yaw_joint",
        f"{s}_elbow_joint", f"{s}_wrist_yaw_joint",
        f"{s}_gripper_left_finger_joint", f"{s}_gripper_right_finger_joint"]
    for s in ("left", "right")
}

# geometria sagittale della gamba (x2_hand_gazebo.urdf)
THIGH = 0.31995   # anca_pitch -> ginocchio (z)
SHANK = 0.282     # ginocchio -> caviglia_pitch (z)
FOOT = 0.060      # caviglia -> pianta
LEG_STRAIGHT = THIGH + SHANK   # 0.602: caviglia sotto l'anca a gamba dritta

# limiti URDF (per il clamp)
LIM = {
    "hip": (-2.704, 2.556), "knee": (0.0, 2.4073), "ankle": (-0.803, 0.453),
    "shoulder": (-3.08, 2.04), "elbow": (-2.3556, 0.0),
}


def _clamp(v, lim):
    return min(max(v, lim[0]), lim[1])


def _smoothstep(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def leg_ik(x, z):
    """Angoli (hip_pitch, knee, ankle_pitch) che portano la caviglia in
    (x, z) rispetto all'anca (x avanti, z in alto, z<0), piede orizzontale.
    Ginocchio in avanti. Portata limitata a [|THIGH-SHANK|, THIGH+SHANK]."""
    r = math.hypot(x, z)
    r = min(max(r, abs(THIGH - SHANK) + 1e-4), LEG_STRAIGHT)   # r = LEG_STRAIGHT -> gamba dritta esatta
    # flessione del ginocchio (0 = dritto)
    c = (THIGH ** 2 + SHANK ** 2 - r ** 2) / (2.0 * THIGH * SHANK)
    knee = math.pi - math.acos(min(max(c, -1.0), 1.0))
    # direzione anca->caviglia dalla verticale (positiva in avanti)
    theta = math.atan2(x, -z)
    # angolo fra coscia e retta anca->caviglia
    cb = (THIGH ** 2 + r ** 2 - SHANK ** 2) / (2.0 * THIGH * r)
    beta = math.acos(min(max(cb, -1.0), 1.0))
    hip = -(theta + beta)          # positivo = coscia indietro (URDF)
    ankle = -(hip + knee)          # piede orizzontale
    return (_clamp(hip, LIM["hip"]), _clamp(knee, LIM["knee"]), _clamp(ankle, LIM["ankle"]))


class WalkToShelf(Node):

    def __init__(self):
        super().__init__("walk_to_shelf")
        self.declare_parameter("distance", float(WALK_DISTANCE))  # base_x_joint finale [m]
        self.declare_parameter("speed", 0.45)        # m/s (tempo simulato)
        self.declare_parameter("cycle", 1.2)         # s per ciclo completo (2 passi)
        self.declare_parameter("step_height", 0.04)  # sollevamento del piede in oscillazione
        self.declare_parameter("crouch", 0.025)      # abbassamento della base durante il passo
        self.declare_parameter("clearance", 0.005)   # pianta sopra il pavimento in appoggio
        self.declare_parameter("ramp", 0.6)          # s di accelerazione/decelerazione
        self.declare_parameter("dt", 0.05)           # passo dei waypoint
        # "Corpo che cammina" (2026-09-13, dopo il primo video: "sembra che
        # scivoli"): il busto non deve avanzare rigido a velocita' costante.
        # bob = oscillazione verticale (alto a meta' appoggio, basso nel
        # doppio appoggio), sway = ondeggiamento laterale sopra il piede
        # d'appoggio, pelvis_yaw = rotazione del bacino verso la gamba che
        # avanza. Tutti moltiplicati dall'inviluppo: a fine camminata zero.
        self.declare_parameter("bob", 0.02)          # m picco-picco
        self.declare_parameter("sway", 0.025)        # m di ampiezza
        self.declare_parameter("pelvis_yaw", 0.05)   # rad di ampiezza
        self.declare_parameter("arm_swing", True)
        self.declare_parameter("arm_amplitude", 0.25)  # rad sul pitch della spalla
        self.declare_parameter("elbow_flex", 0.35)     # rad di flessione del gomito
        self.declare_parameter("dry_run", False)

        g = lambda n: self.get_parameter(n).value
        self.distance = float(g("distance"))
        self.speed = max(0.05, float(g("speed")))
        self.cycle = max(0.4, float(g("cycle")))
        self.step_height = float(g("step_height"))
        self.crouch = float(g("crouch"))
        self.clearance = float(g("clearance"))
        self.ramp = max(0.1, float(g("ramp")))
        self.dt = max(0.02, float(g("dt")))
        self.bob = float(g("bob"))
        self.sway = float(g("sway"))
        self.pelvis_yaw = float(g("pelvis_yaw"))
        self.arm_swing = bool(g("arm_swing"))
        self.arm_amp = float(g("arm_amplitude"))
        self.elbow_flex = float(g("elbow_flex"))
        self.dry_run = bool(g("dry_run"))
        # falcata: distanza percorsa dal corpo durante UN appoggio (mezzo ciclo)
        self.stride = self.speed * self.cycle / 2.0
        low = self.crouch + self.bob / 2      # punto piu' basso del bacino (doppio appoggio)
        max_stride = 2.0 * math.sqrt(max(LEG_STRAIGHT ** 2 - (LEG_STRAIGHT - low - self.clearance) ** 2, 1e-6)) * 0.95
        if self.stride > max_stride:
            self.get_logger().warn(
                f"falcata {self.stride:.2f} m oltre la portata della gamba ({max_stride:.2f} m): "
                f"ridotta (alza crouch o abbassa speed/cycle)")
            self.stride = max_stride

        self._ac = {
            "base": ActionClient(self, FollowJointTrajectory, "/base_controller/follow_joint_trajectory"),
            "legs": ActionClient(self, FollowJointTrajectory, "/legs_controller/follow_joint_trajectory"),
            "left": ActionClient(self, FollowJointTrajectory, "/left_arm_controller/follow_joint_trajectory"),
            "right": ActionClient(self, FollowJointTrajectory, "/right_arm_controller/follow_joint_trajectory"),
        }
        self.joint_pos = {}
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.attached = {"left": "", "right": ""}
        for s in ("left", "right"):
            self.create_subscription(String, f"/gripper/{s}/attached",
                                     lambda m, s=s: self.attached.__setitem__(s, m.data), latched)

    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joint_pos[n] = p

    # ─── generazione del passo ───────────────────────────────────────────

    def _profile(self, d):
        """Ascissa curvilinea s(t) trapezoidale (rampe smoothstep) da 0 a d>0.
        Ritorna liste (t, s)."""
        v, tr = self.speed, self.ramp
        d_ramp = v * tr / 2.0
        if d < 2 * d_ramp:                 # troppo corto per raggiungere v
            tr = math.sqrt(d / v * tr)     # rampe piu' corte, stessa accelerazione media
            v = d / tr
            d_ramp = d / 2.0
        t_cruise = (d - 2 * d_ramp) / v
        T = 2 * tr + t_cruise
        ts, ss = [], []
        # tempi per indice (n*dt), NON accumulando t += dt: con d = 0.9 m
        # T = 2.6 s = 52*dt e l'accumulo in virgola mobile dava un ultimo
        # punto a 2.5999999 s seguito dal finale a 2.6 s -> i controller
        # rifiutavano la traiettoria ("time between points ... not strictly
        # increasing"), 2026-09-13.
        n_last = int(math.floor(T / self.dt - 0.25))   # ultimo indice con t <= T - dt/4
        for n in range(1, n_last + 1):
            t = n * self.dt
            if t < tr:
                s = v * t * t / (2 * tr)          # rampa lineare in velocita'
            elif t < tr + t_cruise:
                s = d_ramp + v * (t - tr)
            else:
                td = T - t
                s = d - v * td * td / (2 * tr)
            ts.append(t); ss.append(min(max(s, 0.0), d))
        ts.append(T); ss.append(d)
        return ts, ss

    def _foot(self, phase, env, sign):
        """(x caviglia rel. anca, sollevamento) per una gamba a fase `phase`
        (0..2pi: [0,pi) oscillazione indietro->avanti, [pi,2pi) appoggio
        avanti->indietro), inviluppo env in [0,1], sign = verso di marcia."""
        ph = phase % (2 * math.pi)
        S = self.stride * env
        if ph < math.pi:                      # oscillazione
            u = ph / math.pi
            x = -S / 2 + S * _smoothstep(u)
            lift = self.step_height * env * math.sin(math.pi * u)
        else:                                 # appoggio: lineare = niente strisciamento
            u = (ph - math.pi) / math.pi
            x = S / 2 - S * u
            lift = 0.0
        return sign * x, lift

    def plan(self, x0):
        """Waypoint (t, base[4], legs[12], swing_left, swing_right, env) per
        andare da base_x = x0 a self.distance."""
        d = self.distance - x0
        sign = 1.0 if d >= 0 else -1.0
        ts, ss = self._profile(abs(d))
        env_len = max(self.stride, 0.15)      # inviluppo su circa una falcata
        pts = []
        for t, s in zip(ts, ss):
            env = min(_smoothstep(s / env_len), _smoothstep((abs(d) - s) / env_len))
            phase = math.pi * s / self.stride   # un appoggio (pi) ogni falcata
            # fase 0/pi = doppio appoggio (basso), pi/2 e 3pi/2 = meta' appoggio (alto)
            base_z = (-self.crouch - self.bob / 2 * math.cos(2 * phase)) * env
            # gamba sinistra in appoggio per phase in [pi, 2pi): corpo verso +y
            base_y = -self.sway * math.sin(phase) * env
            # bacino ruotato verso la gamba che avanza (sinistra in oscillazione = +yaw)
            base_yaw = self.pelvis_yaw * math.sin(phase) * env
            legs = []
            feet = {}
            for side, ph in (("left", phase), ("right", phase + math.pi)):
                xf, lift = self._foot(ph, env, sign)
                feet[side] = xf
                # caviglia rispetto all'anca: pianta a `clearance*env` dal pavimento
                z_ankle = -(LEG_STRAIGHT + base_z) + self.clearance * env + lift
                hip, knee, ankle = leg_ik(xf, z_ankle)
                legs += [hip, 0.0, 0.0, knee, ankle, 0.0]
            base = [x0 + sign * s, base_y, base_z, base_yaw]
            half = max(self.stride / 2, 1e-3)
            # controlaterale: braccio destro avanti (pitch negativo) con piede sinistro avanti
            sw_r = -self.arm_amp * feet["left"] / half * env
            sw_l = -self.arm_amp * feet["right"] / half * env
            pts.append((t, base, legs, sw_l, sw_r, env))
        return pts

    # ─── invio ───────────────────────────────────────────────────────────

    @staticmethod
    def _goal(names, wps):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        for t, q in wps:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            ns = int(round(t * 1e9))
            pt.time_from_start = Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
            goal.trajectory.points.append(pt)
        return goal

    def run(self):
        # stato corrente (serve base_x e le braccia)
        needed = BASE_JOINTS + LEGS_JOINTS + ARM_JOINTS["left"] + ARM_JOINTS["right"]
        t0 = self.get_clock().now()
        while any(n not in self.joint_pos for n in needed):
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.get_clock().now() - t0).nanoseconds > 15e9:
                missing = [n for n in needed if n not in self.joint_pos]
                self.get_logger().error(f"/joint_states senza {missing[:4]}...: URDF/controller vecchi? "
                                        "Ricompila agibot_x2_pkg e rilancia.")
                return False
        x0 = self.joint_pos["base_x_joint"]
        if abs(self.distance - x0) < 0.005:
            self.get_logger().info(f"base_x gia' a {x0:.3f} m: niente da fare")
            return True

        pts = self.plan(x0)
        T = pts[-1][0]
        n_steps = int(round(abs(self.distance - x0) / self.stride))
        self.get_logger().info(
            f"Camminata: base_x {x0:.2f} -> {self.distance:.2f} m, {self.speed:.2f} m/s, "
            f"falcata {self.stride:.2f} m (~{n_steps} passi), {T:.1f} s sim, {len(pts)} waypoint, "
            f"braccia: {'oscillano' if self.arm_swing else 'ferme'}")
        legs_max = np.max(np.abs([p[2] for p in pts]), axis=0)
        self.get_logger().info(
            f"gambe max |anca| {legs_max[0]:.2f} |ginocchio| {legs_max[3]:.2f} "
            f"|caviglia| {legs_max[4]:.2f} rad; base_z min {min(p[1][2] for p in pts):.3f} m")

        goals = {
            "base": self._goal(BASE_JOINTS, [(p[0], p[1]) for p in pts]),
            "legs": self._goal(LEGS_JOINTS, [(p[0], p[2]) for p in pts]),
        }
        if self.arm_swing:
            for side, idx in (("left", 3), ("right", 4)):
                if self.attached.get(side):
                    self.get_logger().info(
                        f"braccio {side}: tiene '{self.attached[side]}', non oscilla")
                    continue
                q_now = [self.joint_pos[n] for n in ARM_JOINTS[side]]
                wps = []
                for p in pts:
                    q = list(q_now)
                    q[0] = _clamp(q_now[0] + p[idx], LIM["shoulder"])
                    q[3] = _clamp(q_now[3] - self.elbow_flex * p[5], LIM["elbow"])
                    wps.append((p[0], q))
                goals[side] = self._goal(ARM_JOINTS[side], wps)

        if self.dry_run:
            self.get_logger().info("dry_run: nessun goal inviato")
            return True
        for k in goals:
            if not self._ac[k].wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f"action server {k} non disponibile "
                                        "(base_controller/legs_controller attivi?)")
                return False
        futs = {k: self._ac[k].send_goal_async(g) for k, g in goals.items()}
        for f in futs.values():
            rclpy.spin_until_future_complete(self, f)
        handles = {k: f.result() for k, f in futs.items()}
        bad = [k for k, h in handles.items() if h is None or not h.accepted]
        if bad:
            self.get_logger().error(f"goal rifiutato: {bad}")
            return False
        for k, h in handles.items():
            r = h.get_result_async()
            rclpy.spin_until_future_complete(self, r)
        rclpy.spin_once(self, timeout_sec=0.5)
        xf = self.joint_pos.get("base_x_joint", float("nan"))
        self.get_logger().info(f"Arrivato: base_x = {xf:.3f} m (posa di lavoro: pelvis a ROBOT_SPAWN_X). "
                               "Ora: ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=it")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = WalkToShelf()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
