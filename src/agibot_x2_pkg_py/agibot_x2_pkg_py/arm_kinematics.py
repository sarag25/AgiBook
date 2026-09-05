"""
Cinematica del braccio destro dell'X2 ricavata direttamente dall'URDF
(2026-08-30): forward kinematics numpy sulla catena
  world -> pelvis -> waist_yaw -> waist_pitch -> waist_roll -> torso
        -> right_shoulder_pitch/roll/yaw -> right_elbow -> right_wrist_yaw
        -> right_gripper_base_link
e inverse kinematics numerica (scipy least_squares con i limiti dei
giunti) per portare il punto fra le dita (TCP) su una posa cartesiana.

Perche' esiste: il progetto non aveva nessuna IK - le sequenze
(action_sequencer.py, pick_book_node.py) usano angoli hardcoded calibrati
per una distanza scaffale ormai cambiata e mai ricalibrati (vedi
Gazebo.md). Con questo modulo pick_test_book.py calcola le pose da un
punto 3D noto (la posizione dei libri fisici di test), stile demo
MOGI-ROS (che pero' usa una IK analitica per il SUO braccio: qui la
geometria e' diversa, quindi IK numerica generica).

Convenzioni (frame del gripper, right_gripper_base_link):
  - le dita si protendono lungo -Z (finger joint a z=-0.045)
  - le dita si aprono/chiudono lungo +-Y
  - il TCP e' a (0, 0, -TCP_OFFSET) = fra le dita
Per afferrare un dorso di libro rivolto verso il robot (-X world):
  approach = -Z_gripper deve puntare +X world, Y_gripper parallelo a Y world.

Uso:
    kin = ArmKinematics.from_urdf(path)         # o from_package()
    q = kin.ik(target_xyz, approach=(1,0,0), finger_axis=(0,1,0),
               q0=None, waist=(0.0, 0.0))     # -> 5 angoli braccio
    q_full = kin.ik(..., optimize_waist=True)  # -> (waist_yaw, waist_pitch, arm[5])
"""

from __future__ import annotations
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
]
WAIST_JOINTS = ["waist_yaw_joint", "waist_pitch_joint"]  # waist_roll fisso a 0
TIP_LINK = "right_gripper_base_link"
TCP_OFFSET = 0.05   # m lungo -Z del gripper: centro delle dita (finger a -0.045, alte 0.08)

# Geometria delle dita (x2_hand_gazebo.urdf): finger joint a y=+-0.02
# (0.030 dal 2026-08-31, era 0.010: l'offset dei giunti e' passato da
# +-0.01 a +-0.02 nel commit "commit pre-pull" senza aggiornare questo
# valore), box spesse 0.01 -> a posizione 0 le facce interne distano
# GRIPPER_MIN_GAP; ogni dito si allontana di p dal centro,
# gap = GRIPPER_MIN_GAP + 2p. Oggetti piu' sottili di GRIPPER_MIN_GAP
# (Ballata 2.5 cm, Mietitura 2.2 cm) non si possono stringere: si
# prendono solo con l'attach del DetachableJoint.
GRIPPER_MIN_GAP = 0.030
GRIPPER_OPEN = 0.037
GRASP_SQUEEZE = 0.001


def grasp_opening(thickness: float) -> float:
    """Posizione di ciascun dito per stringere un oggetto di spessore dato
    (leggero squeeze per toccare senza compenetrare)."""
    return max(0.0, (thickness - GRIPPER_MIN_GAP) / 2.0 - GRASP_SQUEEZE)


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_angle(axis, theta):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


class ArmKinematics:
    def __init__(self, chain, limits, base_z=0.0):
        """
        chain : lista di dict {name, type, xyz, rpy, axis} da world al TIP_LINK
        limits: {joint_name: (lo, hi)}
        base_z: quota world del link 'world' del robot (z di spawn del modello)
        """
        self.chain = chain
        self.limits = limits
        self.base_z = base_z

    # ─── costruzione ─────────────────────────────────────────────────────

    @classmethod
    def from_urdf(cls, urdf_path: str, base_z: float = 0.662):
        root = ET.parse(urdf_path).getroot()
        joints, child_of = {}, {}
        for j in root.findall("joint"):
            o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            joints[j.get("name")] = dict(
                name=j.get("name"), type=j.get("type"),
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                xyz=xyz, rpy=rpy,
                axis=[float(v) for v in a.get("xyz").split()] if a is not None else None,
                limit=(float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else None,
            )
            child_of[j.find("child").get("link")] = j.get("name")
        chain, link = [], TIP_LINK
        while link in child_of:
            jn = child_of[link]
            chain.append(joints[jn])
            link = joints[jn]["parent"]
        chain.reverse()
        limits = {j["name"]: j["limit"] for j in chain if j["limit"] is not None}
        return cls(chain, limits, base_z)

    @classmethod
    def from_package(cls, base_z: float = 0.662):
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory("agibot_x2_pkg")
        return cls.from_urdf(os.path.join(pkg, "urdf", "x2_hand_gazebo.urdf"), base_z)

    # ─── forward kinematics ──────────────────────────────────────────────

    def fk(self, q: dict):
        """q: {joint_name: angolo}; giunti mobili non presenti = 0.
        Ritorna (posizione TCP world, R gripper->world)."""
        T = np.eye(4)
        T[2, 3] = self.base_z
        for j in self.chain:
            L = np.eye(4)
            L[:3, :3] = _rpy_matrix(*j["rpy"])
            L[:3, 3] = j["xyz"]
            T = T @ L
            if j["type"] in ("revolute", "continuous"):
                theta = q.get(j["name"], 0.0)
                Rj = np.eye(4)
                Rj[:3, :3] = _axis_angle(j["axis"], theta)
                T = T @ Rj
        R = T[:3, :3]
        tcp = T[:3, 3] + R @ np.array([0.0, 0.0, -TCP_OFFSET])
        return tcp, R

    def fk_arm(self, arm, waist=(0.0, 0.0)):
        q = dict(zip(ARM_JOINTS, arm))
        q.update(dict(zip(WAIST_JOINTS, waist)))
        return self.fk(q)

    # ─── inverse kinematics ──────────────────────────────────────────────

    def ik(self, target_xyz, approach=(1.0, 0.0, 0.0), finger_axis=(0.0, 1.0, 0.0),
           q0=None, waist=(0.0, 0.0), optimize_waist=False,
           w_pos=50.0, w_approach=0.4, w_finger=6.0, restarts=6):
        # restarts (2026-08-30): numero di ripartenze casuali attorno a q0.
        # Per il PRIMO waypoint di una sequenza conviene esplorare (6);
        # per i successivi va usato restarts=0 con q0 = soluzione
        # precedente: altrimenti il risolutore puo' saltare a un ramo
        # cinematico diverso a costo minore (visto: "lift" con spalla_yaw
        # ruotata di 137 gradi rispetto a "grasp") e il braccio farebbe
        # una sbracciata con il libro in mano.
        # Pesi (2026-08-30, dopo il primo test): con w_pos=1 un errore di
        # 5 cm valeva quanto 10 gradi di orientamento e il risolutore
        # sacrificava la posizione. Ora 1 mm di posizione ~ 10 gradi di
        # orientamento: la posizione domina, l'orientamento e' best-effort
        # (la spalla ha un tilt fisso di 12 gradi che il roll, limitato a
        # +0.061 rad, non puo' cancellare del tutto - e' accettabile: le
        # dita sono alte 8 cm contro dorsi di 18-23 cm).
        """
        Trova gli angoli che portano il TCP su target_xyz con -Z del gripper
        allineato ad `approach` e +-Y del gripper parallelo a `finger_axis`
        (segno libero: le dita sono simmetriche).

        Ritorna (arm[5], waist[2], errore_posizione_m). Con optimize_waist
        anche waist_yaw/pitch entrano nell'ottimizzazione (utile per il
        tavolo, che sta di lato).
        """
        from scipy.optimize import least_squares

        target = np.asarray(target_xyz, dtype=float)
        appr = np.asarray(approach, dtype=float); appr /= np.linalg.norm(appr)
        fax = np.asarray(finger_axis, dtype=float); fax /= np.linalg.norm(fax)

        # optimize_waist: False = vita fissa a `waist`; True = yaw+pitch
        # liberi; "pitch" = solo il pitch libero (yaw fisso) - utile davanti
        # allo scaffale, dove inclinare il busto allunga la portata senza
        # girare il robot.
        if optimize_waist == "pitch":
            free_waist = [WAIST_JOINTS[1]]
        elif optimize_waist:
            free_waist = list(WAIST_JOINTS)
        else:
            free_waist = []
        names = list(ARM_JOINTS) + free_waist
        lo = np.array([self.limits[n][0] for n in names])
        hi = np.array([self.limits[n][1] for n in names])
        if q0 is None:
            x0 = np.zeros(5)
            # spalla in avanti, gomito leggermente piegato: punto di
            # partenza "braccio davanti al robot", evita il minimo locale
            # del braccio che pende lungo il corpo e le posture contorte
            # con spalla_yaw/polso ai limiti viste nel primo test
            x0[0] = -1.3
            x0[3] = -0.5
        else:
            x0 = np.array(q0, dtype=float)[:5]
        if free_waist:
            init_w = [waist[WAIST_JOINTS.index(n)] for n in free_waist]
            x0 = np.concatenate([x0, np.asarray(init_w, dtype=float)])
        x0 = np.clip(x0, lo + 1e-3, hi - 1e-3)

        def waist_of(x):
            w = list(waist)
            for i, n in enumerate(free_waist):
                w[WAIST_JOINTS.index(n)] = x[5 + i]
            return tuple(w)

        def residuals(x):
            arm = x[:5]
            w = waist_of(x)
            tcp, R = self.fk_arm(arm, w)
            minus_z = -R[:, 2]
            y_axis = R[:, 1]
            r_pos = (tcp - target) * w_pos
            r_app = (minus_z - appr) * w_approach
            # parallelismo a segno libero: 1 - |dot|
            r_fin = np.array([(1.0 - abs(float(y_axis @ fax))) * w_finger])
            return np.concatenate([r_pos, r_app, r_fin])

        # Restart casuali attorno a x0 (5 DOF, funzione non convessa), poi
        # SELEZIONE (2026-08-30): fra le soluzioni con errore di posizione
        # < pos_tol si prende la piu' VICINA a q0 nello spazio giunti, non
        # quella a costo minimo - e' cio' che garantisce continuita' fra
        # waypoint consecutivi (un seme da solo non basta: dal grasp con
        # gomito al limite il risolutore locale restava bloccato a 6 cm).
        starts = [x0]
        rng = np.random.default_rng(0)
        for _ in range(restarts):
            starts.append(np.clip(x0 + rng.normal(0, 0.5, size=len(x0)), lo + 1e-3, hi - 1e-3))
        sols = []
        for s in starts:
            sol = least_squares(residuals, s, bounds=(lo, hi), max_nfev=400)
            arm_s = sol.x[:5]
            w_s = waist_of(sol.x)
            tcp_s, _ = self.fk_arm(arm_s, w_s)
            sols.append((float(np.linalg.norm(tcp_s - target)), float(sol.cost), sol.x))
        # Selezione a livelli: prima le soluzioni "buone" (< 5 mm), poi le
        # "accettabili" (< 15 mm) - in entrambi i casi la piu' vicina a q0;
        # il minimo costo assoluto solo come ultima spiaggia (e' quello che
        # portava sul ramo contorto).
        def nearest(cands):
            return min(cands, key=lambda s: np.linalg.norm(s[2][:5] - x0[:5]))
        good = [s for s in sols if s[0] < 0.005]
        okay = [s for s in sols if s[0] < 0.015]
        if q0 is not None and good:
            best = nearest(good)
        elif q0 is not None and okay:
            best = nearest(okay)
        elif good:
            best = min(good, key=lambda s: s[1])
        else:
            best = min(sols, key=lambda s: s[1])
        arm = best[2][:5]
        w = waist_of(best[2])
        return list(arm), list(w), best[0]

    def ik_path(self, p_from, p_to, n, q0, waist=(0.0, 0.0), **kw):
        """
        n configurazioni giunti lungo il segmento p_from -> p_to (escluso
        p_from), ciascuna seminata dalla precedente con selezione della
        soluzione piu' vicina: il TCP segue ~una retta invece di un arco.
        Ritorna (lista di arm[5], errore max in m).
        """
        p_from = np.asarray(p_from, dtype=float)
        p_to = np.asarray(p_to, dtype=float)
        path, worst, q = [], 0.0, list(q0)
        for i in range(1, n + 1):
            p = p_from + (p_to - p_from) * (i / n)
            q, _w, err = self.ik(p, q0=q, waist=waist, optimize_waist=False, restarts=4, **kw)
            path.append(q)
            worst = max(worst, err)
        return path, worst
