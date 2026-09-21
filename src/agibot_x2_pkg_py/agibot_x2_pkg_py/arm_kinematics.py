"""
Helpers for the X2 arm kinematics read from the URDF: numpy FK on the chain world -> pelvis -> waist ->
torso -> shoulder pitch/roll/yaw -> elbow -> wrist_yaw -> gripper_base_link, and numeric IK (scipy
least_squares within joint limits) that puts the TCP between the fingers on a Cartesian pose.
Gripper frame: fingers extend along -Z, open/close along +-Y, TCP at (0, 0, -TCP_OFFSET).
Usage: kin = ArmKinematics.from_package()   # or from_urdf(path)
       arm, waist, err = kin.ik(target_xyz, approach=(1,0,0), finger_axis=(0,1,0), optimize_waist=True)
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
WAIST_JOINTS = ["waist_yaw_joint", "waist_pitch_joint"]  # waist_roll fixed at 0
TIP_LINK = "right_gripper_base_link"
JOINT_LIMIT_MARGIN = 0.005   # rad inside the URDF limit (see from_urdf)
# left arm: mirrored chain, chosen with from_package(side="left")
ARM_JOINTS_SIDE = {
    "right": list(ARM_JOINTS),
    "left": [j.replace("right_", "left_", 1) for j in ARM_JOINTS],
}
TIP_LINK_SIDE = {"right": TIP_LINK, "left": "left_gripper_base_link"}
TCP_OFFSET = 0.05   # m along gripper -Z: finger center (finger joint at -0.045, 0.08 tall)

# Finger joints at y=+-0.02 with 0.01 thick boxes (x2_hand_gazebo.urdf): gap = GRIPPER_MIN_GAP + 2p.
# Objects thinner than GRIPPER_MIN_GAP cannot be squeezed, only held by the DetachableJoint attach.
GRIPPER_MIN_GAP = 0.030   # m between inner finger faces at position 0
GRIPPER_OPEN = 0.037      # m per finger, fully open
GRASP_SQUEEZE = 0.001     # m, touch without interpenetrating


def grasp_opening(thickness: float) -> float:
    """
    Finger position to squeeze an object of the given thickness
    """
    return max(0.0, (thickness - GRIPPER_MIN_GAP) / 2.0 - GRASP_SQUEEZE)


FINGER_THICKNESS = 0.010     # along Y (box .04 x .01 x .08 in x2_hand_gazebo.urdf)
APPROACH_CLEARANCE = 0.004   # inner finger face to object while approaching (IK position error <= 2 mm)
NEIGHBOUR_MARGIN = 0.003     # outer finger face to the nearest neighbour


def approach_opening(thickness: float, free_plus: float, free_minus: float):
    """
    APPROACH opening of each finger (m) to slide beside an object of `thickness` with
    `free_plus`/`free_minus` m free to the neighbours or walls (fully open fingers hit close books).
    Inner finger face at GRIPPER_MIN_GAP/2 + p, outer face at GRIPPER_MIN_GAP/2 + FINGER_THICKNESS + p.
    Returns (p, p_min, p_max); p_max < p_min = no room for the fingers, the caller must stop
    """
    half = thickness / 2.0
    inner0 = GRIPPER_MIN_GAP / 2.0
    p_min = half - inner0 + APPROACH_CLEARANCE
    p_max = min(GRIPPER_OPEN,
                half + min(free_plus, free_minus) - NEIGHBOUR_MARGIN
                - inner0 - FINGER_THICKNESS)
    # in tight spaces graze the target book rather than the neighbours: margin over p_min <= half the slack
    p = min(p_max, p_min + min(0.002, max(0.0, (p_max - p_min) / 2.0)))
    return max(0.0, p), max(0.0, p_min), p_max


def _rpy_matrix(r, p, y):
    """
    Rotation matrix from URDF roll/pitch/yaw (Rz @ Ry @ Rx)
    """
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_angle(axis, theta):
    """
    Rotation matrix of angle theta about axis (Rodrigues)
    """
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


class ArmKinematics:
    """
    FK/IK of one X2 arm (or any URDF chain, e.g. the head camera) with the waist
    """
    def __init__(self, chain, limits, base_z=0.0, arm_joints=None, side="right"):
        """
        chain: list of {name, type, xyz, rpy, axis} dicts from world to the tip link
        limits: {joint_name: (lo, hi)}
        base_z: world z of the robot 'world' link (model spawn z)
        arm_joints: the 5 arm joints of this chain (right or left)
        """
        self.chain = chain
        self.limits = limits
        self.base_z = base_z
        self.side = side
        self.arm_joints = list(arm_joints or ARM_JOINTS)

    # ─── construction ────────────────────────────────────────────────────

    @classmethod
    def from_urdf(cls, urdf_path: str, base_z: float = 0.662, tip: str = None, side: str = "right"):
        """
        Build the chain from a URDF; tip = last link (default: the gripper of `side`).
        With tip="rgbd_head_front_link" it is the head camera chain: use fk_link
        """
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
        chain, link = [], (tip or TIP_LINK_SIDE[side])
        while link in child_of:
            jn = child_of[link]
            chain.append(joints[jn])
            link = joints[jn]["parent"]
        chain.reverse()
        limits = {j["name"]: j["limit"] for j in chain if j["limit"] is not None}
        # never solve exactly at the URDF limit: a joint driven to its end stop sticks and MoveIt rejects the start state
        limits = {n: (lo + JOINT_LIMIT_MARGIN, hi - JOINT_LIMIT_MARGIN) if hi - lo > 2 * JOINT_LIMIT_MARGIN else (lo, hi)
                  for n, (lo, hi) in limits.items()}
        return cls(chain, limits, base_z, arm_joints=ARM_JOINTS_SIDE[side], side=side)

    @classmethod
    def from_package(cls, base_z: float = 0.662, tip: str = None, side: str = "right"):
        """
        Build from x2_hand_gazebo.urdf in the agibot_x2_pkg share directory
        """
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory("agibot_x2_pkg")
        return cls.from_urdf(os.path.join(pkg, "urdf", "x2_hand_gazebo.urdf"), base_z, tip, side)

    # ─── forward kinematics ──────────────────────────────────────────────

    def fk(self, q: dict):
        """
        (world TCP position, R gripper -> world) from q = {joint_name: angle}, missing = 0.
        The virtual base joints (base_x/y/z/yaw) count as zero (prismatic ones are ignored):
        valid in the WORKING POSE after the walk, where robot_x = book_placer.ROBOT_SPAWN_X
        """
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

    def fk_link(self, q: dict):
        """
        (position, R) of the LAST link of the chain without the TCP offset, for non-gripper
        chains (e.g. the head camera). q: {joint_name: value}, missing = 0
        """
        T = np.eye(4)
        T[2, 3] = self.base_z
        for j in self.chain:
            L = np.eye(4)
            L[:3, :3] = _rpy_matrix(*j["rpy"])
            L[:3, 3] = j["xyz"]
            T = T @ L
            if j["type"] in ("revolute", "continuous"):
                Rj = np.eye(4)
                Rj[:3, :3] = _axis_angle(j["axis"], q.get(j["name"], 0.0))
                T = T @ Rj
        return T[:3, 3].copy(), T[:3, :3].copy()

    def fk_joints(self, arm, waist=(0.0, 0.0)):
        """
        Origins of ALL chain joints (same frame as fk): {joint_name: xyz}.
        Used to check elbow, wrist and gripper mount clearance: a correct TCP does not mean the forearm clears the shelf
        """
        q = dict(zip(self.arm_joints, arm))
        q.update(dict(zip(WAIST_JOINTS, waist)))
        T = np.eye(4)
        T[2, 3] = self.base_z
        out = {}
        for j in self.chain:
            L = np.eye(4)
            L[:3, :3] = _rpy_matrix(*j["rpy"])
            L[:3, 3] = j["xyz"]
            T = T @ L
            if j["type"] in ("revolute", "continuous"):
                Rj = np.eye(4)
                Rj[:3, :3] = _axis_angle(j["axis"], q.get(j["name"], 0.0))
                T = T @ Rj
            out[j["name"]] = T[:3, 3].copy()
        return out

    def fk_arm(self, arm, waist=(0.0, 0.0)):
        """
        fk from the 5 arm angles and (waist_yaw, waist_pitch)
        """
        q = dict(zip(self.arm_joints, arm))
        q.update(dict(zip(WAIST_JOINTS, waist)))
        return self.fk(q)

    # ─── inverse kinematics ──────────────────────────────────────────────

    def ik(self, target_xyz, approach=(1.0, 0.0, 0.0), finger_axis=(0.0, 1.0, 0.0),
           q0=None, waist=(0.0, 0.0), optimize_waist=False,
           w_pos=50.0, w_approach=0.4, w_finger=6.0, restarts=6, finger_signed=False):
        """
        Angles putting the TCP on target_xyz with gripper -Z along `approach` and +-Y parallel to `finger_axis`.
        Returns (arm[5], waist[2], position_error_m). optimize_waist: False, True (yaw+pitch) or "pitch".
        finger_signed: +Y must point exactly along finger_axis (to show a SPECIFIC cover to the head camera).
        Weights: 1 mm of position ~ 10 deg of orientation, position dominates (fixed 12 deg shoulder tilt).
        restarts: 6 for the first waypoint, 0 with q0 = previous solution afterwards, or the solver may
        jump to another kinematic branch and swing the arm with the book in hand
        """
        from scipy.optimize import least_squares

        target = np.asarray(target_xyz, dtype=float)
        appr = np.asarray(approach, dtype=float); appr /= np.linalg.norm(appr)
        fax = np.asarray(finger_axis, dtype=float); fax /= np.linalg.norm(fax)

        # "pitch": at the shelf, leaning the torso extends reach without turning the robot
        if optimize_waist == "pitch":
            free_waist = [WAIST_JOINTS[1]]
        elif optimize_waist:
            free_waist = list(WAIST_JOINTS)
        else:
            free_waist = []
        names = list(self.arm_joints) + free_waist
        lo = np.array([self.limits[n][0] for n in names])
        hi = np.array([self.limits[n][1] for n in names])
        if q0 is None:
            x0 = np.zeros(5)
            # arm forward, elbow slightly bent: avoids the hanging-arm local minimum and contorted postures
            x0[0] = -1.3
            x0[3] = -0.5
        else:
            x0 = np.array(q0, dtype=float)[:5]
        if free_waist:
            init_w = [waist[WAIST_JOINTS.index(n)] for n in free_waist]
            x0 = np.concatenate([x0, np.asarray(init_w, dtype=float)])
        x0 = np.clip(x0, lo + 1e-3, hi - 1e-3)

        def waist_of(x):
            """
            Waist tuple with the optimized joints taken from x
            """
            w = list(waist)
            for i, n in enumerate(free_waist):
                w[WAIST_JOINTS.index(n)] = x[5 + i]
            return tuple(w)

        def residuals(x):
            """
            Weighted position, approach and finger-axis residuals
            """
            arm = x[:5]
            w = waist_of(x)
            tcp, R = self.fk_arm(arm, w)
            minus_z = -R[:, 2]
            y_axis = R[:, 1]
            r_pos = (tcp - target) * w_pos
            r_app = (minus_z - appr) * w_approach
            # sign-free parallelism: 1 - |dot|
            d_fin = float(y_axis @ fax)
            r_fin = np.array([((1.0 - d_fin) if finger_signed else (1.0 - abs(d_fin))) * w_finger])
            return np.concatenate([r_pos, r_app, r_fin])

        # random restarts around x0 (non-convex); picking the solution NEAREST to q0 keeps waypoints continuous
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
        # tiers: good (< 5 mm), then okay (< 15 mm), nearest to q0; minimum cost only as last resort (contorted branch)
        def nearest(cands):
            """
            Candidate closest to x0 in arm joint space
            """
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
        n joint configurations along p_from -> p_to (p_from excluded), each seeded by the
        previous one, so the TCP follows ~a straight line instead of an arc.
        Returns (list of arm[5], max error in m)
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
