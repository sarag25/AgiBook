#!/usr/bin/env python3
"""
moveit_bridge - il poco di MoveIt 2 che serve a pick_test_book (2026-09-18).

  * move_joints(group, names, values): pianifica (OMPL, con collisioni
    contro la planning scene e l'oggetto in mano) ed ESEGUE un goal a giunti
    tramite l'action /move_action di move_group. Nessuna IK di MoveIt: i
    valori vengono dall'IK di arm_kinematics.py o dalle pose note.
  * check_path(names, waypoints): verifica ogni waypoint di un percorso
    rettilineo (calcolato da pick_test_book) con /check_state_validity:
    collisioni con la scena, con se stesso e con l'oggetto attaccato. Ritorna
    (ok, dettagli): chi tocca chi, in quale waypoint. Non allenta nulla.
  * attach/detach: avvisano planning_scene_builder.

Tutte le chiamate girano lo spinner del nodo chiamante (rclpy.spin_once),
come il resto di pick_test_book.
"""
import time

import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState


ERR = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int) and k.isupper()}


class MoveItBridge:

    def __init__(self, node, joint_state_getter, logger=None, planner_id=""):
        """joint_state_getter(): dict nome->posizione (stato corrente).
        planner_id (2026-09-19, per provare pianificatori "ottimizzanti" tipo
        RRT* con obiettivo PathLengthOptimizationObjective invece del
        default RRTConnect - vedi config/ompl_planning.yaml): nome di un
        planner_config dichiarato li'; "" = default di MoveIt (RRTConnect)."""
        self.node = node
        self.js = joint_state_getter
        self.log = logger or node.get_logger()
        self.planner_id = str(planner_id or "")
        self._fb = False
        self.move_client = ActionClient(node, MoveGroup, "/move_action")
        self.validity = node.create_client(GetStateValidity, "/check_state_validity")
        self.pub_attach = node.create_publisher(String, "/scene_builder/attach", 10)
        self.pub_detach = node.create_publisher(String, "/scene_builder/detach", 10)
        self.pub_remove = node.create_publisher(String, "/scene_builder/remove", 10)
        self.pub_place = node.create_publisher(String, "/scene_builder/place", 10)

    def available(self, timeout_s=3.0) -> bool:
        ok = self.move_client.wait_for_server(timeout_sec=timeout_s) and \
            self.validity.wait_for_service(timeout_sec=timeout_s)
        return bool(ok)

    def _spin_until(self, fut, timeout_s):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return fut.done()

    # ─── pianificazione + esecuzione ─────────────────────────────────────
    def move_joints(self, group, names, values, label, planning_time=20.0, attempts=8,
                    vel_scale=1.0, timeout_s=2400.0, plan_only=False, tol=0.005):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.planner_id = self.planner_id
        req.num_planning_attempts = int(attempts)
        req.allowed_planning_time = float(planning_time)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(vel_scale)
        req.start_state.is_diff = True          # parte dallo stato corrente del monitor
        c = Constraints()
        for n, v in zip(names, values):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = jc.tolerance_below = float(tol)
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.log.info(f"{label}: MoveIt [{group}] -> {[round(float(v), 3) for v in values]}")
        fut = self.move_client.send_goal_async(goal)
        if not self._spin_until(fut, 30.0):
            self.log.error(f"{label}: move_group non risponde")
            return False
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.log.error(f"{label}: goal MoveIt rifiutato")
            return False
        res = handle.get_result_async()
        if not self._spin_until(res, timeout_s):
            self.log.error(f"{label}: MoveIt non ha finito entro {timeout_s:.0f} s")
            handle.cancel_goal_async()
            return False
        r = res.result().result
        code = int(r.error_code.val)
        if code == MoveItErrorCodes.START_STATE_INVALID and not getattr(self, "_retried", False):
            # 2026-09-18: visto una volta subito dopo un attach (scena in
            # aggiornamento); riprovo una volta dopo 3 s
            self.log.warn(f"{label}: START_STATE_INVALID, riprovo fra 3 s")
            t0 = time.time()
            while time.time() - t0 < 3.0:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self._retried = True
            try:
                return self.move_joints(group, names, values, label, planning_time, attempts, vel_scale, timeout_s, plan_only)
            finally:
                self._retried = False
        if code != MoveItErrorCodes.SUCCESS:
            self.log.error(f"{label}: MoveIt fallito: {ERR.get(code, code)} "
                           f"(pianificazione {r.planning_time:.1f} s)")
            if code == MoveItErrorCodes.FAILURE and self.planner_id and not self._fb:
                # 2026-09-19: RRT* usa TUTTO il tempo concesso e in un passaggio stretto (braccio
                # destro a casa, fra il fianco e il bordo del tavolo) non ha trovato nulla in 20 s;
                # RRTConnect (default) di solito si' in pochi secondi. Un solo nuovo tentativo.
                self.log.warn(f"{label}: '{self.planner_id}' non ha trovato un piano: riprovo con il "
                              "planner di default (RRTConnect)")
                old_id, self.planner_id, self._fb = self.planner_id, "", True
                try:
                    return self.move_joints(group, names, values, label + " (RRTConnect)", planning_time,
                                            attempts, vel_scale, timeout_s, plan_only, tol)
                finally:
                    self.planner_id, self._fb = old_id, False
            return False
        n_pts = len(r.planned_trajectory.joint_trajectory.points)
        self.log.info(f"{label}: MoveIt ok, {n_pts} punti, pianificato in {r.planning_time:.1f} s")
        return True

    # ─── verifica di un percorso ─────────────────────────────────────────
    def check_path(self, names, waypoints, group, label):
        """Ogni waypoint (lista di valori per `names`) viene verificato con lo
        stato corrente degli altri giunti. Ritorna (ok, [messaggi])."""
        if not self.validity.wait_for_service(timeout_sec=2.0):
            return None, [f"{label}: /check_state_validity non disponibile"]
        base = dict(self.js() or {})
        problems = []
        for k, wp in enumerate(waypoints):
            st = dict(base)
            st.update({n: float(v) for n, v in zip(names, wp)})
            req = GetStateValidity.Request()
            rs = RobotState()
            rs.joint_state = JointState()
            rs.joint_state.name = list(st.keys())
            rs.joint_state.position = [float(v) for v in st.values()]
            rs.is_diff = False
            req.robot_state = rs
            req.group_name = group
            fut = self.validity.call_async(req)
            if not self._spin_until(fut, 10.0) or fut.result() is None:
                problems.append(f"waypoint {k}: nessuna risposta")
                continue
            resp = fut.result()
            if not resp.valid:
                det = []
                for c in resp.contacts:
                    pz = c.position
                    det.append(f"{c.contact_body_1}<->{c.contact_body_2} @({pz.x:+.3f},{pz.y:+.3f},{pz.z:+.3f}) "
                               f"prof {c.depth*1000:.1f}mm n=({c.normal.x:+.2f},{c.normal.y:+.2f},{c.normal.z:+.2f})")
                q_txt = ", ".join(f"{n.split('_joint')[0]}={float(v):+.3f}" for n, v in zip(names, wp))
                problems.append(f"waypoint {k}/{len(waypoints)-1}: collisione {det or 'sconosciuta'} [frame radice URDF] giunti: {q_txt}")
        if problems:
            for p in problems:
                self.log.error(f"{label}: {p}")
            return False, problems
        self.log.info(f"{label}: {len(waypoints)} waypoint senza collisioni (planning scene)")
        return True, []

    # ─── scena ───────────────────────────────────────────────────────────
    def _wait_scene(self, pred, timeout_s, label):
        """aspetta /scene_builder/state (latched) finche' pred(stato) e' vero"""
        import json
        state = {}

        def _cb(m):
            try:
                state.update(json.loads(m.data))
            except Exception:
                pass
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
        sub = self.node.create_subscription(
            String, "/scene_builder/state", _cb,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        t0 = time.time()
        ok = False
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if state and pred(state):
                ok = True
                break
        self.node.destroy_subscription(sub)
        if not ok:
            self.log.warn(f"{label}: planning scene non aggiornata entro {timeout_s:.0f} s (stato {state})")
        return ok

    def attach(self, side, name):
        self.pub_attach.publish(String(data=f"{side}:{name}"))
        return self._wait_scene(lambda st: st.get("attached", {}).get(side) == name, 10.0, f"attach {name}")

    def detach(self, side):
        self.pub_detach.publish(String(data=side))
        return self._wait_scene(lambda st: side not in st.get("attached", {}), 10.0, f"detach {side}")

    def place(self, name, x, y, z_base):
        """L'oggetto (gia' staccato) sta sul tavolo: base a z_base, centro in (x, y) [Gazebo]."""
        self.pub_place.publish(String(data=f"{name}:{x:.4f}:{y:.4f}:{z_base:.4f}"))
        return self._wait_scene(lambda st: name in st.get("placed", {}), 10.0, f"place {name}")

    def remove(self, name):
        self.pub_remove.publish(String(data=name))
