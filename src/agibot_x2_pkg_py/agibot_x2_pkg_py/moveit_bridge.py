#!/usr/bin/env python3
"""
Helpers for the small part of MoveIt 2 used by pick_test_book.
move_joints plans (OMPL, with scene and held-object collisions) and executes a joint goal via
/move_action; joint values come from arm_kinematics.py IK or known poses, not from MoveIt IK.
check_path validates a straight path with /check_state_validity; attach/detach/place/remove
notify planning_scene_builder. All calls spin the caller's node (rclpy.spin_once).
"""
import math
import os
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
    """
    Thin client for move_group actions/services and the scene builder topics
    """

    def __init__(self, node, joint_state_getter, logger=None, planner_id=""):
        """
        joint_state_getter(): dict joint name -> current position
        planner_id: a planner_config from config/ompl_planning.yaml (e.g. RRT* with
        PathLengthOptimizationObjective); "" = MoveIt default (RRTConnect)
        """
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
        """
        True if /move_action and /check_state_validity are reachable
        """
        ok = self.move_client.wait_for_server(timeout_sec=timeout_s) and \
            self.validity.wait_for_service(timeout_sec=timeout_s)
        return bool(ok)

    def _spin_until(self, fut, timeout_s):
        """
        Spin the node until the future is done or the timeout expires
        """
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return fut.done()

    def move_joints(self, group, names, values, label, planning_time=20.0, attempts=8,
                    vel_scale=1.0, timeout_s=2400.0, plan_only=False, tol=0.005):
        """
        Plan and execute a joint-space goal with move_group, return True on success
        Retries once on START_STATE_INVALID (scene still updating right after an attach) and
        once with RRTConnect if a custom planner (e.g. RRT*, which uses the whole time budget)
        finds nothing in a narrow passage.
        """
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.planner_id = self.planner_id
        req.num_planning_attempts = int(attempts)
        req.allowed_planning_time = float(planning_time)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(vel_scale)
        req.start_state.is_diff = True          # start from the monitor's current state
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
            self.log.error(f"{label}: move_group not responding")
            return False
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.log.error(f"{label}: MoveIt goal rejected")
            return False
        res = handle.get_result_async()
        if not self._spin_until(res, timeout_s):
            self.log.error(f"{label}: MoveIt did not finish within {timeout_s:.0f} s")
            handle.cancel_goal_async()
            return False
        r = res.result().result
        code = int(r.error_code.val)
        if code == MoveItErrorCodes.START_STATE_INVALID and not getattr(self, "_retried", False):
            self.log.warn(f"{label}: START_STATE_INVALID, retrying in 3 s")
            t0 = time.time()
            while time.time() - t0 < 3.0:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self._retried = True
            try:
                return self.move_joints(group, names, values, label, planning_time, attempts, vel_scale, timeout_s, plan_only)
            finally:
                self._retried = False
        if code != MoveItErrorCodes.SUCCESS:
            self.log.error(f"{label}: MoveIt failed: {ERR.get(code, code)} "
                           f"(planning {r.planning_time:.1f} s)")
            if code == MoveItErrorCodes.FAILURE and self.planner_id and not self._fb:
                # single fallback: RRTConnect usually solves narrow passages in a few seconds
                self.log.warn(f"{label}: '{self.planner_id}' found no plan: retrying with the "
                              "default planner (RRTConnect)")
                old_id, self.planner_id, self._fb = self.planner_id, "", True
                try:
                    return self.move_joints(group, names, values, label + " (RRTConnect)", planning_time,
                                            attempts, vel_scale, timeout_s, plan_only, tol)
                finally:
                    self.planner_id, self._fb = old_id, False
            return False
        n_pts = len(r.planned_trajectory.joint_trajectory.points)
        self.log.info(f"{label}: MoveIt ok, {n_pts} points, planned in {r.planning_time:.1f} s")
        return True

    @staticmethod
    def _densify(waypoints, max_step):
        """
        Linearly interpolate waypoints in joint space, one sample every `max_step` (rad or m)
        Returns [(position, values)], position = waypoint index k or k + t for intermediate samples.
        """
        pts = [(0, [float(v) for v in waypoints[0]])]
        for k in range(len(waypoints) - 1):
            a, b = [float(v) for v in waypoints[k]], [float(v) for v in waypoints[k + 1]]
            n = max(1, int(math.ceil(max(abs(y - x) for x, y in zip(a, b)) / max_step)))
            for j in range(1, n + 1):
                t = j / n
                pts.append((k + t if j < n else k + 1, [x + (y - x) * t for x, y in zip(a, b)]))
        return pts

    def check_path(self, names, waypoints, group, label, max_step=None):
        """
        Check every waypoint and every sample in between for collisions (other joints at current state)
        Samples between waypoints matter: the waist can turn 2.4 rad in 12 steps, leaving 0.2 rad gaps.
        max_step: max rad/m between samples (default 0.05 or X2_CHECK_STEP); 0 = waypoints only.
        Returns (ok, [messages]); never relaxes anything.
        """
        if not self.validity.wait_for_service(timeout_sec=2.0):
            return None, [f"{label}: /check_state_validity non disponibile"]
        if max_step is None:
            max_step = float(os.environ.get("X2_CHECK_STEP", "0.05"))
        base = dict(self.js() or {})
        pts = self._densify(waypoints, max_step) if max_step > 0 and len(waypoints) > 1 else \
            [(k, [float(v) for v in wp]) for k, wp in enumerate(waypoints)]
        problems = []
        for pos, wp in pts:
            k = int(pos) if float(pos).is_integer() else None
            where = f"waypoint {k}/{len(waypoints) - 1}" if k is not None else \
                f"fra i waypoint {int(pos)} e {int(pos) + 1} (a {pos - int(pos):.2f})"
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
                problems.append(f"{where}: nessuna risposta")
                continue
            resp = fut.result()
            if not resp.valid:
                det = []
                for c in resp.contacts:
                    pz = c.position
                    det.append(f"{c.contact_body_1}<->{c.contact_body_2} @({pz.x:+.3f},{pz.y:+.3f},{pz.z:+.3f}) "
                               f"prof {c.depth*1000:.1f}mm n=({c.normal.x:+.2f},{c.normal.y:+.2f},{c.normal.z:+.2f})")
                q_txt = ", ".join(f"{n.split('_joint')[0]}={float(v):+.3f}" for n, v in zip(names, wp))
                problems.append(f"{where}: collisione {det or 'sconosciuta'} [frame radice URDF] giunti: {q_txt}")
        if problems:
            for p in problems:
                self.log.error(f"{label}: {p}")
            return False, problems
        n_mid = len(pts) - len(waypoints)
        self.log.info(f"{label}: {len(waypoints)} waypoint" + (f" + {n_mid} intermediate samples" if n_mid > 0 else "")
                      + " collision-free (planning scene)")
        return True, []

    def _wait_scene(self, pred, timeout_s, label):
        """
        Wait on the latched /scene_builder/state until pred(state) is true
        """
        import json
        state = {}

        def _cb(m):
            """
            Merge the JSON scene state into `state`
            """
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
            self.log.warn(f"{label}: planning scene not updated within {timeout_s:.0f} s (state {state})")
        return ok

    def attach(self, side, name):
        """
        Attach object `name` to hand `side` in the planning scene
        """
        self.pub_attach.publish(String(data=f"{side}:{name}"))
        return self._wait_scene(lambda st: st.get("attached", {}).get(side) == name, 10.0, f"attach {name}")

    def detach(self, side):
        """
        Detach the object held by hand `side`
        """
        self.pub_detach.publish(String(data=side))
        return self._wait_scene(lambda st: side not in st.get("attached", {}), 10.0, f"detach {side}")

    def place(self, name, x, y, z_base):
        """
        Put a detached object on the table: base at z_base, center at (x, y) [Gazebo frame]
        """
        self.pub_place.publish(String(data=f"{name}:{x:.4f}:{y:.4f}:{z_base:.4f}"))
        return self._wait_scene(lambda st: name in st.get("placed", {}), 10.0, f"place {name}")

    def remove(self, name):
        """
        Remove an object from the planning scene
        """
        self.pub_remove.publish(String(data=name))
