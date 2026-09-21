#!/usr/bin/env python3
"""
ROS 2 node that fills the MoveIt planning scene (/apply_planning_scene) from perception.
Objects measured by the head camera (/tmp/x2_detections.json or /library_manager/detections) become
boxes "obj<id>"; bookshelf and table come from the URDF geometry at the launch default pose (not measured),
with the shelf under the detections aligned to the measured height. /scene_builder/attach|detach|place|remove
move an object between world and gripper (AttachedCollisionObject) so transports are planned with it.
  ros2 run agibot_x2_pkg_py planning_scene_builder
"""
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
import tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK
from moveit_msgs.msg import PlanningSceneComponents

from agibot_x2_pkg.book_placer import (SHELF_SURFACES_Z, SHELF_HALF_INNER_WIDTH, ROBOT_SPAWN_X, WALK_DISTANCE,
                                       BOOK_COLLISION_SIDE_MARGIN)

# MoveIt "world" = URDF root at the Gazebo spawn pose; Gazebo coordinates are shifted by -spawn
SPAWN_DEFAULT = (ROBOT_SPAWN_X - WALK_DISTANCE, 0.0, 0.662)

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# bookshelf and table geometry, derived from the URDFs
from agibot_x2_pkg.scene_config import (
    PLANK_FRONT_X as SHELF_X_FRONT, SHELF_X_BACK, SHELF_BACK_PANEL_X, SHELF_HALF_OUTER, SHELF_PLANK_T, SHELF_TOP_Z,
    TABLE_CENTER, TABLE_SIZE, TABLE_TOP_CENTER_Z, TABLE_LEG, TABLE_LEG_XY)
DEFAULT_DEPTH = 0.16        # m, assumed depth of a book whose top face is not visible (see docs/COSTANTI.md)


_SPAWN = list(SPAWN_DEFAULT)      # set by the node (robot_spawn_x/y/z parameters)


def _matrix_to_quat(R):
    """
    Convert a 3x3 rotation matrix to a geometry_msgs/Quaternion
    """
    from geometry_msgs.msg import Quaternion
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        r = math.sqrt(1.0 + t); w = 0.5 * r; r = 0.5 / r
        x, y, z = (R[2][1] - R[1][2]) * r, (R[0][2] - R[2][0]) * r, (R[1][0] - R[0][1]) * r
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        r = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]); x = 0.5 * r; r = 0.5 / r
        w, y, z = (R[2][1] - R[1][2]) * r, (R[0][1] + R[1][0]) * r, (R[0][2] + R[2][0]) * r
    elif R[1][1] > R[2][2]:
        r = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]); y = 0.5 * r; r = 0.5 / r
        w, x, z = (R[0][2] - R[2][0]) * r, (R[0][1] + R[1][0]) * r, (R[1][2] + R[2][1]) * r
    else:
        r = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]); z = 0.5 * r; r = 0.5 / r
        w, x, y = (R[1][0] - R[0][1]) * r, (R[0][2] + R[2][0]) * r, (R[1][2] + R[2][1]) * r
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


def _quat_to_matrix(x, y, z, w):
    """
    Convert a quaternion (x, y, z, w) to a 3x3 rotation matrix
    """
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return [
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ]


def _box(name, size, center, frame="world"):
    """
    Box CollisionObject; center in Gazebo coordinates, expressed in the URDF root frame
    """
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    pose = Pose()
    pose.position.x = float(center[0]) - _SPAWN[0]
    pose.position.y = float(center[1]) - _SPAWN[1]
    pose.position.z = float(center[2]) - _SPAWN[2]
    pose.orientation.w = 1.0
    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def _stack(name, profile, x_front, y_c, length, band=0.01, frame="world"):
    """
    CollisionObject with one primitive per band of the profile [(z, width)]
    Round objects (width varies > 25 % along z, e.g. globe) use cylinders, so non-existent box corners
    do not block the palm approaching at an angle; constant-section objects use boxes.
    """
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    widths = [float(w) for _z, w in profile if float(w) > 0.003]
    round_obj = bool(widths) and (max(widths) - min(widths)) / max(widths) > 0.25
    for z, w in profile:
        w = float(w)
        if w <= 0.003:
            continue
        depth = float(length) if length > 0.0 else w
        prim = SolidPrimitive()
        if round_obj:
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [band, max(w, depth) / 2.0]     # [height, radius]
            depth = max(w, depth)
        else:
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [depth, w, band]
        pose = Pose()
        pose.position.x = float(x_front) + depth / 2.0 - _SPAWN[0]
        pose.position.y = float(y_c) - _SPAWN[1]
        pose.position.z = float(z) - _SPAWN[2]
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


class PlanningSceneBuilder(Node):
    """
    Keeps move_group's planning scene in sync with perception and the grasp state
    """
    _quat_to_matrix = staticmethod(_quat_to_matrix)

    def __init__(self):
        """
        Declare parameters, create service clients, subscriptions and the polling timer
        """
        super().__init__("planning_scene_builder")
        self.declare_parameter("detections_file", "/tmp/x2_detections.json")
        self.declare_parameter("poll_s", 2.0)
        self.declare_parameter("static_scene", True)
        self.declare_parameter("robot_spawn_x", float(SPAWN_DEFAULT[0]))
        self.declare_parameter("robot_spawn_y", float(SPAWN_DEFAULT[1]))
        self.declare_parameter("robot_spawn_z", float(SPAWN_DEFAULT[2]))
        _SPAWN[:] = [float(self.get_parameter(f"robot_spawn_{a}").value) for a in "xyz"]
        self.get_logger().info(f"URDF root frame (MoveIt world) at spawn pose {_SPAWN} (Gazebo coords)")
        self.det_file = str(self.get_parameter("detections_file").value)
        self._seeded = False
        self._pending_topic = None
        self._mtime = None
        self._objects = {}          # id -> CollisionObject (in the world)
        self._attached = {}         # side -> obj id
        self._plank_z = None
        cbg = ReentrantCallbackGroup()   # _apply() blocks on a threading.Event
        self._fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene", callback_group=cbg)
        self.create_subscription(String, "/library_manager/detections", self._det_cb, LATCHED, callback_group=cbg)
        self.create_subscription(String, "/scene_builder/attach", self._attach_cb, 10, callback_group=cbg)
        self.create_subscription(String, "/scene_builder/detach", self._detach_cb, 10, callback_group=cbg)
        self.create_subscription(String, "/scene_builder/place", self._place_cb, 10, callback_group=cbg)
        self._placed = {}
        self.create_subscription(String, "/scene_builder/remove", self._remove_cb, 10, callback_group=cbg)
        self.create_subscription(Empty, "/scene_builder/refresh", lambda _m: self._reload(force=True), 10, callback_group=cbg)
        self.pub_state = self.create_publisher(String, "/scene_builder/state", LATCHED)
        self._ready = False
        # steady clock: with use_sim_time and RTF < 0.05 a sim-time timer would take minutes
        self.create_timer(float(self.get_parameter("poll_s").value), self._tick, callback_group=cbg,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"planning_scene_builder: waiting for /apply_planning_scene; detections from {self.det_file}")

    def _apply(self, scene: PlanningScene, what: str, timeout_s: float = 10.0) -> bool:
        """
        Apply a planning scene diff synchronously, return success
        Never spins from inside a callback: a threading.Event set by the done-callback (run on another
        MultiThreadedExecutor thread) unblocks the wait.
        """
        scene.is_diff = True
        if not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{what}: /apply_planning_scene not available (move_group not up yet?)")
            return False
        fut = self.cli.call_async(ApplyPlanningScene.Request(scene=scene))
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        got = done.wait(timeout=timeout_s)
        if not got:
            self.get_logger().error(f"planning scene: {what} -> TIMEOUT ({timeout_s:.0f} s, no response)")
            return False
        try:
            ok = bool(fut.result() is not None and fut.result().success)
        except Exception as e:
            ok = False
            self.get_logger().error(f"planning scene: {what}: {e}")
        if ok:
            self.get_logger().info(f"planning scene: {what} -> ok")
        else:
            self.get_logger().error(f"planning scene: {what} -> FAILED")
        return ok

    def _static_objects(self):
        """
        Collision objects of the bookshelf (planks, walls, back) and the table
        """
        objs = []
        surfaces = list(SHELF_SURFACES_Z)
        if self._plank_z is not None:
            # align the closest shelf to the measured height
            k = min(range(len(surfaces)), key=lambda i: abs(surfaces[i] - self._plank_z))
            if abs(surfaces[k] - self._plank_z) < 0.05:
                surfaces[k] = self._plank_z
        depth = SHELF_X_BACK - SHELF_X_FRONT
        xc = (SHELF_X_FRONT + SHELF_X_BACK) / 2.0
        for i, z in enumerate(surfaces):
            objs.append(_box(f"shelf_plank_{i}", (depth, 2 * SHELF_HALF_OUTER, SHELF_PLANK_T),
                             (xc, 0.0, z - SHELF_PLANK_T / 2.0)))
        objs.append(_box("shelf_top", (depth, 2 * SHELF_HALF_OUTER, SHELF_PLANK_T),
                         (xc, 0.0, SHELF_TOP_Z - SHELF_PLANK_T / 2.0)))
        wall_t = SHELF_HALF_OUTER - SHELF_HALF_INNER_WIDTH
        for name, y in (("shelf_wall_plus", SHELF_HALF_INNER_WIDTH + wall_t / 2.0),
                        ("shelf_wall_minus", -(SHELF_HALF_INNER_WIDTH + wall_t / 2.0))):
            objs.append(_box(name, (depth, wall_t, SHELF_TOP_Z), (xc, y, SHELF_TOP_Z / 2.0)))
        back_t = SHELF_X_BACK - SHELF_BACK_PANEL_X
        objs.append(_box("shelf_back", (back_t, 2 * SHELF_HALF_OUTER, SHELF_TOP_Z),
                         (SHELF_BACK_PANEL_X + back_t / 2.0, 0.0, SHELF_TOP_Z / 2.0)))
        objs.append(_box("table_top", TABLE_SIZE, (TABLE_CENTER[0], TABLE_CENTER[1], TABLE_TOP_CENTER_Z)))
        for k, (dx, dy) in enumerate(TABLE_LEG_XY):
            objs.append(_box(f"table_leg_{k}", TABLE_LEG,
                             (TABLE_CENTER[0] + dx, TABLE_CENTER[1] + dy, TABLE_LEG[2] / 2.0)))
        return objs

    def _objects_from(self, dets):
        """
        Build {obj<id>: CollisionObject} from detections and store the measured shelf height
        """
        out = {}
        zb = []
        for d in dets:
            th = float(d.get("thickness_m", 0.0) or 0.0)
            if th <= 0.0:
                continue
            h = float(d["z_top"]) - float(d["z_bottom"])
            depth = float(d.get("length_m", 0.0) or 0.0) or DEFAULT_DEPTH
            if d.get("is_book"):
                # depth measures the mesh; the Gazebo book collision is BOOK_COLLISION_SIDE_MARGIN narrower per side
                th = max(0.03, th - 2.0 * BOOK_COLLISION_SIDE_MARGIN)
            else:
                # objects: ~square footprint if depth is not measured
                depth = float(d.get("length_m", 0.0) or 0.0) or th
            x_front = float(d["world_x"])
            name = f"obj{int(d['id'])}"
            prof = d.get("width_profile") or []
            if not d.get("is_book") and len(prof) >= 3:
                # non-book with width profile: one primitive per 1 cm band, so a sphere is not a single box
                out[name] = _stack(name, prof, x_front, float(d["world_y"]),
                                   float(d.get("length_m", 0.0) or 0.0))
            else:
                out[name] = _box(name, (depth, th, h),
                                 (x_front + depth / 2.0, float(d["world_y"]), (float(d["z_top"]) + float(d["z_bottom"])) / 2.0))
            zb.append(float(d["z_bottom"]))
        if zb:
            self._plank_z = float(min(zb))
        return out

    def _publish_all(self, what):
        """
        Publish static objects plus world objects (not the held ones) to the planning scene
        """
        scene = PlanningScene()
        attached_ids = set(self._attached.values())
        scene.world.collision_objects = []
        for co in self._static_objects():
            scene.world.collision_objects.append(co)
        for name, co in self._objects.items():
            if name not in attached_ids:
                scene.world.collision_objects.append(co)
        ok = self._apply(scene, what)
        self._publish_state(ok)
        return ok

    def _publish_state(self, ok=True):
        """
        Publish the latched JSON state on /scene_builder/state
        """
        self.pub_state.publish(String(data=json.dumps({
            "objects": sorted(self._objects.keys()), "attached": dict(self._attached),
            "plank_z": self._plank_z, "placed": dict(self._placed), "ok": ok})))

    def _load_json(self, text, what):
        """
        Replace world objects from a detections JSON, keeping held and placed objects
        """
        try:
            dets = json.loads(text)
        except Exception as e:
            self.get_logger().error(f"{what}: invalid JSON ({e})")
            return False
        new = self._objects_from(dets)
        # objects placed on the table are missing from later photos but must stay in the scene
        gone = [n for n in self._objects if n not in new and n not in self._attached.values()
                and n not in self._placed]
        if gone:
            scene = PlanningScene()
            for n in gone:
                co = CollisionObject(); co.id = n; co.header.frame_id = "world"; co.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(co)
            self._apply(scene, f"rimuovo {gone}")
        for n in list(self._attached.values()):
            if n in self._objects:
                new[n] = self._objects[n]          # the held object does not change
            elif n in new:
                pass                               # keep the JSON one (not republished: attached)
        for n in self._placed:
            if n in self._objects:
                new[n] = self._objects[n]          # keep where it was placed, not its shelf pose
        self._objects = new
        self.get_logger().info(f"{what}: {len(new)} objects -> {sorted(new)} (shelf at z={self._plank_z})")
        return self._publish_all(what)

    def _reload(self, force=False):
        """
        Reload the detections file if it changed (or if forced)
        """
        try:
            m = os.path.getmtime(self.det_file)
        except OSError:
            return
        if force or m != self._mtime:
            self._mtime = m
            with open(self.det_file, encoding="utf-8") as f:
                self._load_json(f.read(), f"detection da {os.path.basename(self.det_file)}")

    def _det_cb(self, msg):
        """
        Load detections from the topic, deferred until attached objects are seeded
        """
        if not getattr(self, "_seeded", False):
            self._pending_topic = msg.data      # seed attached objects first
            return
        self._load_json(msg.data, "detection dal topic")

    def _seed_attached(self):
        """
        At startup, read objects already attached in move_group's scene so they are not put back in the world
        """
        cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        if not cli.wait_for_service(timeout_sec=2.0):
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        fut = cli.call_async(req)

        def _done(f):
            """
            Record attached objects, then load the pending detections
            """
            try:
                for aco in f.result().scene.robot_state.attached_collision_objects:
                    side = "left" if aco.link_name.startswith("left") else "right"
                    self._attached[side] = aco.object.id
                    self.get_logger().info(f"existing scene: {aco.object.id} attached to {aco.link_name}")
            except Exception as e:
                self.get_logger().warn(f"get_planning_scene: {e}")
            self._seeded = True
            pend = getattr(self, "_pending_topic", None)
            if pend:
                self._load_json(pend, "detection dal topic")
            else:
                self._reload(force=True)
        fut.add_done_callback(_done)

    def _tick(self):
        """
        Wait for move_group once, seed attached objects, then poll the detections file
        """
        if not self._ready:
            if not self.cli.wait_for_service(timeout_sec=0.1):
                return
            self._ready = True
            self.get_logger().info("move_group ready: publishing bookshelf, table and objects")
            self._seed_attached()
            return
            if not self._objects:
                self._publish_all("scena statica")
            return
        self._reload()

    def _attach_cb(self, msg):
        """
        Move "<side>:obj<id>" from the world to <side>_gripper_base_link as AttachedCollisionObject
        MoveIt does not transform the pose by header.frame_id, so position and orientation are
        re-expressed in the link frame via /compute_fk (the TF "world" frame does not exist, root = pelvis).
        """
        self.get_logger().info(f"attach requested: '{msg.data}'")
        try:
            side, name = msg.data.strip().split(":", 1)
        except ValueError:
            self.get_logger().error(f"attach: expected '<side>:obj<id>', got '{msg.data}'")
            return
        co = self._objects.get(name)
        if co is None:
            self.get_logger().error(f"attach: '{name}' is not in the scene {sorted(self._objects)}")
            return
        link = f"{side}_gripper_base_link"
        try:
            Rm, tv = self._world_to_link(link)
            poses_link = []
            for p in co.primitive_poses:
                wp = (p.position.x, p.position.y, p.position.z)
                lp = tuple(Rm[i][0] * wp[0] + Rm[i][1] * wp[1] + Rm[i][2] * wp[2] + tv[i] for i in range(3))
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = lp
                # orientation too, or the box tilts with the rotated gripper: R_link<-world x R_box(world)
                qb = p.orientation
                Rb = _quat_to_matrix(qb.x, qb.y, qb.z, qb.w)
                Rn = [[sum(Rm[i][k] * Rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
                pose.orientation = _matrix_to_quat(Rn)
                poses_link.append(pose)
        except Exception as e:
            self.get_logger().error(f"attach {name}: TF {link}<-world not available ({e}) - "
                                    "the attached object will NOT have a correct pose in the scene")
            poses_link = co.primitive_poses
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object = CollisionObject()
        aco.object.id = name
        aco.object.header.frame_id = link
        aco.object.operation = CollisionObject.ADD
        aco.object.primitives = co.primitives
        aco.object.primitive_poses = poses_link
        aco.touch_links = [link, f"{side}_gripper_left_finger_link",
                           f"{side}_gripper_right_finger_link", f"{side}_wrist_yaw_link"]
        # REMOVE and attached ADD of the same id in one diff failed (see Bugs.md): two separate calls;
        # REMOVE only if it is a world object, otherwise the diff fails ("does not exist in this scene")
        if name not in self._attached.values():
            rm = CollisionObject(); rm.id = name; rm.header.frame_id = "world"; rm.operation = CollisionObject.REMOVE
            scene_rm = PlanningScene(); scene_rm.world.collision_objects = [rm]
            if not self._apply(scene_rm, f"rimuovo {name} dal mondo (prima dell'aggancio)"):
                return
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [aco]
        scene.robot_state.is_diff = True
        if self._apply(scene, f"ATTACH {name} a {aco.link_name}"):
            self._attached[side] = name
            self._publish_state()

    def _world_to_link(self, link, timeout_s=5.0):
        """
        (R, t) such that p_link = R @ p_world + t, from the pose of `link` given by /compute_fk
        """
        cli = self._fk_cli
        if not cli.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("/compute_fk not available")
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = [link]
        fut = cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout_s) or fut.result() is None or not fut.result().pose_stamped:
            raise RuntimeError("/compute_fk no response")
        ps = fut.result().pose_stamped[0].pose
        q = ps.orientation
        Rlw = _quat_to_matrix(q.x, q.y, q.z, q.w)                     # link -> world
        Rwl = [[Rlw[j][i] for j in range(3)] for i in range(3)]       # world -> link (transpose)
        pw = (ps.position.x, ps.position.y, ps.position.z)
        tv = tuple(-sum(Rwl[i][k] * pw[k] for k in range(3)) for i in range(3))
        self.get_logger().info(f"attach: {link} in world at ({pw[0]:+.3f}, {pw[1]:+.3f}, {pw[2]:+.3f}) from /compute_fk")
        return Rwl, tv

    def _detach_cb(self, msg):
        """
        Detach the object held by <side>; it stays in the world where it was released
        """
        self.get_logger().info(f"detach requested: '{msg.data}'")
        side = msg.data.strip()
        name = self._attached.get(side)
        if not name:
            self.get_logger().warn(f"detach {side}: nothing held in the scene")
            return
        aco = AttachedCollisionObject()
        aco.link_name = f"{side}_gripper_base_link"
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE     # back to the world where it is
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [aco]
        scene.robot_state.is_diff = True
        if self._apply(scene, f"DETACH {name} da {aco.link_name}"):
            del self._attached[side]
            self._publish_state()

    @staticmethod
    def _extents(co):
        """
        (xmin, xmax, ymin, ymax, zmin) of a CollisionObject (root frame)
        """
        xs, ys, zs = [], [], []
        for prim, pose in zip(co.primitives, co.primitive_poses):
            d = list(prim.dimensions)
            if prim.type == SolidPrimitive.CYLINDER:
                hx = hy = d[1]; hz = d[0] / 2.0
            else:
                hx, hy, hz = d[0] / 2.0, d[1] / 2.0, d[2] / 2.0
            xs += [pose.position.x - hx, pose.position.x + hx]
            ys += [pose.position.y - hy, pose.position.y + hy]
            zs.append(pose.position.z - hz)
        return min(xs), max(xs), min(ys), max(ys), min(zs)

    def _place_cb(self, msg):
        """
        Move a released object "<name>:<x>:<y>:<z_base>" (Gazebo coords) to where it really lies
        After DETACH move_group leaves it at the hand's release height (TABLE_RELEASE_AIR, 3 cm above the
        table) while in Gazebo it falls onto the table; the too-high copy can collide with the arm later.
        """
        self.get_logger().info(f"place requested: '{msg.data}'")
        try:
            name, xs, ys, zs = msg.data.strip().split(":")
            x, y, zb = float(xs), float(ys), float(zs)
        except ValueError:
            self.get_logger().error(f"place: expected '<name>:<x>:<y>:<z_base>', got '{msg.data}'")
            return
        co = self._objects.get(name)
        if co is None:
            self.get_logger().error(f"place: '{name}' is not in the scene {sorted(self._objects)}")
            return
        if name in self._attached.values():
            self.get_logger().warn(f"place: {name} is still held, not moving it")
            return
        x0, x1, y0, y1, z0 = self._extents(co)
        dx = (x - _SPAWN[0]) - (x0 + x1) / 2.0
        dy = (y - _SPAWN[1]) - (y0 + y1) / 2.0
        dz = (zb - _SPAWN[2]) - z0
        new = CollisionObject()
        new.header.frame_id = co.header.frame_id
        new.id = name
        new.operation = CollisionObject.ADD          # ADD on an existing id replaces it
        new.primitives = co.primitives
        poses = []
        for p in co.primitive_poses:
            q = Pose()
            q.position.x, q.position.y, q.position.z = p.position.x + dx, p.position.y + dy, p.position.z + dz
            q.orientation = p.orientation
            poses.append(q)
        new.primitive_poses = poses
        scene = PlanningScene()
        scene.world.collision_objects = [new]
        if self._apply(scene, f"PLACE {name} sul tavolo (spostato di {dx*100:+.1f}, {dy*100:+.1f}, {dz*100:+.1f} cm)"):
            self._objects[name] = new
            self._placed[name] = [x, y, zb]
            self._publish_state()

    def _remove_cb(self, msg):
        """
        Remove an object from the world
        """
        name = msg.data.strip()
        rm = CollisionObject(); rm.id = name; rm.header.frame_id = "world"; rm.operation = CollisionObject.REMOVE
        scene = PlanningScene(); scene.world.collision_objects = [rm]
        if self._apply(scene, f"rimuovo {name}"):
            self._objects.pop(name, None)


def main(args=None):
    """
    Spin the builder on a MultiThreadedExecutor (needed by the blocking _apply)
    """
    rclpy.init(args=args)
    node = PlanningSceneBuilder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
