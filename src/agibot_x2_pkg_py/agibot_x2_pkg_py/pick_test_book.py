"""
ROS 2 node that picks a book or object from the shelf and lays it on the table, puts it back or moves it to another slot.
Arm poses come from inverse kinematics (arm_kinematics.py) on the known or measured object position, no hardcoded angles;
free motions are planned by MoveIt, straight segments are computed here and checked against the planning scene.
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=hunger   # hunger | it | ballata | alba | pen | globe
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p scene:=full -p book:=emma
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p dry_run:=true   # IK only, no motion
"""

import json
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
                                       test_entities, ROBOT_SPAWN_X, WALK_DISTANCE, SHELF_SURFACES_Z,
                                       BOOK_COLLISION_SIDE_MARGIN)
from agibot_x2_pkg_py.arm_kinematics import (
    ArmKinematics, ARM_JOINTS, WAIST_JOINTS, GRIPPER_OPEN, GRIPPER_MIN_GAP, grasp_opening,
    approach_opening, _rpy_matrix)
from agibot_x2_pkg_py.moveit_bridge import MoveItBridge

GRIPPER_JOINTS = ["right_gripper_left_finger_joint", "right_gripper_right_finger_joint"]
# the fingers belong to right_arm_controller (x2_controllers.yaml): every arm trajectory lists all 7 joints
ARM7_JOINTS = ARM_JOINTS + GRIPPER_JOINTS

CARRY_ARM = [-0.5, 0.0, 0.0, -2.2, 0.0]   # arm folded in front of the chest: TCP ~0.28 m from the torso, below the shelf front
# Table release: waist at -90 degrees, torso bent forward to the limit; the release point comes from IK.
# DROP_ARM (arm stretched horizontally, TCP at y=-0.447 z=0.94, 19 cm drop) is only the fallback if IK fails.
DROP_WAIST = [-1.57, 0.31]
DROP_ARM = [-1.57, 0.0, 0.0, 0.0, 0.0]
from agibot_x2_pkg.scene_config import TABLE_TOP_Z, TABLE_RELEASE_Y, SHELF_FRONT_X, PLANK_FRONT_X   # from table.urdf / bookshelf.urdf
TABLE_RELEASE_X_OFFSET = -0.19   # relative to robot_x (rotated right shoulder)
TABLE_RELEASE_AIR = 0.03
# tall narrow objects (pen holder, 47 mm x 85 mm) bounce and tip over when dropped from 3 cm: 1 cm drop (IK error <= 0.3 mm)
OBJECT_TABLE_RELEASE_AIR = 0.01
# The straight exit must bring the whole object past the shelf front (plus margin) before the waist rotates,
# otherwise the rotation sweeps it over the neighbours.
SHELF_EXIT_MARGIN = 0.03
RETREAT_LIFT = 0.03   # height gained during the exit (8 cm free above IT, 2 mm below)
# The forearm (right_elbow_link: cylinder r=0.03) must stay above the plank when past its real front edge
# (PLANK_FRONT_X), otherwise the arm gets stuck under the edge and the fingers close on nothing.
FOREARM_RADIUS = 0.03
PLANK_CLEARANCE_MIN = 0.01      # minimum forearm height above the plank
OBJECT_GRASP_FROM_TOP = 0.025   # objects (not books): grasp near the top
OBJECT_EXTRA_OPENING = 0.010    # objects: wider opening (non-flat faces)
# COVER normal in world at spawn (books with yaw pi: local +Y -> world -y); used to lay the book face down
# on the table (back cover with the ISBN towards the table_camera).
BOOK_COVER_NORMAL_WORLD = np.array([0.0, -1.0, 0.0])
HOME_ARM = [0.0] * 5
HOME_WAIST = [0.0, 0.0]
# Never command a finger to exactly 0 (lower end stop): the OUTER fingers stay stuck there forever, at 0.002
# they reopen. _move applies this minimum to every finger command (see Bugs.md).
FINGER_MIN = 0.002
STAGE_BACK = 0.12          # MoveIt staging pose: pre-grasp moved back 12 cm (outside the shelf front)
# arm:=auto -> left arm for world y > 20 cm: the right arm would need a 109-degree waist yaw and a finger
# brushes the object itself during the approach, while the left arm reaches it with the torso straight.
ARM_AUTO_LEFT_Y = 0.20
# ISBN from the head camera: head_camera in control_file.gazebo (rgbd 1920x1440 on trigger, hfov 1.0,
# the same camera that photographs the bookshelf)
HEAD_JOINTS = ["head_yaw_joint", "head_pitch_joint"]
HEAD_ISBN_HFOV = 1.0                       # = <horizontal_fov> of the sensor
HEAD_ISBN_ASPECT = 1440.0 / 1920.0
HEAD_SENSOR_R = _rpy_matrix(-1.5707963, -1.5707963, 0.0)   # <pose> of the sensor in the link
HEAD_ISBN_TORSO_CLEAR = 0.13   # minimum distance of the book corners from the torso axis
HEAD_ISBN_FOV_MARGIN = 0.92    # corners within 92% of the half field of view
# A 23 cm book does not fit vertically at 30 cm (0.25 m field) and the in-plane book rotation is not
# commandable: up to 3 shots per cover with head pitch -/+ this sweep (centre, then the two halves).
HEAD_ISBN_PITCH_SWEEP = 0.15


class PickTestBook(Node):
    """
    Node that runs one pick sequence (to the table, back to the shelf, to another slot or from the table)
    """

    def __init__(self):
        """
        Declare the parameters, resolve the target object and create publishers, subscribers and action clients
        """
        super().__init__("pick_test_book")
        self.declare_parameter("book", "it")
        self.declare_parameter("scene", "grasp_test")   # grasp_test (default) | full, as in the launch file
        self.declare_parameter("robot_x", ROBOT_SPAWN_X)
        # robot_x = robot_spawn_x + base_x_joint when /joint_states arrives (the walk can stop 1-2 cm off)
        self.declare_parameter("robot_spawn_x", ROBOT_SPAWN_X - WALK_DISTANCE)
        self.declare_parameter("robot_x_from_joints", True)
        self.declare_parameter("robot_y", 0.0)
        self.declare_parameter("dry_run", False)
        # after the exit, show a cover to the head camera and read the barcode (wrist turned 180 deg to retry)
        self.declare_parameter("head_isbn", False)
        # after the exit (and the optional head ISBN) the book goes BACK to its slot instead of the table
        self.declare_parameter("put_back", False)
        # resume_put_back: restart from the end of the exit with the book already in hand (see run());
        # resume_book_off "x,y,z" in mm: offset of the book in the gripper (log "book off centre")
        self.declare_parameter("resume_put_back", False)
        self.declare_parameter("resume_from_detach", False)   # with resume_put_back: only R5-R10
        # release_open (m/finger, nan = automatic): release opening when putting back objects squeezed
        # between a wall and another object (pen holder: at 32 mm the right finger touched the wall)
        self.declare_parameter("release_open", float("nan"))
        # dest_y (reorder): world y of the DESTINATION slot (nan = none). The book is picked, pulled out,
        # moved sideways in front of the shelf and put in the destination slot (never on the table).
        # Exit 3 = destination out of reach for this arm (checked BEFORE moving), exit 1 = other error.
        self.declare_parameter("dest_y", float("nan"))
        # from_table_x/y: the object (globe/pen holder) lies on the TABLE at world (x, y) and goes to the shelf
        # slot given by the detection (world_x/world_y). IK check first (exit 3 = out of reach, nothing moves).
        self.declare_parameter("from_table_x", float("nan"))
        self.declare_parameter("from_table_y", float("nan"))
        self.declare_parameter("resume_book_off", "0,0,0")
        # books: grasp the spine this far from the TOP (0 = mid height). Off by default: a higher grasp made
        # the IK pick a gripper rotated 180 deg and the put-back swept the neighbours.
        self.declare_parameter("grasp_from_top", 0.0)
        self.declare_parameter("head_isbn_dist", 0.25)     # starting camera-cover distance (m)
        # 'right' | 'left' | 'auto' (reach map, then ARM_AUTO_LEFT_Y). The fixed poses have zero shoulder
        # roll/yaw, so they are the same for both sides.
        self.declare_parameter("arm", "auto")
        # MoveIt plans the free motions against the planning scene; the straight segments (approach, exit,
        # re-entry) are computed here and checked against the scene before execution.
        self.declare_parameter("moveit", True)
        # "" = MoveIt default (RRTConnect); "RRTstarkConfigDefault" (config/ompl_planning.yaml) = optimizing
        # planner, not yet verified live and probably slower
        self.declare_parameter("planner_id", "")
        self.declare_parameter("head_yaw", float("nan"))   # nan = -0.35 (right) / +0.35 (left)
        # a lower head put the book in front of the chest and the torso hid the lower half of the cover (barcode)
        self.declare_parameter("head_pitch", -0.10)
        # weight asking the top of the book to match the top of the image (0 = free in-plane rotation, whose
        # 30-40 deg tilt made pyzbar fail; 2.0 gives ~9 deg of tilt)
        self.declare_parameter("head_isbn_w_upright", 2.0)
        self.declare_parameter("head_isbn_wait_s", 120.0)  # frame wait (low RTF)
        # the gripper +Y side is the back cover: only that one is photographed (max 3 shots: centre,
        # bottom, top); True = if unread, turn the book and retry
        self.declare_parameter("head_isbn_both_sides", True)
        self.declare_parameter("grasp_depth", 0.025)   # how far past the spine
        self.declare_parameter("approach_back", 0.08)  # pre-grasp: this far back
        self.declare_parameter("retreat", 0.14)
        # IK error above which the waist yaw is freed too (the right shoulder does not reach the body centre)
        self.declare_parameter("yaw_retry_mm", 5.0)
        # Perception grasp: target = object id in the detections JSON (written by library_manager_node after
        # the 'shelf' trigger), 'auto' = first measured book, or a title word. Spine, thickness, height and
        # free space come from the shelf_camera depth; attach goes through the GraspManager.
        # dynamic_typing: -p target:=3 arrives as INTEGER, 'auto' as STRING
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter("target", "", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("detections_file", "/tmp/x2_detections.json")
        # 'contact' = close slowly until the finger sensor touches, 'fixed' = position from the thickness,
        # 'auto' = contact
        self.declare_parameter("close_mode", "auto")
        self.declare_parameter("close_speed", 0.005)   # m/s per finger (sim time)
        # world x/y of the TCP at the table release; default = point under the table_camera
        # (library_pipeline.py parks objects elsewhere)
        self.declare_parameter("release_x", ROBOT_SPAWN_X + TABLE_RELEASE_X_OFFSET)
        self.declare_parameter("release_y", TABLE_RELEASE_Y)

        self.book = self.get_parameter("book").value
        self.target = str(self.get_parameter("target").value).strip()
        mode = str(self.get_parameter("close_mode").value)
        # 'auto' = always by contact: a fixed close on the collision width makes no contact when the
        # book is not perfectly centred
        self.contact_close = (mode == "contact") or (mode == "auto")
        self.close_speed = max(0.001, float(self.get_parameter("close_speed").value))
        self._contact_model = None
        self._finger_pos = None
        self.robot_x = float(self.get_parameter("robot_x").value)
        self.robot_y = float(self.get_parameter("robot_y").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.head_isbn = bool(self.get_parameter("head_isbn").value)
        self.put_back = bool(self.get_parameter("put_back").value)
        self._head_image = None
        if self.head_isbn:
            self.head_kin = ArmKinematics.from_package(tip="rgbd_head_front_link")
            self.head_client = ActionClient(self, FollowJointTrajectory,
                                            "/head_controller/follow_joint_trajectory")
            self.head_trigger_pub = self.create_publisher(Bool, "/head_camera/trigger", 10)
            self.create_subscription(Image, "/head_camera/image", self._head_image_cb, 1)

        self.waist_client = ActionClient(self, FollowJointTrajectory,
                                         "/waist_controller/follow_joint_trajectory")
        # current state, to build the 7-joint messages
        self._arm_now = [0.0] * 5
        self._grip_now = FINGER_MIN       # never 0.0 (see FINGER_MIN)

        scene = self.get_parameter("scene").value
        self.scene = scene
        # real Gazebo names of the books (ground truth from book_placer): safety stop if perception
        # classifies a book as a "decoration" (it would go to the table instead of the ISBN reading)
        self._known_book_names = {e[0] for e in test_entities(scene) if e[2] == "book"}
        if self.target:
            self._load_target(self.target, str(self.get_parameter("detections_file").value))
            self._pick_side()
            self.attach_pub = self.create_publisher(String, f"/gripper/{self.side}/attach", 10)
            self.detach_pub = self.create_publisher(Empty, f"/gripper/{self.side}/detach", 10)
        else:
            entities = test_entities(scene)
            entry = next((e for e in entities if e[0].endswith(self.book) or e[1].startswith(self.book)), None)
            if entry is None:
                raise RuntimeError(f"'{self.book}' not in scene {scene}: {[e[0] for e in entities]}")
            self.entity, self.key, self.kind, self.bx, self.by = entry
            sx, sy, sz = catalog_entry(self.kind, self.key)["size"]
            self.spine_x = self.bx - sx / 2.0
            self.thick_mesh = sy                                   # for the approach opening
            self.thick_close = collision_size(self.kind, self.key)[1]   # to close on (fixed)
            self.height = sz
            self.length = sx
            self.z_center = SHELF_SURFACES_Z[3] + sz / 2.0 + 0.002
            self.free = free_space_sides(self.entity, self.scene)
            self._pick_side()
            sfx = "" if self.side == "right" else "_left"
            self.attach_pub = self.create_publisher(Empty, f"/{self.entity}/attach{sfx}", 10)
            self.detach_pub = self.create_publisher(Empty, f"/{self.entity}/detach{sfx}", 10)
        self.create_subscription(Contacts, f"/contact_{self.side}_tcp", self._contact_cb, 10)
        # both fingers: closing stops at the FIRST contact, so the finger without sensor does not push the
        # book against its neighbour
        self.create_subscription(Contacts, f"/contact_{self.side}_tcp_b", self._contact_cb, 10)
        # GraspManager auto-attach is OFF while this node works (a finger touching a neighbour would attach
        # it too); attach/detach here are explicit. Turned back on in main().
        self.auto_attach_pub = self.create_publisher(Bool, "/gripper/auto_attach", 10)
        self.detach_all_pub = self.create_publisher(Empty, f"/gripper/{self.side}/detach", 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.mi = MoveItBridge(self, lambda: getattr(self, "_js", {}),
                              planner_id=str(self.get_parameter("planner_id").value))
        self.use_moveit = False
        if bool(self.get_parameter("moveit").value) and not self.dry_run:
            # wait up to 90 s (slow DDS discovery on a loaded machine) and STOP if MoveIt is missing,
            # instead of moving the robot without collision checking
            self.use_moveit = self.mi.available(90.0)
            if self.use_moveit:
                self.get_logger().info("MoveIt: move_group found - free motions planned with collisions, "
                                       "straight paths verified")
            else:
                self.get_logger().error("MoveIt: /move_action or /check_state_validity missing after 90 s "
                                        "(ros2 launch agibot_x2_pkg moveit.launch.py). Not moving the robot without "
                                        "collision checking: stopping (for the old direct behaviour: "
                                        "-p moveit:=false)")
                raise SystemExit(1)

    def _pick_side(self):
        """
        Choose the arm (self.side) and create kinematics, action client and joint lists for that side.
        Call it once self.by is known.
        """
        arm = str(self.get_parameter("arm").value).strip().lower()
        if arm not in ("right", "left", "auto"):
            raise RuntimeError(f"arm='{arm}': use right, left or auto")
        # auto: left only if the object is CLEARLY on the left; near the centre the left arm reaches the grasp
        # point but has no straight approach, the right one with the waist rotated does
        if arm == "auto":
            # the measured reachability map (reach_map) first, then the y threshold: for some slots only
            # the arm opposite to the threshold's choice reaches
            try:
                from agibot_x2_pkg_py.reach_map import arms as _reach_arms
                pref = _reach_arms(int(str(self.entity)[3:]), self.by)
            except Exception:
                pref = None
            self.side = "left" if pref == "L" else "right" if pref == "R" else \
                ("left" if self.by > ARM_AUTO_LEFT_Y else "right")
        else:
            self.side = arm
        self.kin = ArmKinematics.from_package(side=self.side)
        self.GJ = [f"{self.side}_gripper_left_finger_joint", f"{self.side}_gripper_right_finger_joint"]
        self.ARM7 = list(self.kin.arm_joints) + self.GJ
        # waist towards the table: -90 deg with the right arm; the left hand is on the other side of the
        # torso and needs more (the release IK is free in yaw anyway if this seed is not enough)
        self.drop_waist = list(DROP_WAIST) if self.side == "right" else [-2.60, DROP_WAIST[1]]
        hy = float(self.get_parameter("head_yaw").value)
        self.head_yaw = hy if hy == hy else (-0.35 if self.side == "right" else 0.35)
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       f"/{self.side}_arm_controller/follow_joint_trajectory")
        self.get_logger().info(f"Arm: {self.side.upper()} (arm={arm}, y={self.by:+.3f})")

    def _load_target(self, target: str, path: str):
        """
        Load the target object (id, 'auto' or title word) and its measurements from the detections JSON
        """
        import json
        try:
            with open(path, encoding="utf-8") as f:
                dets = json.load(f)
        except Exception as e:
            raise RuntimeError(f"target '{target}': cannot read {path} ({e}). Run the 'shelf' trigger "
                               "of library_manager_node first (detector:=depth or sam3).")
        measured = [d for d in dets if d.get("thickness_m", 0) > 0]
        if not measured:
            raise RuntimeError(f"{path}: no object with 3D measurements (thickness_m): is the shelf_camera "
                               "rgbd and did the depth arrive? (log 'dorso x=...')")
        if target == "auto":
            books = [d for d in measured if d.get("is_book")]
            d = (books or measured)[0]
        elif target.isdigit():
            d = next((x for x in measured if int(x["id"]) == int(target)), None)
        else:
            d = next((x for x in measured if target.lower() in str(x.get("title", "")).lower()), None)
        if d is None:
            raise RuntimeError(f"target '{target}' not found among {[(x['id'], x.get('title')) for x in measured]}")
        self.entity = f"obj{d['id']}"
        self.key = d.get("title") or d.get("class", "?")
        self.kind = "book" if d.get("is_book") else "decoration"
        self.spine_x = float(d["world_x"])
        self.by = float(d["world_y"])
        self.bx = self.spine_x          # book depth unknown: not needed
        self.thick_mesh = float(d["thickness_m"])
        # in Gazebo the book collision is narrower than the mesh (BOOK_COLLISION_SIDE_MARGIN per side):
        # fallback for the 'fixed' close, not needed with 'contact'
        self.thick_close = self.thick_mesh - (2 * BOOK_COLLISION_SIDE_MARGIN if self.kind == "book" else 0.0)
        self.height = float(d["height_m"])
        self.width_profile = d.get("width_profile") or []     # [(z, width)]
        # spine->fore-edge depth: measured from the top face if visible, else 0.16 m (conservative: longer exit)
        self.length = float(d.get("length_m") or 0.0) or 0.16
        self.z_center = (float(d["z_bottom"]) + float(d["z_top"])) / 2.0
        self.free = (float(d.get("free_plus_m", 0.0)), float(d.get("free_minus_m", 0.0)))
        self.get_logger().info(
            f"Target from JSON: obj {d['id']} ({self.kind}, '{d.get('title','')}' {d.get('color','')}): "
            f"spine x={self.spine_x:.3f} y={self.by:.3f} z={self.z_center:.3f}, thickness "
            f"{self.thick_mesh*1000:.0f} mm, height {self.height*1000:.0f} mm, depth {self.length*1000:.0f} mm, free "
            f"+{self.free[0]*1000:.0f}/-{self.free[1]*1000:.0f} mm")

    # --- contacts / finger state ---

    def _contact_cb(self, msg):
        """
        Remember the scene model touched by a finger sensor (robot, furniture and ground ignored)
        """
        for c in msg.contacts:
            for ent in (c.collision1, c.collision2):
                model = ent.name.split("::")[0]
                if model and model not in ("mogi_arm", "bookshelf", "table", "ground_plane", "video_camera"):
                    self._contact_model = model
                    return

    def _js_cb(self, msg):
        """
        Store the latest joint states and the finger position
        """
        if self.GJ[0] in msg.name:
            self._finger_pos = msg.position[msg.name.index(self.GJ[0])]
        self._js = dict(zip(msg.name, msg.position))

    def _wait_converged(self, names, targets, label, tol=0.012, timeout_s=120.0):
        """
        Wait until the real joints (/joint_states) are within `tol` rad of the command.
        The JTC reports "goal reached" when the trajectory time ends even if joints lag by tens of degrees;
        at RTF 0.05 one second of settling is 20 s real time, hence the long timeout.
        """
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
        self.get_logger().warn(f"{label}: joints not settled after {timeout_s:.0f} s "
                               f"({worst_name} {math.degrees(worst or 0):.1f} degrees from the command)")
        return False

    def close_until_contact(self, p_from, label):
        """
        Close the fingers at close_speed until a finger sensor touches a scene object, then cancel the goal
        (the JTC holds the reached position). Return the finger position, or None.
        """
        T = max(0.5, p_from / self.close_speed)
        self.get_logger().info(f"{label}: contact close from {p_from*1000:.1f} mm/finger, "
                               f"{self.close_speed*1000:.0f} mm/s ({T:.0f} s sim)")
        if self.dry_run:
            self._grip_now = grasp_opening(self.thick_close)
            return self._grip_now
        self._contact_model = None
        # diagnostics: where the gripper REALLY is (FK of /joint_states) versus the planned grasp point
        try:
            js = self._js
            q_real = [float(js[j]) for j in self.kin.arm_joints]
            w_real = (float(js["waist_yaw_joint"]), float(js["waist_pitch_joint"]))
            p_real, _R = self.kin.fk_arm(q_real, w_real)
            p_real = p_real + np.array([self.robot_x, self.robot_y, 0.0])
            g_plan = getattr(self, "_grasp_point_world", None)
            msg = f"{label}: TCP reale a {np.round(p_real, 3).tolist()}"
            if g_plan is not None:
                msg += f", pianificato {np.round(g_plan, 3).tolist()}, scarto {np.linalg.norm(p_real - g_plan)*1000:.0f} mm"
            self.get_logger().info(msg)
        except Exception as e:
            self.get_logger().warn(f"{label}: real FK not available ({e})")
        full = list(self._arm_now) + [0.0, 0.0]
        goal = self._goal(self.ARM7, [full], T)
        fut = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return None
        res = handle.get_result_async()
        t0 = time.time()
        while not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._contact_model is not None:
                handle.cancel_goal_async()
                break
            if time.time() - t0 > 20 * T + 30:      # very low RTF: do not hang
                break
        if self._contact_model is None:
            self.get_logger().error(f"{label}: fingers closed without contact - object not between the fingers")
            return None
        # let it settle and read where the finger stopped
        t1 = time.time()
        while time.time() - t1 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        pos = self._finger_pos if self._finger_pos is not None else self._grip_now
        self._grip_now = float(pos)
        self.get_logger().info(f"{label}: contact with {self._contact_model} at {pos*1000:.1f} mm/finger")
        if self.kind != "book":
            # stopping at the first contact on a round surface (fingers tangent to a sphere) gives almost no
            # grip force and the object spins in the hand; books have flat faces, decorations get 3 mm more
            squeeze = max(FINGER_MIN, pos - 0.003)
            if self.gripper(squeeze, 1.0, f"{label} (stringo altri 3 mm per una presa salda)"):
                self._grip_now = squeeze
        return self._grip_now

    def _attach(self):
        """
        Attach the held object in Gazebo (GraspManager by name, or the object's DetachableJoint)
        """
        self._held_model = self._contact_model      # the held model no longer changes with later contacts
        if self.attach_pub.msg_type is String:
            # GraspManager: name of the entity touched while closing (with "" it looked for a contact in the
            # last 2 REAL seconds, already gone at RTF 0.05, and ignored the request)
            self.attach_pub.publish(String(data=str(self._contact_model or "")))
        else:
            self.attach_pub.publish(Empty())

    # --- trajectory execution ---
    def _walk_to_base_x(self, target_x, label):
        """
        Real walk (animated legs) to base_x_joint = target_x by running walk_to_shelf as a subprocess.
        Safe with the object in hand: arm_swing is off and walk_to_shelf never swings an attached arm.
        """
        import subprocess
        r = subprocess.run(
            ["ros2", "run", "agibot_x2_pkg_py", "walk_to_shelf", "--ros-args",
             "-p", f"distance:={target_x}", "-p", "arm_swing:=false",
             # 15 cm is less than a default stride (0.27 m) and the legs barely move: 0.09 m stride
             # (speed 0.15) and a higher foot give ~2 real steps
             "-p", "speed:=0.15", "-p", "step_height:=0.06"],
            capture_output=True, text=True)
        ok = r.returncode == 0
        self.get_logger().info(f"{label}: {'ok' if ok else 'FAILED'} (walk_to_shelf, base_x -> {target_x:.3f})")
        if not ok:
            self.get_logger().warn(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr.strip())
        # self._js still holds the pre-walk value (updated only when this node spins): wait for fresh
        # /joint_states, otherwise _robot_x_from_joints() would use the old position
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self._robot_x_from_joints()
        return ok

    def _step_back(self, dx=0.15):
        """
        Walk back dx before the big jump to the release, to move the freshly pulled-out object away from the shelf edge
        """
        t0 = time.time()
        while "base_x_joint" not in getattr(self, "_js", {}) and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        js = getattr(self, "_js", {})
        if "base_x_joint" not in js:
            self.get_logger().warn("step back: base_x_joint not received yet, skipping the step")
            return False
        x0 = float(js.get("base_x_joint", 0.0))
        # restored at the end (_release_and_home): the pipeline does not walk between objects, so the next
        # object would otherwise be approached 15 cm too far from the shelf
        self._pre_step_back_x = x0
        return self._walk_to_base_x(x0 - dx, f"passo indietro di {dx*100:.0f} cm")

    def _step_forward_undo(self):
        """
        Walk back to the base_x saved by _step_back(), so the next object is approached from the usual distance
        """
        x0 = getattr(self, "_pre_step_back_x", None)
        if x0 is None:
            return True
        return self._walk_to_base_x(x0, "passo avanti (torno alla posa di lavoro)")


    def _move(self, client, names, positions, duration, label):
        """
        Send a joint trajectory and wait for it; positions is one configuration or a LIST of them
        (waypoints evenly spaced up to `duration`). Fingers are kept >= FINGER_MIN, joints inside their limits.
        """
        waypoints = positions if isinstance(positions[0], (list, tuple, np.ndarray)) else [positions]
        self.get_logger().info(
            f"{label}: {len(waypoints)} waypoints, last {np.round(waypoints[-1], 3).tolist()} ({duration:.1f}s)")
        if self.dry_run:
            return True
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"{label}: action server not available")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        fingers = [k for k, n in enumerate(names) if n in self.GJ]
        lims = [self.kin.limits.get(n) for n in names]    # already narrowed by JOINT_LIMIT_MARGIN
        for i, wp in enumerate(waypoints, start=1):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in wp]
            for k in fingers:                     # never at the end stop (FINGER_MIN)
                pt.positions[k] = max(pt.positions[k], FINGER_MIN)
            for k, lim in enumerate(lims):        # never at the limit of ANY joint
                if lim is not None and k not in fingers:
                    pt.positions[k] = min(max(pt.positions[k], lim[0]), lim[1])
            t_i = duration * i / len(waypoints)
            pt.time_from_start = Duration(sec=int(t_i), nanosec=int((t_i % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        self._wait_converged(names, waypoints[-1], label)
        return True

    def _goal(self, names, waypoints, duration):
        """
        Build a FollowJointTrajectory goal with waypoints evenly spaced up to `duration`
        """
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
        """
        Move arm (7 joints) and waist (2) together, waypoints paired at the same instants: this keeps the
        object on a straight line while the torso rotates/tilts, with the gripper aligned.
        With MoveIt the path is checked against the planning scene first. Fails if the hand does not arrive.
        """
        arm_full = [list(wp) + [self._grip_now, self._grip_now] for wp in arm_wps]
        try:   # diagnostics: where each planned waypoint REALLY leads
            off = np.array([self.robot_x, self.robot_y, 0.0])
            pts = [np.round(self.kin.fk_arm(list(q)[:5], tuple(w))[0] + off, 3).tolist()
                   for q, w in zip(arm_wps, waist_wps)]
            self.get_logger().info(f"{label}: waypoint FK (world): {pts[0]} ... {pts[-1]}")
        except Exception as e:
            self.get_logger().warn(f"{label}: waypoint FK not available ({e})")
        self.get_logger().info(
            f"{label}: {len(arm_wps)} arm+waist waypoints, final waist "
            f"{np.round(waist_wps[-1], 2).tolist()} ({duration:.1f}s)")
        if self.dry_run:
            self._arm_now = list(arm_wps[-1])
            return True
        if self.use_moveit:
            names = list(WAIST_JOINTS) + list(self.kin.arm_joints)
            wps = [[float(v) for v in w] + [float(v) for v in list(q)[:5]] for q, w in zip(arm_wps, waist_wps)]
            ok_c, _problems = self.mi.check_path(names, wps, f"{self.side}_arm", label)
            if ok_c is False:
                self.get_logger().error(
                    f"{label}: the straight path touches the planning scene: stopping without executing it "
                    "(see the colliding pairs above; no margin was relaxed)")
                return False
        for c, n in ((self.arm_client, "braccio"), (self.waist_client, "vita")):
            if not c.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f"{label}: action server {n} not available")
                return False
        f_arm = self.arm_client.send_goal_async(self._goal(self.ARM7, arm_full, duration))
        f_waist = self.waist_client.send_goal_async(self._goal(WAIST_JOINTS, waist_wps, duration))
        rclpy.spin_until_future_complete(self, f_arm)
        rclpy.spin_until_future_complete(self, f_waist)
        handles = [f_arm.result(), f_waist.result()]
        if any(h is None or not h.accepted for h in handles):
            self.get_logger().error(f"{label}: goal rejected")
            return False
        results = [h.get_result_async() for h in handles]
        for r in results:
            rclpy.spin_until_future_complete(self, r)
        # settling timeout proportional to the duration; if the joints do not arrive the step FAILS
        # (going on would e.g. detach the book in mid-air)
        t_wait = max(120.0, 90.0 * float(duration))
        ok_a = self._wait_converged(self.ARM7, arm_full[-1], label, timeout_s=t_wait)
        ok_w = self._wait_converged(WAIST_JOINTS, waist_wps[-1], label, timeout_s=t_wait)
        self._arm_now = list(arm_wps[-1])
        if not (ok_a and ok_w) and not self.dry_run:
            # the joint tolerance (0.7 deg) is tighter than physics holds at steady state: what matters is the
            # HAND pose, accepted if the TCP is within 10 mm and no joint is over 3 degrees off
            try:
                js = self._js
                q_m = [float(js[j]) for j in self.kin.arm_joints]
                w_m = (float(js[WAIST_JOINTS[0]]), float(js[WAIST_JOINTS[1]]))
                p_m, _r = self.kin.fk_arm(q_m, w_m)
                p_c, _r = self.kin.fk_arm(list(arm_wps[-1]), tuple(waist_wps[-1]))
                tcp_err = float(np.linalg.norm(np.asarray(p_m) - np.asarray(p_c)))
                j_err = max([abs(a - b) for a, b in zip(q_m, arm_wps[-1])] +
                            [abs(a - b) for a, b in zip(w_m, waist_wps[-1])])
                if tcp_err <= 0.010 and j_err <= math.radians(3.0):
                    self.get_logger().warn(f"{label}: joints {math.degrees(j_err):.1f} degrees from the command but the "
                                           f"hand is at {tcp_err*1000:.1f} mm: accepted")
                    return True
                self.get_logger().error(f"{label}: hand at {tcp_err*1000:.1f} mm, worst joint "
                                        f"{math.degrees(j_err):.1f} degrees off")
            except Exception as e:
                self.get_logger().error(f"{label}: real pose check failed ({e})")
        if not (ok_a and ok_w):
            self.get_logger().error(f"{label}: the joints did not reach the command: the step did NOT succeed")
            return False
        return True

    def _safe_home(self, why):
        """
        Fold the arm, then waist and arm home (with MoveIt: planned with collisions; else step by step).
        Fingers untouched.
        """
        self.get_logger().warn(f"{why}: bringing arm and waist home")
        if self.use_moveit:
            if self.arm(CARRY_ARM, 2.5, "ritorno: braccio raccolto") and \
               self.goto(HOME_ARM, HOME_WAIST, "ritorno: a casa (MoveIt)"):
                return True
            self.get_logger().error("return with MoveIt failed: trying the direct motions")
            self.use_moveit = False
        self.arm(CARRY_ARM, 2.5, "ritorno: braccio raccolto")
        self.waist(HOME_WAIST, 3.0, "ritorno: vita a casa")
        self.arm(HOME_ARM, 2.5, "ritorno: braccio a casa")
        return True

    def to_release(self, q_rel, w_rel, label):
        """
        Go to the release point with the object in hand using TWO MoveIt plans, because the single arm+waist
        plan for this big joint-space jump kept failing: 1) waist to w_rel with the arm at CARRY,
        2) arm to q_rel with the waist still (arm-only group, fewer variables for OMPL).
        If step 1 fails, the direct jump is tried as a last resort.
        """
        if not self.goto(CARRY_ARM, list(w_rel), f"{label} (1: vita, braccio a CARRY)"):
            self.get_logger().warn(f"{label}: the waist alone does not reach the release, trying the direct jump")
            return self.goto(list(q_rel), list(w_rel), f"{label} (salto diretto)")
        names = list(self.kin.arm_joints)
        vals = [float(v) for v in q_rel]
        self._nudge_off_limits()
        if not self.mi.move_joints(f"{self.side}_arm_only", names, vals, f"{label} (2: braccio, vita ferma)"):
            # OMPL is randomized: a generic FAILURE may succeed on the same problem at the next attempt
            self.get_logger().warn(f"{label}: planning failed, retrying the same plan once")
            if not self.mi.move_joints(f"{self.side}_arm_only", names, vals, f"{label} (2o tentativo)"):
                # a goal in marginal contact with an already placed object makes OMPL reject every goal
                # sample: +-0.02 rad tolerance on the GOAL joints only, collisions are checked as always
                self.get_logger().warn(f"{label}: failed again, last attempt with joint tolerance 0.02 rad "
                                       "(not a collision margin: only the end point may shift ~1 cm)")
                if not self.mi.move_joints(f"{self.side}_arm_only", names, vals, f"{label} (3o tentativo, tol 0.02)",
                                           tol=0.02):
                    return False
        self._arm_now = list(vals)
        if not self._wait_converged(names, vals, f"{label} (2: braccio, vita ferma)"):
            self.get_logger().warn(f"{label}: tracking failed, retrying the same plan once")
            if not self.mi.move_joints(f"{self.side}_arm_only", names, vals, f"{label} (2, 2o tentativo)"):
                return False
            if not self._wait_converged(names, vals, f"{label} (2, 2o tentativo)"):
                return False
        return True

    def to_pregrasp(self, q_pre, w_pre, p_pre_rel):
        """
        3. Arm to the pre-grasp. With MoveIt: the pre-grasp lies INSIDE the shelf between the neighbours (a
        narrow passage OMPL does not solve in time), so MoveIt first plans to a staging pose STAGE_BACK in
        front of it, then the last straight segment is computed here and checked against the scene.
        """
        if not self.use_moveit:
            return self.arm(q_pre, 3.0, "3. braccio pre-grasp")
        p_stage = np.asarray(p_pre_rel) - np.array([STAGE_BACK, 0.0, 0.0])
        # restarts=0: random restarts may choose a different kinematic branch from q_pre; the staging pose is a
        # 12 cm perturbation of a valid pre-grasp, continuity with q_pre matters more
        q_stage, _w, e_stage = self.kin.ik(p_stage, q0=list(q_pre), waist=tuple(w_pre), restarts=0)
        if e_stage > 0.01:
            self.get_logger().warn(f"3a. staging pose not reachable ({e_stage*1000:.0f} mm): direct pre-grasp")
            return self.arm(q_pre, 3.0, "3. braccio pre-grasp")
        if not self.arm(list(q_stage), 3.0, f"3a. posa di sosta davanti allo scaffale ({STAGE_BACK*100:.0f} cm prima del pre-grasp)"):
            return False
        n = 4
        line = [list(np.asarray(q_stage) + (np.asarray(q_pre) - np.asarray(q_stage)) * k / n) for k in range(1, n + 1)]
        # joint interpolation between two poses on the same line: checked waypoint by waypoint (move_both)
        return self.move_both(line, [list(w_pre)] * n, 3.0, "3b. sosta -> pre-grasp (rettilineo, verificato)")

    def _nudge_off_limits(self):
        """
        Move joints within 3 mrad of a URDF limit 10 mrad inside with a direct goal, before any plan:
        MoveIt rejects a start state exactly at the limit (START_STATE_INVALID) and fix_start_state is not enough.
        """
        js = getattr(self, "_js", {})
        if not js:
            return
        tol, step = 0.003, 0.01
        arm_now = [float(js.get(j, 0.0)) for j in self.kin.arm_joints]
        arm_new = list(arm_now)
        moved = []
        for i, j in enumerate(self.kin.arm_joints):
            lo, hi = self.kin.limits.get(j, (-1e9, 1e9))
            if arm_now[i] <= lo + tol:
                arm_new[i] = lo + step; moved.append(j)
            elif arm_now[i] >= hi - tol:
                arm_new[i] = hi - step; moved.append(j)
        w_now = [float(js.get(j, 0.0)) for j in WAIST_JOINTS]
        w_new = list(w_now)
        for i, j in enumerate(WAIST_JOINTS):
            lo, hi = self.kin.limits.get(j, (-1e9, 1e9))
            if w_now[i] <= lo + tol:
                w_new[i] = lo + step; moved.append(j)
            elif w_now[i] >= hi - tol:
                w_new[i] = hi - step; moved.append(j)
        if not moved:
            return
        self.get_logger().info(f"joints at their limit ({moved}): moving them 10 mrad inside before planning")
        if arm_new != arm_now:
            self._move(self.arm_client, self.ARM7, arm_new + [self._grip_now, self._grip_now], 1.0, "fine corsa: braccio")
            self._arm_now = list(arm_new)
        if w_new != w_now:
            self._move(self.waist_client, WAIST_JOINTS, w_new, 1.0, "fine corsa: vita")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def goto(self, q_arm, w, label):
        """
        Arm + waist to (q_arm, w): with MoveIt one plan on the <side>_arm group (collisions with the scene and
        the held object); without MoveIt, waist then arm
        """
        if self.use_moveit:
            self._nudge_off_limits()
            names = list(WAIST_JOINTS) + list(self.kin.arm_joints)
            vals = [float(v) for v in w] + [float(v) for v in q_arm]
            if not self.mi.move_joints(f"{self.side}_arm", names, vals, label):
                return False
            self._arm_now = list(q_arm)
            # MoveIt/the controller may report success while a real joint stays tens of degrees off:
            # a tracking failure is a real failure, with one retry (it may be just the momentary load)
            if not self._wait_converged(names, vals, label):
                self.get_logger().warn(f"{label}: tracking failed, retrying the same plan once")
                if not self.mi.move_joints(f"{self.side}_arm", names, vals, label + " (2o tentativo)"):
                    return False
                if not self._wait_converged(names, vals, label + " (2o tentativo)"):
                    return False
            return True
        return self.waist(list(w), 3.0, label + " (vita)") and self.arm(list(q_arm), 3.0, label + " (braccio)")

    def arm(self, q, duration, label):
        """
        Move the arm to q (5 joints) or through a list of configurations; the fingers keep the current
        opening (_grip_now). A single target goes through MoveIt when enabled.
        """
        waypoints = q if isinstance(q[0], (list, tuple, np.ndarray)) else [q]
        if self.use_moveit and len(waypoints) == 1 and not self.dry_run:
            self._nudge_off_limits()
            names = list(self.kin.arm_joints)
            vals = [float(v) for v in waypoints[0]]
            if not self.mi.move_joints(f"{self.side}_arm_only", names, vals, label):
                return False
            self._arm_now = list(vals)
            if not self._wait_converged(names, vals, label):
                return False
            return True
        full = [list(wp) + [self._grip_now, self._grip_now] for wp in waypoints]
        ok = self._move(self.arm_client, self.ARM7, full, duration, label)
        if ok:
            self._arm_now = list(waypoints[-1])
        return ok

    def waist(self, q, duration, label):
        """
        Move the waist to q (through MoveIt when enabled) or through a list of configurations
        """
        if self.use_moveit and not isinstance(q[0], (list, tuple, np.ndarray)) and not self.dry_run:
            self._nudge_off_limits()
            vals = [float(v) for v in q]
            if not self.mi.move_joints("waist", list(WAIST_JOINTS), vals, label):
                return False
            if not self._wait_converged(list(WAIST_JOINTS), vals, label):
                return False
            return True
        return self._move(self.waist_client, WAIST_JOINTS, q, duration, label)

    def gripper(self, opening, duration, label):
        """
        Move only the fingers, keeping the arm where it is (_arm_now)
        """
        full = list(self._arm_now) + [opening, opening]
        ok = self._move(self.arm_client, self.ARM7, full, duration, label)
        if ok:
            self._grip_now = float(opening)
        return ok

    # --- ISBN from the head camera ---

    def _head_image_cb(self, msg):
        """
        Store the latest head camera frame
        """
        self._head_image = msg

    def head(self, q, duration, label):
        """
        Move the head joints (yaw, pitch)
        """
        return self._move(self.head_client, HEAD_JOINTS, q, duration, label)

    def head_camera(self, waist, head_q):
        """
        Position and axes (optical, image up, image right) of the head camera in the robot frame
        for the given waist and head
        """
        q = {WAIST_JOINTS[0]: float(waist[0]), WAIST_JOINTS[1]: float(waist[1]),
             HEAD_JOINTS[0]: float(head_q[0]), HEAD_JOINTS[1]: float(head_q[1])}
        p, R = self.head_kin.fk_link(q)
        Rc = R @ HEAD_SENSOR_R
        return p, Rc[:, 0], Rc[:, 2], -Rc[:, 1]

    def plan_show(self, waist, head_q, q_seed=None):
        """
        Arm pose bringing the book cover in front of the head camera (cover normal, gripper +-Y, towards it).
        Searches the distance (from head_isbn_dist up) where the IK converges, the whole book is in the frame
        and its corners stay away from the torso. Return (q_A, q_B, dist), q_B showing the other cover, or None.
        """
        p, o, up, right = self.head_camera(waist, head_q)
        n = -o
        sx, sz = self.length, self.height
        half_h = HEAD_ISBN_HFOV / 2.0 * HEAD_ISBN_FOV_MARGIN
        half_v = math.atan(HEAD_ISBN_ASPECT * math.tan(HEAD_ISBN_HFOV / 2.0)) * HEAD_ISBN_FOV_MARGIN + HEAD_ISBN_PITCH_SWEEP
        d0 = float(self.get_parameter("head_isbn_dist").value)
        book_up_g = getattr(self, "_book_up_g", np.array([1.0, 0.0, 0.0]))
        book_dz = getattr(self, "_book_dz", 0.0)
        book_depth_g = getattr(self, "_book_depth_g", np.array([0.0, 0.0, -1.0]))
        gd = float(self.get_parameter("grasp_depth").value)
        w_upr = float(self.get_parameter("head_isbn_w_upright").value)
        for d in [d0 + 0.025 * k for k in range(5)]:
            c = p + d * o
            # top of the book = top of the image: gripper X = up_cam (or its opposite if X points down in this
            # branch), Y = n, so Z = X x Y and the approach (-Z) = n x X
            x_des = up * (1.0 if book_up_g[0] >= 0 else -1.0)
            a_des = np.cross(n, x_des); a_des /= np.linalg.norm(a_des)
            a = a_des.copy()
            up_b = np.array([0.0, 0.0, 1.0]); up_b -= (up_b @ o) * o; up_b /= np.linalg.norm(up_b)
            # seed = current arm pose: from the just-extracted book the arm goes STRAIGHT in front of the camera
            # (the wrist turns the back cover to it), without folding or crossing in front of the chest
            q0 = list(q_seed) if q_seed is not None else list(CARRY_ARM)
            q = None
            for it in range(2):          # the TCP depends on the rotation found: 2 passes
                tcp = c - a * (sx / 2.0 - gd) - up_b * book_dz
                q, wq, e = self.kin.ik(tcp, approach=a_des, finger_axis=n, waist=tuple(waist),
                                       optimize_waist=False, w_finger=20.0, w_approach=w_upr,
                                       finger_signed=True, q0=q0, restarts=6 if it == 0 else 0)
                _pos, R = self.kin.fk_arm(q, tuple(wq))
                a = R @ book_depth_g
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
            ok = e < 0.012 and ang_n < 20.0 and in_fov and torso > HEAD_ISBN_TORSO_CLEAR and zmin > TABLE_TOP_Z + 0.05
            self.get_logger().info(
                f"  show to head d={d:.3f}: IK {e*1000:.1f} mm, cover->camera {ang_n:.0f} degrees, "
                f"book tilted {tilt:.0f} degrees, in frame {in_fov}, corners {torso:.2f} m from the torso, z min {zmin:.2f}"
                + (" -> OK" if ok else ""))
            if not ok:
                continue
            # other cover: wrist turned by 180 degrees (same TCP pose)
            lo, hi = self.kin.limits[self.kin.arm_joints[4]]
            q_b = None
            for dw in (math.pi, -math.pi):
                if lo <= q[4] + dw <= hi:
                    q_b = list(q); q_b[4] = q[4] + dw
                    break
            if q_b is None:
                q_b, _w, e_b = self.kin.ik(tcp, approach=-R[:, 2], finger_axis=-n, waist=tuple(waist),
                                           optimize_waist=False, w_finger=20.0, w_approach=0.0,
                                           finger_signed=True, q0=q, restarts=6)
                if e_b > 0.01:
                    q_b = None
            return list(q), q_b, d
        return None

    def read_isbn_head(self, label):
        """
        Take a head_camera shot and decode the barcode (book_identification/extract_isbn.py, OCR as fallback).
        Return the metadata dict (at least ISBN-13) or None.
        """
        if self.dry_run:
            self.get_logger().info(f"{label}: dry_run, no shot")
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
                self.get_logger().error(f"{label}: no frame from /head_camera/image in {wait:.0f} s "
                                        "(image_bridge and head_camera sensor in the launch file?)")
                return None
            msg = self._head_image
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == "rgb8":
                img = img[:, :, ::-1]
            lit = float((img.max(axis=2) > 40).mean())
            if lit >= 0.3:
                break
            # the first render of an rgbd sensor has no colour (gz-sensors, see Bugs.md): shoot once more
            self.get_logger().warn(f"{label}: dark frame ({lit*100:.0f}% lit pixels), shooting again")
        self._head_shot = getattr(self, "_head_shot", 0) + 1
        path = f"/tmp/x2_head_isbn_{self.entity}_{self._head_shot}.jpg"
        cv2.imwrite(path, img)
        self.get_logger().info(f"{label}: {msg.width}x{msg.height} photo saved to {path}")
        # book_identification/extract_isbn.py: searched upwards from the cwd (like library_manager_node)
        import importlib, sys, os
        root = os.getcwd()
        while root != os.path.dirname(root) and not os.path.isdir(os.path.join(root, "book_identification")):
            root = os.path.dirname(root)
        sdir = os.path.join(root, "book_identification")
        if not os.path.isdir(sdir):
            self.get_logger().warn(f"{label}: book_identification/ folder not found from the cwd (launch from the repo root)")
            return None
        if sdir not in sys.path:
            sys.path.insert(0, sdir)
        # `ros2 run` uses /usr/bin/python3 (colcon shebang), so an active venv does not count: pyzbar and
        # isbnlib are taken from the repo .venv site-packages added to the path
        import glob
        venv_sp = sorted(glob.glob(os.path.join(root, ".venv", "lib", "python3*", "site-packages")))
        try:
            importlib.import_module("pyzbar")
        except Exception:
            if venv_sp and venv_sp[-1] not in sys.path:
                sys.path.append(venv_sp[-1])
                self.get_logger().info(f"{label}: pyzbar from the repo .venv ({venv_sp[-1]})")
        try:
            ei = importlib.import_module("extract_isbn")
        except Exception as e:
            self.get_logger().error(f"{label}: cannot import extract_isbn ({e}): the .venv needs pyzbar and isbnlib")
            return None
        isbns = []
        try:
            isbns = ei.extract_isbns_from_barcode(path)
        except Exception as e:
            self.get_logger().warn(f"{label}: barcode decoding failed ({e})")
        if not isbns:
            try:
                isbns = ei.extract_isbns_from_ocr(path)
                if isbns:
                    self.get_logger().info(f"{label}: no barcode, ISBN from OCR: {isbns}")
            except Exception as e:
                self.get_logger().info(f"{label}: OCR not available ({e})")
        if not isbns:
            self.get_logger().info(f"{label}: no readable ISBN in {path}")
            return None
        self.get_logger().info(f"{label}: ISBN {isbns}")
        meta = None
        try:
            meta = ei.get_book_info(isbns[0])
        except Exception as e:
            self.get_logger().warn(f"{label}: metadata not retrieved ({e}) - network needed")
        meta = dict(meta or {})
        meta.setdefault("ISBN-13", isbns[0])
        meta["image"] = path
        return meta

    def show_to_head_and_read(self, waist):
        """
        Full head ISBN sequence: show pose (cover A) -> head -> shots -> other cover if needed -> shots.
        Writes /tmp/x2_head_isbn_<entity>.json. The head returns straight; the arm stays in the show pose
        (the caller goes through CARRY_ARM).
        """
        head_q = [self.head_yaw, float(self.get_parameter("head_pitch").value)]
        self.get_logger().info("H0. ISBN from the head: looking for the show pose")
        plan = self.plan_show(waist, head_q, q_seed=list(self._arm_now))
        if plan is None:
            self.get_logger().warn("H0. no valid show pose: skipping the head reading")
            return None
        q_a, q_b, d = plan
        meta = None
        lo_h, hi_h = -0.3838, 0.3838     # head_pitch_joint (x2_hand_gazebo.urdf)

        def aim_head(target):
            """
            (head_yaw, head_pitch) pointing the camera optical axis at `target` (robot frame), waist at `waist`
            """
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
            """
            Up to 3 shots of the same cover, aimed from the real pose of the book in hand: centre (whole
            cover), bottom (where the barcode usually is), top. Stops at the first shot that reads an ISBN.
            """
            pos, R = self.kin.fk_arm(q_side, tuple(waist))
            a_dir = -R[:, 2]
            up = R @ getattr(self, "_book_up_g", np.array([1.0, 0.0, 0.0]))
            c = (pos + a_dir * (self.length / 2.0 - float(self.get_parameter("grasp_depth").value))
                 + up * getattr(self, "_book_dz", 0.0))
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
            # metadata also in the detections JSON: target mode matches "obj<id>", catalogue mode matches the
            # book by position (world_y of the spine)
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
                        self.get_logger().info(f"Metadata also saved to {dp} (obj {hit.get('id')})")
                except Exception as e:
                    self.get_logger().warn(f"detections JSON not updated: {e}")
        if meta:
            self.get_logger().info(f"ISBN from the head: {out['isbn']} '{out['title']}' {out['author']} {out['year']} -> {path}")
        else:
            self.get_logger().warn(f"ISBN from the head: not read (result in {path})")
        return meta

    def _table_plan(self, tx, ty):
        """
        Grasp pose for the object on the table (inverse of the release: fingers along x, approach towards -y
        with the waist at -90 degrees), staging pose 12 cm behind, straight approach and lift paths.
        Return a dict with the IK error `err` (m).
        """
        rel = lambda p: np.asarray(p, dtype=float) - np.array([self.robot_x, self.robot_y, 0.0])
        z_tab = TABLE_TOP_Z + self.height / 2.0 + getattr(self, "_grasp_dz", 0.0)
        p_tab = np.array([tx, ty, z_tab])
        kw = dict(approach=(0, -1, 0), finger_axis=(1, 0, 0))
        q_t, w_t, e_t = self.kin.ik(rel(p_tab), waist=tuple(self.drop_waist), optimize_waist="pitch", **kw)
        if e_t > 0.01:
            q2, w2, e2 = self.kin.ik(rel(p_tab), waist=tuple(self.drop_waist), optimize_waist=True, **kw)
            if e2 < e_t:
                q_t, w_t, e_t = q2, w2, e2
        p_st = p_tab + np.array([0.0, 0.12, 0.0])
        q_s, w_s, e_s = self.kin.ik(rel(p_st), q0=q_t, waist=tuple(w_t), optimize_waist=False, restarts=3, **kw)
        ap, e_ap = self.kin.ik_path(rel(p_st), rel(p_tab), 6, q_s, waist=tuple(w_t), **kw)
        lift_to = p_tab + np.array([0.0, 0.0, 0.08])
        lf, e_lf = self.kin.ik_path(rel(p_tab), rel(lift_to), 5, q_t, waist=tuple(w_t), **kw)
        err = max(e_t, e_s, e_ap, e_lf)
        self.get_logger().info(f"T-. grasp from the table at ({tx:+.3f}, {ty:+.3f}, {z_tab:.3f}): IK errors grasp {e_t*1000:.1f}, "
                               f"staging {e_s*1000:.1f}, approach {e_ap*1000:.1f}, lift {e_lf*1000:.1f} mm "
                               f"(waist {np.round(w_t, 2).tolist()})")
        return dict(q_t=q_t, w_t=list(w_t), q_s=q_s, w_s=list(w_s), ap=ap, lf=lf, err=err)

    def _table_execute(self, tp, q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app):
        """
        Object from the table to the shelf: grasp, attach, lift, then the put-back (put_back_on_shelf)
        """
        w_t = tp["w_t"]
        n = len(tp["ap"])
        # at CARRY with 37 mm fingers the right finger touches the table top (MoveIt start state in collision):
        # 26 mm at CARRY as for books, full opening only in front of the object
        ok = (self.arm(CARRY_ARM, 2.5, "T0. braccio raccolto")
              and self.gripper(0.026, 1.0, "T1. apro le dita a 26 mm/dito")
              and self.goto(tp["q_s"], tp["w_s"], "T2. verso la posa davanti all'oggetto sul tavolo (MoveIt)")
              and self.gripper(GRIPPER_OPEN, 1.0, f"T2b. apro le dita a {GRIPPER_OPEN*1000:.0f} mm/dito")
              and self.move_both(tp["ap"], [list(w_t)] * n, 3.0, "T3. avvicinamento rettilineo all'oggetto"))
        if not ok:
            self.get_logger().error("T0-T3 failed: stopping with the hand empty")
            self._safe_home("presa dal tavolo non riuscita")
            return False
        got = self.close_until_contact(GRIPPER_OPEN, "T4. chiudo le dita sull'oggetto")
        if got is None or self._contact_model is None:
            self.get_logger().error("T4: no contact with the object: opening and going home")
            self.gripper(GRIPPER_OPEN, 1.0, "T4. riapro")
            self._safe_home("oggetto non afferrato")
            return False
        if self._contact_model in self._known_book_names or not str(self._contact_model).startswith("gt_"):
            self.get_logger().error(f"T4: the contact is with '{self._contact_model}', not with an object: opening and going home")
            self.gripper(GRIPPER_OPEN, 1.0, "T4. riapro")
            self._safe_home("contatto sbagliato")
            return False
        self._pause(0.5)
        self.get_logger().info(f"T5. ATTACH of {self._contact_model} ({self.attach_pub.topic_name})")
        self._attach()
        if self.use_moveit and self.target and not self.dry_run:
            attached_ok = self.mi.attach(self.side, self.entity) or self.mi.attach(self.side, self.entity)
            if not attached_ok:
                self.get_logger().error("T5: attach not confirmed in the scene: opening and going home (the object stays on the table)")
                self.detach_pub.publish(Empty())
                self.detach_all_pub.publish(Empty())
                self.gripper(GRIPPER_OPEN, 1.0, "T5. riapro")
                self._safe_home("attach non confermato")
                return False
        self._pause(0.5)
        if not self.move_both(tp["lf"], [list(w_t)] * len(tp["lf"]), 3.0, "T6. sollevo l'oggetto dal tavolo"):
            self.get_logger().error("T6: lift failed: the object STAYS IN HAND, stopping")
            return False
        # MEASURED offset of the object in the gripper: real pose (Gazebo) versus FK TCP along the finger axis
        # (the books' finger-contact formula gives the wrong sign here); the TCP is shifted by the opposite
        self._book_off = np.zeros(3)
        if not self.dry_run:
            pose = self._gz_pose(self._contact_model)
            if pose is not None:
                rel = lambda p: np.asarray(p, dtype=float) - np.array([self.robot_x, self.robot_y, 0.0])
                p_t, R_t = self.kin.fk_arm(list(tp["lf"][-1])[:5], tuple(w_t))
                d = float(np.dot(rel(pose[:3]) - np.asarray(p_t, dtype=float), R_t[:, 1]))
                _pg, R_gg = self.kin.fk_arm(q_grasp, tuple(w_grasp))
                self._book_off = -R_gg[:, 1] * d
                self.get_logger().info(f"T6b. object off centre by {d*1000:+.1f} mm along the fingers (Gazebo versus kinematics): "
                                       f"the put-back shifts the TCP by {np.round(self._book_off*1000, 1).tolist()} mm")
        self.get_logger().info("T7. object in hand: taking it to the shelf (same put-back as the books)")
        return self.put_back_on_shelf(q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app, False)

    def _reorder_paths(self, dy, off, q_grasp, w_grasp, path_out, q_pre, w_pre):
        """
        IK paths (straight lines, robot frame) moving the book, from the end-of-exit pose, sideways by `dy`
        (m, + = left): translation in front of the shelf, straight re-entry into the destination slot,
        retreat to the pre-grasp. Same orientation as the grasp.
        Return dict(lat, r4, r7, err), err = maximum IK error (m).
        """
        a_out, w_out = path_out
        q_end = list(a_out[-1])
        w_end = [float(w_out[-1][0]), float(w_out[-1][1])]
        p_end, _R = self.kin.fk_arm(q_end, tuple(w_end))
        p_g, R_g = self.kin.fk_arm(q_grasp, tuple(w_grasp))
        a_g = -R_g[:, 2]
        p_pre, _R = self.kin.fk_arm(q_pre, tuple(w_pre))
        d = np.array([0.0, float(dy), 0.0])
        off = np.asarray(off, dtype=float)
        worst = [0.0]

        def straight(p0, p1, n, q0, w0, mode):
            """
            Straight TCP path p0 -> p1 in n IK steps with the grasp orientation; returns (arm_wps, waist_wps)
            """
            qq, ww, arm_w, waist_w = list(q0), tuple(w0), [], []
            for i in range(1, n + 1):
                pt = p0 + (p1 - p0) * i / n
                q1, w1, e = self.kin.ik(pt, approach=a_g, finger_axis=R_g[:, 1], q0=qq, waist=ww,
                                        optimize_waist=mode, restarts=0, w_pos=np.array([30.0, 50.0, 15.0]),
                                        w_approach=0.05, w_finger=40.0)
                if e > 0.01:        # following the previous waypoint may stay in a local minimum: random restarts
                    q2, w2, e2 = self.kin.ik(pt, approach=a_g, finger_axis=R_g[:, 1], q0=qq, waist=ww,
                                             optimize_waist=mode, restarts=6, w_pos=np.array([30.0, 50.0, 15.0]),
                                             w_approach=0.05, w_finger=40.0)
                    if e2 < e:
                        q1, w1, e = q2, w2, e2
                qq, ww = q1, w1
                worst[0] = max(worst[0], float(e))
                arm_w.append(list(qq))
                waist_w.append(list(ww))
            return arm_w, waist_w

        lat = straight(p_end + off, p_end + off + d, 12, q_end, w_end, True)
        r4 = straight(p_end + off + d, p_g + off + d, 10, lat[0][-1], lat[1][-1], True)
        r7 = straight(p_g + off + d, p_pre + off + d, 4, r4[0][-1], r4[1][-1], "pitch")
        return dict(lat=lat, r4=r4, r7=r7, err=worst[0])

    def _reorder_execute(self, dy, q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app):
        """
        With the book in hand at the end of the exit, take it to the destination slot
        """
        off = getattr(self, "_book_off", np.zeros(3))
        paths = self._reorder_paths(dy, off, q_grasp, w_grasp, path_out, q_pre, w_pre)
        self.get_logger().info(f"Y0. paths to the destination slot (dy {dy*100:+.1f} cm): max IK error "
                               f"{paths['err']*1000:.1f} mm")
        if paths["err"] > 0.02:
            self.get_logger().error("Y0. destination not reachable with the book in hand: putting it back in the "
                                    "start slot (nothing forced)")
            return self.put_back_on_shelf(q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app, False)
        ok = self.move_both(paths["lat"][0], paths["lat"][1], 6.0,
                            "Y1. traslo il libro davanti allo scaffale verso lo slot di destinazione")
        ok = ok and self.move_both(paths["r4"][0], paths["r4"][1], 6.0,
                                   "Y2. rientro rettilineo nello slot di destinazione")
        if not ok:
            self.get_logger().error("Y1-Y2 failed: the book STAYS IN HAND, stopping (no blind detach)")
            return False
        self._pause(0.5)
        self.set_auto_attach(False)
        self.get_logger().info("Y3. DETACH in the destination slot")
        if self.use_moveit and self.target and not self.dry_run:
            self.mi.detach(self.side)
        if not self.dry_run:
            self.detach_pub.publish(Empty())
            self.detach_all_pub.publish(Empty())
        self._pause(0.5)
        try:        # the book is in the destination slot: even if the retreat fails, the executor knows it
            with open(f"/tmp/x2_reorder_done_{self.entity}.json", "w") as f:
                json.dump({"entity": self.entity, "dest_dy": dy, "time": time.time(), "detached": True}, f)
        except Exception:
            pass
        p_open = min(GRIPPER_OPEN, p_app + 0.010)
        ok = (self.gripper(p_open, 1.0, f"Y4. apro le dita a {p_open*1000:.1f} mm/dito")
              and self.move_both(paths["r7"][0], paths["r7"][1], 3.0, "Y5. arretro fino al pre-grasp (linea retta)")
              and self.gripper(max(min(p_open, 0.026), p_app), 1.0, "Y5b. richiudo un poco le dita per tornare a casa")
              and self.arm(CARRY_ARM, 2.5, "Y6. braccio raccolto")
              and self.waist(HOME_WAIST, 3.0, "Y7. vita a casa")
              and self.arm(HOME_ARM, 2.5, "Y8. braccio a casa"))
        if ok:
            self.get_logger().info(f"Book moved to the destination slot (dy {dy*100:+.1f} cm). Sequence completed.")
        return ok

    def put_back_on_shelf(self, q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app, shown):
        """
        Put the object back where it was: arm folded, torso to the end-of-exit waist, arm to the end-of-exit
        pose, exit path BACKWARDS to the grasp (the book re-enters along the line it came out on), DETACH,
        fingers opened, approach path backwards to the pre-grasp, home. Joint-space moves happen only with the
        book entirely outside the shelf front.
        """
        a_out, w_out = path_out
        a_in, w_in = path_in
        w_end = [float(w_out[-1][0]), float(w_out[-1][1])]
        # R1 ALWAYS: going straight from the show pose to the end-of-exit pose, the joint interpolation swept
        # the book over the neighbours. resume_from_detach: the book is ALREADY in the slot, skip R1-R4 and do
        # only R5-R10 (detach, open, retreat, home); never with the arm far from the slot.
        resume_det = bool(self.get_parameter("resume_from_detach").value)
        ok = True if resume_det else self.arm(CARRY_ARM, 2.5, "R1. braccio raccolto (libro in mano)")
        # R3 in two steps: from CARRY directly to the end-of-exit pose the joint interpolation overshoots and
        # the book tip (20 cm past the fingers) enters the shelf. R3a = SAFE pose (end of exit moved back 12 cm
        # along the book axis and up 5 cm, reached in joint space away from the shelf), R3b = STRAIGHT approach
        # from there to the end-of-exit pose (IK with the waist fixed).
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
        line[-1] = q_end            # ends exactly on the end-of-exit pose
        self.get_logger().info(f"R3. safe pose at {e_safe*1000:.1f} mm, then 4 straight waypoints to the end of the exit")
        # R4/R7: straight line from the end of the exit to the grasp, SHIFTED by the book offset in the
        # gripper so that the book (not the TCP) returns to its slot; waist free as in the exit
        off = np.asarray(getattr(self, "_book_off", np.zeros(3)), dtype=float)
        p_g, R_g = self.kin.fk_arm(q_grasp, tuple(w_grasp))
        a_g = -R_g[:, 2]
        p_pre_tcp, _R = self.kin.fk_arm(q_pre, tuple(w_pre))
        def straight(p0, p1, n, q0, w0, mode):
            """
            Straight TCP path p0 -> p1 in n IK steps with the grasp orientation; returns (arm_wps, waist_wps)
            """
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
            self.get_logger().info(f"R4/R7 shifted by {np.round(off*1000, 1).tolist()} mm (book off centre in the gripper)")
        ok = (ok and (resume_det or (
                  self.waist(w_end, 3.0, f"R2. busto alla vita di fine uscita {np.round(w_end, 2).tolist()}")
              and self.arm(list(q_safe), 3.0, "R3a. braccio alla posa sicura (12 cm piu' indietro, 5 cm piu' su)")
              and self.arm(line, 4.0, "R3b. avvicinamento rettilineo alla posa di fine uscita")
              and self.move_both(r4_arm, r4_w, 6.0,
                                 "R4. rientro rettilineo fino alla posa di presa (con l'offset del libro)"))))
        if not ok:
            return False
        self._pause(0.5)
        self.set_auto_attach(False)
        self.get_logger().info(f"R5. DETACH ({self.detach_pub.topic_name} + /gripper/{self.side}/detach)")
        if self.use_moveit and self.target and not self.dry_run:
            self.mi.detach(self.side)
        if not self.dry_run:
            self.detach_pub.publish(Empty())
            self.detach_all_pub.publish(Empty())    # also whatever the manager attached on its own
        self._pause(0.5)
        # the approach opening is almost the grasp opening: after the detach the fingers still touch and would
        # rub the book while retreating, so open 10 mm more per finger (max GRIPPER_OPEN) first
        p_open = min(GRIPPER_OPEN, p_app + 0.010)
        if self.kind != "book" and not self.dry_run:
            # object between other objects (or a wall 4 mm away): wide fingers would touch the neighbour
            # already while opening, so open only 4 mm past the current grip
            js = getattr(self, "_js", {})
            held = max([float(js.get(j, 0.0)) for j in self.GJ] + [0.0])
            if held > 0.0:
                p_open = min(p_open, held + 0.004)
        _ro = float(self.get_parameter("release_open").value)
        if _ro == _ro:                                   # not nan: forced opening
            p_open = _ro
        ok = (self.gripper(p_open, 1.0, f"R6. apro le dita a {p_open*1000:.1f} mm/dito (piu' larghe per sfilare)")
              and self.move_both(r7_arm, r7_w, 3.0,
                                 "R7. arretro fino al pre-grasp (linea retta, con l'offset)")
              # at home with 37 mm fingers the right finger touches the table top (MoveIt start state in
              # collision): close a little once the object is free (26 mm, at least the approach opening)
              and self.gripper(max(min(p_open, 0.026), p_app), 1.0, "R7b. richiudo un poco le dita per tornare a casa")
              and self.arm(CARRY_ARM, 2.5, "R8. braccio raccolto")
              and self.waist(HOME_WAIST, 3.0, "R9. vita a casa")
              and self.arm(HOME_ARM, 2.5, "R10. braccio a casa"))
        if ok:
            self.get_logger().info("Object put back in its place. Sequence completed.")
        return ok

    def fingers_open_check(self, p_app, label, tol=0.002):
        """
        Check both fingers are within tol of the approach opening; resend the command up to 3 times, then
        stop (never enter between the books with a closed finger)
        """
        if self.dry_run:
            return True
        for attempt in range(3):
            js = getattr(self, "_js", {})
            pos = [float(js.get(j, -1.0)) for j in self.GJ]
            if all(abs(p - p_app) <= tol for p in pos):
                self.get_logger().info(f"{label}: fingers at {[round(p*1000, 1) for p in pos]} mm")
                return True
            self.get_logger().warn(f"{label}: fingers at {[round(p*1000, 1) for p in pos]} mm, expected {p_app*1000:.1f}: resending the command")
            self.gripper(p_app, 1.0, f"{label}: riapro")
        js = getattr(self, "_js", {})
        pos = [float(js.get(j, -1.0)) for j in self.GJ]
        if all(abs(p - p_app) <= tol for p in pos):
            return True
        self.get_logger().error(f"{label}: a finger does not open ({[round(p*1000, 1) for p in pos]} mm): stopping without touching the shelf")
        self.arm(HOME_ARM, 2.5, "braccio a casa")
        return False

    def _pause(self, sec):
        """
        Sleep for sec seconds (not in dry_run)
        """
        if not self.dry_run:
            time.sleep(sec)

    # --- sequence ---

    def set_auto_attach(self, on: bool):
        """
        Turn the GraspManager auto-attach on or off; waits for the subscriber first, since messages published
        before the DDS match are lost, then publishes several times
        """
        if self.dry_run:
            return
        t0 = time.time()
        while self.auto_attach_pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.auto_attach_pub.get_subscription_count() == 0:
            self.get_logger().warn("GraspManager not listening on /gripper/auto_attach (grasp_manager:=false?)")
            return
        for _ in range(3):
            self.auto_attach_pub.publish(Bool(data=bool(on)))
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"GraspManager auto_attach {'ON' if on else 'OFF'}")

    def _object_grasp_from_shape(self, z_top):
        """
        Grasp height of a NON-book object from its depth-measured width profile [(z, width)] in 1 cm bands.
        Squeeze where the object is WIDEST among the bands that fit the gripper (a sphere at the equator: the
        fingers cannot slide towards a wider part), the highest on ties (forearm away from the plank); if none
        fits, a narrower neck. Return (z, width), or (None, None) if the profile is missing or nothing fits.
        """
        prof = [(float(z), float(w)) for z, w in (getattr(self, "width_profile", None) or [])]
        if len(prof) < 2:
            return None, None
        # usable gripper opening: fingers at GRIPPER_OPEN + minimum gap, minus a margin per side
        w_max = GRIPPER_MIN_GAP + 2.0 * GRIPPER_OPEN - 2.0 * 0.006
        z_lo = min(z for z, _w in prof) + 0.015          # not flush with the plank
        z_hi = z_top - 0.008                             # not on the top edge
        bands = [(z, w) for z, w in prof if z_lo <= z <= z_hi and w > 0.005]
        if not bands:
            bands = prof
        w_top = max(w for _z, w in bands)
        fit = [(z, w) for z, w in bands if w <= w_max]
        if not fit:
            self.get_logger().error(
                f"Object {w_top*1000:.0f} mm wide at every height: does not fit the gripper (max {w_max*1000:.0f} mm)")
            return None, None
        w_best = max(w for _z, w in fit)
        # "wide enough" bands (>= 90% of the widest that fits): the highest one
        wide = [(z, w) for z, w in fit if w >= 0.9 * w_best]
        z_sel, w_sel = max(wide, key=lambda t: t[0])
        how = "la fascia piu' larga che entra nella pinza"
        if w_top > w_max:
            how = f"un collo (la parte piu' larga, {w_top*1000:.0f} mm, non entra nella pinza)"
        self.get_logger().info(
            f"Object shape ({len(prof)} bands, width {min(w for _z,w in prof)*1000:.0f}-{w_top*1000:.0f} mm): "
            f"squeezing at z={z_sel:.3f} where it is {w_sel*1000:.0f} mm wide - {how}")
        return z_sel, w_sel

    def _robot_x_from_joints(self):
        """
        Update self.robot_x from base_x_joint (see the robot_x_from_joints parameter)
        """
        if not bool(self.get_parameter("robot_x_from_joints").value):
            return
        t0 = time.time()
        while "base_x_joint" not in getattr(self, "_js", {}) and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        js = getattr(self, "_js", {})
        if "base_x_joint" not in js:
            self.get_logger().warn(f"robot_x: no /joint_states, keeping {self.robot_x:.3f}")
            return
        x = float(self.get_parameter("robot_spawn_x").value) + float(js["base_x_joint"])
        if abs(x - self.robot_x) > 0.0005:
            self.get_logger().info(
                f"robot_x from the joints: {x:.3f} (nominal {self.robot_x:.3f}, base_x {js['base_x_joint']:.3f})")
        self.robot_x = x

    def run(self):
        """
        Plan the grasp (IK, paths, release) and run the sequence chosen by the parameters.
        Return True when completed; SystemExit(3) = out of reach for this arm before anything moved.
        """
        self._robot_x_from_joints()
        self.set_auto_attach(False)
        sz = self.height
        sx = self.length
        sy = self.thick_mesh
        z_center = self.z_center
        z_top = self.z_center + sz / 2.0
        gft = float(self.get_parameter("grasp_from_top").value)
        if self.kind == "book" and gft > 0.0 and z_top - gft > z_center:
            z_center = z_top - gft
            self.get_logger().info(f"Book: spine grasped {gft*100:.1f} cm from the top (z={z_center:.3f}, "
                                   "barcode free)")
        if self.kind != "book":
            z_pick, w_pick = self._object_grasp_from_shape(z_top)
            if z_pick is not None:
                z_center = z_pick
                if w_pick is not None:
                    sy = self.thick_mesh = float(w_pick)     # opening on the width at THAT height
            elif z_top - OBJECT_GRASP_FROM_TOP > z_center:
                # no width profile: low objects grasped at mid height put the forearm under the plank edge
                # (see PLANK_FRONT_X) -> grasp near the top
                z_center = z_top - OBJECT_GRASP_FROM_TOP
                self.get_logger().info(f"Low object: grasp raised from z={self.z_center:.3f} to {z_center:.3f} "
                                       f"(top at {z_top:.3f})")
        # REAL object centre relative to the grasp point along the height (0 if grasped at mid height), positive
        # if the grasp is HIGHER than the centre: used by the head show (books only) and by the table release,
        # which otherwise sinks an object grasped above its centre into the table
        self._book_dz = float(self.z_center - z_center) if self.kind == "book" else 0.0
        self._grasp_dz = float(z_center - self.z_center)
        spine_x = self.spine_x
        g = np.array([spine_x + float(self.get_parameter("grasp_depth").value), self.by, z_center])
        self._grasp_point_world = g.copy()
        rel = lambda p: np.asarray(p) - np.array([self.robot_x, self.robot_y, 0.0])
        back = float(self.get_parameter("approach_back").value)
        retreat = float(self.get_parameter("retreat").value)

        self.get_logger().info(
            f"Book {self.entity} ({self.key}): world centre=({self.bx:.3f},{self.by:.3f},{z_center:.3f}) "
            f"spine x={spine_x:.3f}, world grasp point={np.round(g,3).tolist()}, robot at x={self.robot_x}")

        # approach opening: just enough to enter beside the object without hitting the neighbours
        free_plus, free_minus = self.free
        p_app, p_min, p_max = approach_opening(sy, free_plus, free_minus)
        if self.kind != "book" and p_max >= p_min:
            # the depth "thickness" is the width of the front face: for a round object it is less than the
            # maximum width and p_min+2 mm hit the front, so 1 cm more if there is room
            p_app = max(0.0, min(p_max, p_min + OBJECT_EXTRA_OPENING))
        self.get_logger().info(
            f"Free space on the sides: +y {free_plus*1000:.0f} mm, -y {free_minus*1000:.0f} mm -> "
            f"approach opening {p_app*1000:.1f} mm/finger "
            f"(allowed {p_min*1000:.1f}..{p_max*1000:.1f})")
        if p_max < p_min:
            self.get_logger().error(
                f"No room for the fingers (1 cm) next to {self.entity}: "
                f"{p_min*1000:.1f} mm/finger needed, {p_max*1000:.1f} available. "
                "Move the neighbours apart in book_placer.GRASP_TEST_ENTITIES. Stopping.")
            return

        t0 = time.time()

        def orient_deg(q, w):
            """
            Angle (degrees) between the gripper approach axis and world +x
            """
            _p, R = self.kin.fk_arm(q, tuple(w))
            return math.degrees(math.acos(min(1.0, max(-1.0, float(-R[:, 2] @ np.array([1.0, 0, 0]))))))

        def free_path(p0, p1, n, q0, w0, lift, mode):
            """
            STRAIGHT TCP motion with the waist free (mode 'pitch' or True = yaw+pitch): finger axis strongly
            constrained (w_finger 40, the book neither turns nor rolls), approach almost free (w_approach 0.05,
            the forearm may tilt), position tight in y (50) and loose in x/z (30/15), plus `lift` along the path.
            With a fixed waist the gripper rotated up to 49 degrees and swept the neighbours; this way it stays
            below 1 degree. Return (arm_wps, waist_wps, max 3D error, max angle, x_end).
            """
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
                # full 3D error: with the weak x/z weights the IK can drift by cm in x and z unnoticed
                dy_max = max(dy_max, float(np.linalg.norm(pos - p)))
                ang_max = max(ang_max, yaw_b, roll_b)
                arm_wps.append(list(qq)); waist_wps.append(list(ww))
            x_end = float(pos[0])
            return arm_wps, waist_wps, dy_max, ang_max, x_end

        def plan_paths(q_g, w_g, retreat_m):
            """
            Pre-grasp + approach (pitch free) + exit (yaw+pitch free). Return (q_pre, path_in, path_out, errs,
            angs): path_* are (arm_wps, waist_wps); errs in metres (pre-grasp, max path error, missing exit
            length); angs in degrees (max gripper rotation).
            """
            p_pre = rel(g - [back, 0, 0])
            # (a) pre-grasp with restarts (seed q_g), then path pre -> grasp: the version verified on books
            q_p, _w, e_p = self.kin.ik(p_pre, q0=q_g, waist=w_g)
            a_in, w_in, dy_in, ang_in, _x = free_path(p_pre, rel(g), 4, q_p, w_g, 0.0, "pitch")
            # the last waypoint is EXACTLY the grasp pose
            a_in[-1], w_in[-1] = list(q_g), list(w_g)
            w_p = list(w_g)
            clr_f = plank_clearance(q_g, w_g, (a_in, w_in))
            if clr_f < PLANK_CLEARANCE_MIN:
                # (b) the random restarts may pick a "low elbow" branch with the forearm under the plank: plan
                #     BACKWARDS from the grasp and reverse it (same branch as q_grasp, ends exactly at the grasp).
                #     Only when needed, so the verified book grasps do not change.
                a_bk, w_bk, dy_b, ang_b, _x = free_path(rel(g), p_pre, 4, q_g, w_g, 0.0, "pitch")
                q_b, w_b = list(a_bk[-1]), list(w_bk[-1])
                a_b = [list(q) for q in reversed(a_bk[:-1])] + [list(q_g)]
                w_bi = [list(w) for w in reversed(w_bk[:-1])] + [list(w_g)]
                clr_b = plank_clearance(q_g, w_g, (a_b, w_bi))
                self.get_logger().info(
                    f"    forward approach: forearm {clr_f*1000:+.0f} mm from the plank -> "
                    f"planned backwards from the grasp: {clr_b*1000:+.0f} mm")
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

        # exit: at least until the object is ENTIRELY past the shelf front (then the waist rotates)
        retreat_needed = (spine_x + sx) - (SHELF_FRONT_X - SHELF_EXIT_MARGIN)
        if retreat_needed > retreat:
            self.get_logger().info(
                f"Exit lengthened from {retreat*100:.0f} to {retreat_needed*100:.0f} cm: "
                f"the object is {sx*100:.1f} cm wide and must pass the front (x={SHELF_FRONT_X})")
            retreat = retreat_needed

        # Grasp candidates: 1) torso straight (pitch only); 2) if position or ORIENTATION is off (5 joints +
        # pitch cannot always reach the 6D pose, and a rotated gripper sweeps the neighbours 2 cm away), waist
        # yaw free and intermediate yaws. The winner has feasible paths (<= 2 cm) and minimum orientation.
        # Plank under the grasp: forearm and gripper mount must stay above it when past its front edge.
        surface_z = max([z_ for z_ in SHELF_SURFACES_Z if z_ <= z_center + 1e-3] or [SHELF_SURFACES_Z[0]])

        def plank_clearance(q_c, w_c, p_in):
            """
            Minimum height (m) of forearm and gripper mount above the plank, in the grasp pose and along the
            approach; 1.0 if no point is over the plank
            """
            off = np.array([self.robot_x, self.robot_y, 0.0])
            worst_c = 1.0
            for qq, ww in [(q_c, w_c)] + list(zip(p_in[0], p_in[1])):
                pts = self.kin.fk_joints(qq, tuple(ww))
                el = pts[f"{self.side}_elbow_joint"] + off
                wr = pts[f"{self.side}_wrist_yaw_joint"] + off
                mo = pts[f"{self.side}_gripper_mount_joint"] + off
                for s_ in np.linspace(0.0, 1.0, 11):
                    pt = el + (wr - el) * s_
                    if pt[0] >= PLANK_FRONT_X - FOREARM_RADIUS:
                        worst_c = min(worst_c, pt[2] - FOREARM_RADIUS - surface_z)
                if mo[0] >= PLANK_FRONT_X - FOREARM_RADIUS:
                    worst_c = min(worst_c, mo[2] - FOREARM_RADIUS - surface_z)
            return worst_c

        def search_grasp():
            """
            Evaluate the grasp candidates and return the best one (feasible first, then least rotated)
            """
            yaw_retry = float(self.get_parameter("yaw_retry_mm").value) / 1000.0
            q_a, w_a, e_a = self.kin.ik(rel(g), optimize_waist="pitch")
            cands = [("busto dritto", q_a, w_a, e_a)]
            o_a = orient_deg(q_a, w_a)
            if e_a > yaw_retry or o_a > 3.0:
                self.get_logger().info(
                    f"IK with the torso straight: {e_a*1000:.1f} mm, gripper rotated by {o_a:.1f} degrees -> "
                    "trying with the waist yaw free")
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
                    self.get_logger().info(f"  candidate {label}: grasp {e_c*1000:.1f} mm -> discarded")
                    continue
                o_c = orient_deg(q_c, w_c)
                (q_p, w_p), p_in, p_out, errs_c, angs_c = plan_paths(q_c, w_c, retreat)
                worst = max(errs_c.values())
                worst_ang = max(angs_c.values())
                self.get_logger().info(
                    f"  candidate {label}: grasp {e_c*1000:.1f} mm, orientation {o_c:.1f} degrees, "
                    f"paths: max lateral {worst*1000:.1f} mm, max gripper rotation {worst_ang:.1f} degrees")
                clr = plank_clearance(q_c, w_c, p_in)
                self.get_logger().info(f"    forearm above the plank: {clr*1000:+.0f} mm (min {PLANK_CLEARANCE_MIN*1000:.0f})")
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
                    f"Forearm {clr_best*1000:+.0f} mm from the plank (min {PLANK_CLEARANCE_MIN*1000:.0f}) and "
                    f"no higher grasp available (top {z_top:.3f}): going on, it may get stuck")
                break
            self.get_logger().info(
                f"Forearm {clr_best*1000:+.0f} mm from the plank: raising the grasp to z={z_new:.3f}")
            g[2] = z_new
        _score, label, q_grasp, w_grasp, e1, o_best, q_pre, path_in, path_out, errs_p, _clr, w_pre_plan = best
        # "book up" direction in the gripper frame: depending on the IK branch the gripper X points up OR down;
        # the book is rigid with the gripper, so it is stored here for the head show (up = R @ _book_up_g)
        _pg, R_g = self.kin.fk_arm(q_grasp, tuple(w_grasp))
        self._book_up_g = R_g.T @ np.array([0.0, 0.0, 1.0])
        # book DEPTH axis (into the shelf) in the gripper frame: if the gripper is rotated relative to the book
        # it differs from the approach axis (-Z gripper), and plan_show would misplace the book corners
        self._book_depth_g = R_g.T @ np.array([1.0, 0.0, 0.0])
        e2, e3, e5 = errs_p["pre"], errs_p["approach"], max(errs_p["retreat"], errs_p["uscita_incompleta"])
        self.get_logger().info(f"Grasp: {label} (orientation {o_best:.1f} degrees)")
        if o_best > 5.0:
            self.get_logger().warn(
                f"Gripper rotated by {o_best:.1f} degrees from the approach: with 2 cm between the "
                "books the object may touch the neighbours on the way out")
        # release INSIDE the table top: waist at -90 degrees, gripper pointing to world -y, fingers along x
        # close on the COLLISION thickness (narrower than the mesh for books)
        opening = grasp_opening(self.thick_close)
        face_down = self.kind == "book"
        if face_down:
            # book laid FLAT, cover down: fingers vertical, the lower finger (outer face at TCP - 0.025 -
            # opening) must stay above the top -> TCP at 5 cm + opening above the table
            z_rel = TABLE_TOP_Z + 0.05 + opening
        else:
            # + _grasp_dz: if the grasp point is above the real centre, the TCP must be higher by the same
            # amount, otherwise the lower half of the object ends up below the table top
            z_rel = TABLE_TOP_Z + sz / 2.0 + OBJECT_TABLE_RELEASE_AIR + getattr(self, "_grasp_dz", 0.0)
        p_rel = np.array([float(self.get_parameter("release_x").value),
                          float(self.get_parameter("release_y").value), z_rel])
        # first the solution with horizontal fingers (book upright): the wrist turns about the approach axis
        # (gripper Z), so +-pi/2 on the wrist lays the book down without moving the TCP; the cover normal
        # picks the sign
        q_rel, w_rel, e_rel = self.kin.ik(rel(p_rel), approach=(0, -1, 0), finger_axis=(1, 0, 0),
                                          waist=tuple(self.drop_waist), optimize_waist="pitch")
        if e_rel > 0.01:
            # slot off the -90 degree waist line (pipeline release_x/y): yaw free
            q2, w2, e2 = self.kin.ik(rel(p_rel), approach=(0, -1, 0), finger_axis=(1, 0, 0),
                                     waist=tuple(self.drop_waist), optimize_waist=True)
            if e2 < e_rel:
                q_rel, w_rel, e_rel = q2, w2, e2
                self.get_logger().info(
                    f"Release with the waist yaw free ({w_rel[0]:+.2f} rad): {e_rel*1000:.1f} mm")
        if face_down and e_rel <= 0.02:
            _pos, R_grasp = self.kin.fk_arm(q_grasp, tuple(w_grasp))
            n_g = R_grasp.T @ BOOK_COVER_NORMAL_WORLD      # cover normal in the gripper frame
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
                    f"FACE DOWN release: wrist turned to {q_rel[4]:+.2f} rad, "
                    f"cover normal z={best_z:+.2f} (back cover with the ISBN towards the camera)")
            else:
                self.get_logger().warn(
                    f"Cannot lay the book cover down (wrist limits): "
                    f"placing it upright (normal z={best_z:+.2f})")
        errs = dict(grasp=e1, pre=e2, approach=e3, retreat=e5)
        self.get_logger().info(
            f"IK in {time.time()-t0:.1f}s - max errors: " +
            ", ".join(f"{k} {v*1000:.1f}mm" for k, v in errs.items()) +
            f"; release {e_rel*1000:.1f}mm at {np.round(p_rel, 3).tolist()}")
        if max(errs.values()) > 0.02:
            self.get_logger().error("IK error > 2 cm: book out of reach from this position, stopping.")
            _dy = float(self.get_parameter("dest_y").value)
            if _dy == _dy:                    # reorder: nothing moved, the executor tries the other arm
                raise SystemExit(3)
            return
        if e_rel > 0.02:
            self.get_logger().warn(
                f"Release IK does not converge ({e_rel*1000:.0f} mm): using DROP_ARM (drop from the edge)")
            q_rel, w_rel = DROP_ARM, self.drop_waist
        w_pre = list(w_pre_plan)     # pre-grasp waist as planned (backward path)
        dest_y = float(self.get_parameter("dest_y").value)
        reorder = dest_y == dest_y                                   # not nan
        if reorder:
            dy_move = dest_y - float(self.by)
            chk = self._reorder_paths(dy_move, np.zeros(3), q_grasp, w_grasp, path_out, q_pre, w_pre)
            self.get_logger().info(f"Y-. check BEFORE moving: from slot y={self.by:+.3f} to slot y={dest_y:+.3f} "
                                   f"(dy {dy_move*100:+.1f} cm), max IK error {chk['err']*1000:.1f} mm ({self.side})")
            if chk["err"] > 0.02:
                self.get_logger().error(f"Y-. destination out of reach for the {self.side} arm: not moving anything")
                raise SystemExit(3)

        tbx = float(self.get_parameter("from_table_x").value)
        tby = float(self.get_parameter("from_table_y").value)
        if tbx == tbx and tby == tby:                     # object from the TABLE to the shelf
            tp = self._table_plan(tbx, tby)
            if tp["err"] > 0.02:
                self.get_logger().error("T-. grasp from the table out of reach for this arm: not moving anything")
                raise SystemExit(3)
            return self._table_execute(tp, q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app)

        # Order: FIRST the arm folded (hand free in front of the chest), THEN the fingers: at home the hand is
        # against the hip and the outer finger did not open. Both fingers are checked open before approaching.
        if bool(self.get_parameter("resume_put_back").value) and self.put_back and not self.dry_run:
            # RESUME: the book is ALREADY in hand at the end of the exit; skip the grasp and go to the
            # put-back. Needs the object attached in the planning scene and the DetachableJoint still active.
            self.get_logger().warn("RESUME: book already in hand at the end of the exit: putting it back in the slot")
            self._book_off = np.array([float(x) for x in
                                       str(self.get_parameter("resume_book_off").value).split(",")]) * 0.001
            self._arm_now = list(q_grasp) if bool(self.get_parameter("resume_from_detach").value) \
                else list(path_out[0][-1])
            return self.put_back_on_shelf(q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app, False)
        ok = (
            self.arm(CARRY_ARM, 2.5, "1. braccio raccolto")
            and self.gripper(p_app, 1.0, f"1b. apri gripper a {p_app*1000:.1f} mm/dito (avvicinamento)")
            and self.fingers_open_check(p_app, "1c. verifica dita aperte")
            and self.waist(list(w_pre), 2.0, "2. vita (pitch) per il pre-grasp")
            and self.to_pregrasp(q_pre, w_pre, rel(g - [back, 0, 0]))
            and self.move_both(path_in[0], path_in[1], 3.0, "4-5. avvicinamento rettilineo fino al grasp (braccio+vita)")
        )
        if not ok:
            # never leave the arm stretched (the next walk drags it and the hand spoils the camera depth)
            self._safe_home("fermato prima della presa")
            return
        if self.contact_close:
            ok = self.close_until_contact(p_app, "6. chiudo le dita fino al contatto") is not None
            # book offset from the TCP along the finger axis: the sensor finger touches at p_actual, a centred
            # book would be touched at p_nom = (collision - gap_min)/2. The book is attached where it is, so the
            # put-back path is shifted by this offset.
            self._book_off = np.zeros(3)
            if ok and not self.dry_run:
                p_nom = max(0.0, (self.thick_close - GRIPPER_MIN_GAP) / 2.0)
                d_off = p_nom - float(self._grip_now)
                _pg, R_gg = self.kin.fk_arm(q_grasp, tuple(w_grasp))
                self._book_off = R_gg[:, 1] * d_off
                self.get_logger().info(f"   book off centre by {d_off*1000:+.1f} mm along the fingers "
                                       f"(contact at {self._grip_now*1000:.1f} mm, expected {p_nom*1000:.1f})")
        else:
            ok = self.gripper(opening, 1.5, f"6. chiudo le dita a {opening*1000:.1f} mm/lato")
            self._book_off = np.zeros(3)
        if not ok:
            # closing failed: do NOT leave the arm in the shelf. Open to p_app, retreat along the approach
            # path, fold, waist home, home.
            self.get_logger().warn("6. closing failed: opening and going home along the approach path")
            self.gripper(p_app, 1.0, f"apri le dita a {p_app*1000:.1f} mm/dito")
            self.move_both([list(q) for q in reversed(path_in[0][:-1])] + [list(q_pre)],
                           [list(w) for w in reversed(path_in[1][:-1])] + [list(w_pre)], 3.0,
                           "arretro fino al pre-grasp")
            self.arm(CARRY_ARM, 2.5, "braccio raccolto")
            self.waist(HOME_WAIST, 3.0, "vita a casa")
            self.arm(HOME_ARM, 2.5, "braccio a casa")
            return
        self._pause(0.5)
        # before attaching, the sensor finger must touch THIS object (fixed close only: the contact close
        # already verified it). The wait is in REAL time: at RTF 0.05, 2 s real are only 0.1 s sim.
        if not self.dry_run and not self.contact_close:
            self._contact_model = None
            t_c = time.time()
            while time.time() - t_c < 20.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._contact_model == self.entity:
                    break
            if self._contact_model != self.entity:
                self.get_logger().error(
                    f"7. the fingers do not touch {self.entity} (contact: {self._contact_model or 'none'}): "
                    "misplaced grasp, NOT attaching. Opening and going home.")
                # APPROACH opening, not full: the fingers are still between the neighbours (full opening
                # pushes them out of the shelf)
                self.gripper(p_app, 1.0, f"apri le dita a {p_app*1000:.1f} mm/dito")
                self.move_both([list(q) for q in reversed(path_in[0][:-1])] + [list(q_pre)],
                               [list(w) for w in reversed(path_in[1][:-1])] + [list(w_pre)], 3.0,
                               "arretro fino al pre-grasp")
                # FIRST folded, THEN home: from the pre-grasp the direct interpolation to [0,0,0,0,0] passes
                # with the arm horizontal inside the bookshelf
                self.arm(CARRY_ARM, 2.5, "braccio raccolto")
                self.waist(HOME_WAIST, 3.0, "vita a casa")
                self.arm(HOME_ARM, 2.5, "braccio a casa")
                return False
        if self._contact_model in self._known_book_names and self.kind != "book":
            # perception classified as "decoration" (so, to the table) something the physical contact shows is
            # a REAL book (Gazebo ground truth): a book must never go on the table (it stays in hand for the
            # ISBN), so stop here BEFORE attaching
            self.get_logger().error(
                f"7. SAFETY STOP: the contact is with {self._contact_model} (a real book), "
                f"but perception classified it as '{self.kind}' (destination: table). "
                "A book must never be placed on the table: opening and going home without touching it.")
            self.gripper(p_app, 1.0, f"apri le dita a {p_app*1000:.1f} mm/dito")
            self.move_both([list(q) for q in reversed(path_in[0][:-1])] + [list(q_pre)],
                           [list(w) for w in reversed(path_in[1][:-1])] + [list(w_pre)], 3.0,
                           "arretro fino al pre-grasp (libro non toccato)")
            self.arm(CARRY_ARM, 2.5, "braccio raccolto")
            self.waist(HOME_WAIST, 3.0, "vita a casa")
            self.arm(HOME_ARM, 2.5, "braccio a casa")
            return False
        self.get_logger().info(f"7. ATTACH ({self.attach_pub.topic_name})")
        if not self.dry_run:
            self._attach()
            if self.use_moveit and self.target:
                attached_ok = self.mi.attach(self.side, self.entity)   # obj<id> in the planning scene
                if not attached_ok:
                    self.get_logger().warn(f"7. ATTACH: {self.entity} not confirmed in the planning scene, retrying")
                    attached_ok = self.mi.attach(self.side, self.entity)
                if not attached_ok:
                    # if the scene does not know the object is held, every later plan treats the gripper as
                    # EMPTY and MoveIt does not protect neighbours/table from it. The object is still in place:
                    # detach now and go back EMPTY along the verified approach path.
                    self.get_logger().error(
                        f"7. ATTACH: {self.entity} not confirmed in hand after 2 attempts: "
                        "not going on blindly, reopening and going home without moving the object")
                    self.detach_pub.publish(Empty())
                    self.detach_all_pub.publish(Empty())
                    self.gripper(p_app, 1.0, f"apri le dita a {p_app*1000:.1f} mm/dito")
                    self.move_both([list(q) for q in reversed(path_in[0][:-1])] + [list(q_pre)],
                                   [list(w) for w in reversed(path_in[1][:-1])] + [list(w_pre)], 3.0,
                                   "arretro fino al pre-grasp (oggetto lasciato sullo scaffale)")
                    self.arm(CARRY_ARM, 2.5, "braccio raccolto")
                    self.waist(HOME_WAIST, 3.0, "vita a casa")
                    self.arm(HOME_ARM, 2.5, "braccio a casa")
                    return False
        self._pause(0.5)

        # no joint-space "carry pose" before the rotation (it swept the book over the neighbours): the
        # straight exit brings the object entirely out of the shelf front first
        ok = self.move_both(path_out[0], path_out[1], 6.0,
                            "8-9. sfilo l'oggetto all'indietro (rettilineo, braccio+vita, tutto fuori dallo scaffale)")
        if not ok:
            return
        if reorder:
            return self._reorder_execute(dy_move, q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app)
        to_table = self.use_moveit and not self.put_back and not (self.head_isbn and self.kind == "book")
        if to_table:
            # right after the exit the object is still a hair from the shelf front in the planning scene, and
            # the big waist+arm jump starts from an almost-colliding state: a step back gives real margin
            self._step_back(0.15)
            # q_rel/w_rel were computed before the step back (release_x/y are world, not robot-relative):
            # recompute them with the updated self.robot_x (rel() reads it again)
            self._robot_x_from_joints()
            q_rel, w_rel, e_rel = self.kin.ik(rel(p_rel), approach=(0, -1, 0), finger_axis=(1, 0, 0),
                                              waist=tuple(self.drop_waist), optimize_waist="pitch")
            if e_rel > 0.01:
                q2, w2, e2 = self.kin.ik(rel(p_rel), approach=(0, -1, 0), finger_axis=(1, 0, 0),
                                         waist=tuple(self.drop_waist), optimize_waist=True)
                if e2 < e_rel:
                    q_rel, w_rel, e_rel = q2, w2, e2
            self.get_logger().info(
                f"release recomputed after the step back: error {e_rel*1000:.1f} mm")
            if e_rel > 0.02:
                self.get_logger().warn(
                    f"release after the step back does not converge ({e_rel*1000:.0f} mm): using DROP_ARM")
                q_rel, w_rel = DROP_ARM, self.drop_waist
        if to_table:
            if not self.to_release(q_rel, w_rel, "10-13. verso il punto di rilascio dentro il tavolo"
                                   + (" (libro coricato)" if face_down else "")):
                # NEVER leave the arm stretched with the object in hand, and never detach blindly (it would
                # fall anywhere): only fold the arm (CARRY, object still held) into a known safe pose, then
                # stop and report; resume by hand or by relaunching
                self.get_logger().error(
                    "10-13: no way found to reach the release with the object in hand: "
                    "only trying to fold the arm, then stopping (NOT detaching the object blindly)")
                if not self.arm(CARRY_ARM, 3.0, "10-13 fallito: braccio raccolto (oggetto ancora in mano)"):
                    self.get_logger().error("folding failed too: the arm stays where it is, with the object in hand")
                else:
                    self._step_forward_undo()
                return
            # release pose (q, waist, point relative to the robot NOW, after the step back) for the straight
            # retreat in _release_and_home
            self._rel_ctx = dict(q=list(q_rel), w=list(w_rel), p=np.asarray(rel(p_rel), dtype=float))
            return self._release_and_home(face_down)
        show_yaw = self.drop_waist[0]      # the angle does not change the show pose (camera and arm rotate together)
        ok = self.waist([show_yaw, float(path_out[1][-1][1])], 3.0, "10-11. vita verso il tavolo (braccio fermo)")
        if not ok:
            if self.put_back:
                # never leave the book in hand outside the shelf
                self.get_logger().error("10-11: waist rotation rejected: putting the book back in the "
                                        "slot WITHOUT reading the ISBN (no forced motion)")
                return self.put_back_on_shelf(q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app, False)
            return
        # slot with the waist little rotated (yaw > -1.0): coming back from -1.57 with the arm still stretched
        # the tip would sweep the shelf front again (r~0.41 m from the waist axis: x > 0.25 for |yaw| < 0.64),
        # so fold the arm first, then rotate, then stretch over the slot
        shown = False
        if self.head_isbn and self.kind == "book":
            self.show_to_head_and_read([show_yaw, float(path_out[1][-1][1])])
            shown = True
        if self.put_back:
            return self.put_back_on_shelf(q_grasp, w_grasp, path_in, path_out, q_pre, w_pre, p_app, shown)
        if w_rel[0] > -1.0 or shown:
            ok = self.arm(CARRY_ARM, 2.5, "11b. braccio raccolto (slot con busto poco ruotato)"
                          if not shown else "11b. braccio raccolto (dopo la mostra alla testa)")
            if not ok:
                return
        if self.use_moveit:
            ok = self.goto(list(q_rel), list(w_rel), "12-13. verso il punto di rilascio dentro il tavolo (MoveIt)"
                           + (" (libro coricato)" if face_down else ""))
        else:
            ok = (
                self.waist(list(w_rel), 2.0, "12. busto verso lo slot")
                and self.arm(list(q_rel), 3.0, "13. braccio sul punto di rilascio dentro il tavolo"
                             + (" (libro coricato)" if face_down else ""))
            )
        if not ok:
            return
        return self._release_and_home(face_down)

    def _gz_pose(self, model):
        """
        (x, y, z, qz, qw) of a Gazebo model from the simulation ground truth (gz topic pose/info), to keep the
        planning scene aligned with where the object REALLY ended up (on a real robot: table perception).
        None if not available.
        """
        import re
        import subprocess
        try:
            txt = subprocess.run(["gz", "topic", "-e", "-t", "/world/bookshelf_world/pose/info", "-n", "1"],
                                 capture_output=True, text=True, timeout=90).stdout
        except Exception as e:
            self.get_logger().warn(f"real pose of {model}: gz not responding ({e})")
            return None
        i = txt.find(f'name: "{model}"')
        if i < 0:
            return None
        blk = txt[i:i + 900]

        def grab(tag):
            """
            Numeric fields of the `tag { ... }` block
            """
            m = re.search(tag + r"\s*\{([^}]*)\}", blk)
            return {k: float(v) for k, v in re.findall(r"(\w+):\s*([-+0-9.e]+)", m.group(1))} if m else {}
        pp, oo = grab("position"), grab("orientation")
        return pp.get("x", 0.0), pp.get("y", 0.0), pp.get("z", 0.0), oo.get("z", 0.0), oo.get("w", 1.0)

    def _sync_placed_pose(self, final):
        """
        Put the just-placed object in the planning scene where it really is (a released object can end up
        several cm away and rotated, and the next object's slot is close). With final=True also write
        /tmp/x2_placed_<entity>.json, read by library_pipeline to choose the next slot.
        """
        if not (self.use_moveit and self.target and not self.dry_run and getattr(self, "_rel_ctx", None)):
            return
        import json
        import math
        rx, ry = float(self.get_parameter("release_x").value), float(self.get_parameter("release_y").value)
        held = getattr(self, "_held_model", None) or self._contact_model
        pose = self._gz_pose(held) if held else None
        if pose is None:
            x, y, yaw = rx, ry, None
            self.get_logger().warn("real pose of the object not available: using the release point")
        else:
            x, y = pose[0], pose[1]
            yaw = math.degrees(2.0 * math.atan2(pose[3], pose[4]))
            self.get_logger().info(
                f"real pose of {held} on the table: ({x:+.3f}, {y:+.3f}) z={pose[2]:.3f}, "
                f"{math.hypot(x - rx, y - ry) * 100:.1f} cm from the release point ({rx:+.2f}, {ry:+.2f}), rotation {yaw:+.0f} degrees")
        if not self.mi.place(self.entity, x, y, TABLE_TOP_Z):
            self.get_logger().warn("15c. the scene does not confirm the object position on the table")
        if final:
            with open(f"/tmp/x2_placed_{self.entity}.json", "w", encoding="utf-8") as f:
                json.dump({"entity": self.entity, "model": held, "x": x, "y": y,
                           "release": [rx, ry], "yaw_deg": yaw, "measured": pose is not None}, f)

    def _retreat_after_release(self, d=0.10):
        """
        STRAIGHT post-place retreat of d metres towards +y (the gripper points to -y at the release), checked
        waypoint by waypoint against the planning scene: with the fingers a few mm from the placed object MoveIt
        rejects the direct move to CARRY (INVALID_MOTION_PLAN). On failure it logs and the usual return follows.
        """
        ctx = getattr(self, "_rel_ctx", None)
        if not ctx or self.dry_run or not self.use_moveit:
            return True
        p0 = np.asarray(ctx["p"], dtype=float)
        p1 = p0 + np.array([0.0, d, 0.0])
        n = 5
        # restarts=0 (as in free_path): with random restarts the wrist jumped 2.5 rad between two waypoints
        # (branch change); this way the maximum jump is 0.2-0.5 rad
        wp, err, q = [], 0.0, list(ctx["q"])
        for i in range(1, n + 1):
            q, _w, e = self.kin.ik(p0 + (p1 - p0) * (i / n), q0=q, waist=tuple(ctx["w"]),
                                   optimize_waist=False, restarts=0,
                                   approach=(0, -1, 0), finger_axis=(1, 0, 0))
            wp.append(list(q))
            err = max(err, e)
        if err > 0.02:
            self.get_logger().warn(f"15b. retreat from the release: IK error {err*1000:.0f} mm, skipping it")
            return False
        ok = self.move_both([list(q) for q in wp], [list(ctx["w"])] * n, 3.0,
                            f"15b. ritiro rettilineo di {d*100:.0f} cm dal rilascio (verificato)")
        if not ok:
            self.get_logger().error("15b. retreat from the release not executed: trying the return to CARRY anyway")
        return ok

    def _release_and_home(self, face_down):
        """
        Detach and open at the release point, retreat, then fold, waist home and arm home
        """
        self.get_logger().info(f"14. DETACH ({self.detach_pub.topic_name} + /gripper/{self.side}/detach) + opening the gripper")
        if not self.dry_run:
            self.detach_pub.publish(Empty())
            self.detach_all_pub.publish(Empty())
            if self.use_moveit and self.target:
                self.mi.detach(self.side)                   # back into the world, on the table
        self._pause(0.3)
        self.gripper(GRIPPER_OPEN, 1.0, "15. apri gripper")
        self._pause(1.0)
        self._retreat_after_release()
        # with the fingers at GRIPPER_OPEN (37 mm) the right arm home pose touches the table top (valid at
        # 26 mm or less): like step R7b, close a little BEFORE going back
        self.gripper(min(GRIPPER_OPEN, 0.026), 1.0, "15c. richiudo un poco le dita per tornare a casa")
        # AFTER the retreat the scene is aligned with reality: where the object REALLY is on the table
        self._sync_placed_pose(final=False)
        # a single goto() jump from the release to home fails like step 10-13 (INVALID_MOTION_PLAN): always the
        # 3-step decomposition; arm()/waist() already use the smaller MoveIt groups
        if not self.arm(CARRY_ARM, 2.5, "16. braccio raccolto"):
            return False
        if not self.waist(HOME_WAIST, 3.0, "17. vita a casa"):
            return False
        if not self.arm(HOME_ARM, 2.5, "18. braccio a casa"):
            return False
        self._sync_placed_pose(final=True)
        self._step_forward_undo()
        self.get_logger().info("Sequence completed.")
        return True


def main(args=None):
    """
    Run the pick sequence; exit code 0 on success, 1 if stopped before the release (read by library_pipeline)
    """
    rclpy.init(args=args)
    node = PickTestBook()
    try:
        ok = node.run() is True      # False/None = stopped before the release (exit 1 for library_pipeline)
    except KeyboardInterrupt:
        ok = False
    finally:
        try:
            node.set_auto_attach(True)       # the teleop finds it on again
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
