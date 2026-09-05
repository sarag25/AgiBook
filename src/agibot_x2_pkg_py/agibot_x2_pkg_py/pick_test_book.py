"""
pick_test_book: presa e trasporto AUTOMATICI di un libro fisico di test,
stile demo MOGI-ROS (avvicinamento -> chiusura -> attach del DetachableJoint
-> sollevamento -> trasporto -> rilascio), con le pose del braccio calcolate
dalla cinematica inversa (arm_kinematics.py) a partire dalla posizione nota
del libro (book_placer.TEST_BOOKS) - niente angoli hardcoded.

Uso (con rviz_gaz_control.launch.py attivo e controller su):
    ros2 run agibot_x2_pkg_py pick_test_book                 # libro IT (scena grasp_test, default)
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=hunger   # hunger | it | ballata | alba | pen | mug
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p scene:=full -p book:=emma
    ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p dry_run:=true   # solo IK, niente movimento

Sequenza:
  1. apre il gripper destro
  2. IK: grasp (dorso + grasp_depth), poi pre-grasp/lift/retreat seminati
     dalla soluzione grasp (continuita' nello spazio giunti)
  3. vita+braccio a pre-grasp, braccio a grasp
  4. chiude le dita allo spessore del libro, ATTACH
  5. sfila il libro all'indietro (stessa quota), posa di trasporto (braccio
     raccolto: non spazza la libreria ruotando la vita)
  6. vita a -90 gradi (verso il tavolo), busto in avanti, braccio teso:
     il libro sporge oltre il bordo del tavolo -> DETACH, apre -> cade sul piano
  7. torna a casa

Limite noto: il tavolo e' ai margini della portata (bordo vicino a
y=-0.45, TCP a y~-0.45 col braccio teso): il libro viene LASCIATO CADERE
sul bordo, non appoggiato. Vedi Gazebo.md.
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

from agibot_x2_pkg.book_placer import catalog_entry, collision_size, test_entities, ROBOT_SPAWN_X, SHELF_SURFACES_Z
from agibot_x2_pkg_py.arm_kinematics import (
    ArmKinematics, ARM_JOINTS, WAIST_JOINTS, GRIPPER_OPEN, grasp_opening)

GRIPPER_JOINTS = ["right_gripper_left_finger_joint", "right_gripper_right_finger_joint"]

# Posa di trasporto (braccio raccolto davanti al petto): TCP a ~0.28 m dal
# busto, sotto il fronte dello scaffale anche a vita ruotata - verificato
# con la FK (arm_kinematics) il 2026-08-30.
CARRY_ARM = [-0.5, 0.0, 0.0, -2.2, 0.0]
# Rilascio sul tavolo: vita a -90 gradi, busto in avanti al limite,
# braccio teso orizzontale -> TCP a y~-0.45 (bordo del tavolo), il libro
# afferrato per il dorso sporge di altri ~8 cm oltre.
DROP_WAIST = [-1.57, 0.31]
DROP_ARM = [-1.57, 0.0, 0.0, 0.0, 0.0]
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

        self.book = self.get_parameter("book").value
        self.robot_x = float(self.get_parameter("robot_x").value)
        self.robot_y = float(self.get_parameter("robot_y").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.kin = ArmKinematics.from_package()

        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       "/right_arm_controller/follow_joint_trajectory")
        self.waist_client = ActionClient(self, FollowJointTrajectory,
                                         "/waist_controller/follow_joint_trajectory")
        self.gripper_client = ActionClient(self, FollowJointTrajectory,
                                           "/right_gripper_controller/follow_joint_trajectory")

        scene = self.get_parameter("scene").value
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

    def arm(self, q, duration, label):
        return self._move(self.arm_client, ARM_JOINTS, q, duration, label)

    def waist(self, q, duration, label):
        return self._move(self.waist_client, WAIST_JOINTS, q, duration, label)

    def gripper(self, opening, duration, label):
        return self._move(self.gripper_client, GRIPPER_JOINTS, [opening, opening], duration, label)

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

        t0 = time.time()
        # 1) grasp: unica IK "libera" (vita pitch inclusa) - fissa il ramo
        q_grasp, w_grasp, e1 = self.kin.ik(rel(g), optimize_waist="pitch")
        # 2) pre-grasp: stesso ramo (vita fissa alla soluzione grasp)
        q_pre, _w, e2 = self.kin.ik(rel(g - [back, 0, 0]), q0=q_grasp, waist=w_grasp)
        # 3) percorsi cartesiani: pre->grasp (avvicinamento rettilineo),
        #    grasp->lift, lift->retreat (uscita rettilinea dallo scaffale)
        path_in, e3 = self.kin.ik_path(rel(g - [back, 0, 0]), rel(g), 4, q0=q_pre, waist=w_grasp)
        # Niente "lift" nello scaffale (2026-08-30): in posa di presa il
        # gomito e' al limite (braccio teso) e alzare di 4 cm nello stesso
        # ramo non e' possibile - il risolutore saltava a un ramo contorto
        # (spalla_yaw +107 gradi). Il libro e' incollato al dito: lo si
        # SFILA all'indietro alla stessa quota (2 mm sopra il ripiano, come
        # e' stato spawnato) e si alza solo fuori dallo scaffale, con la
        # posa di trasporto.
        path_out, e5 = self.kin.ik_path(rel(g), rel(g + [-retreat, 0, 0]), 5,
                                        q0=path_in[-1], waist=w_grasp)
        errs = dict(grasp=e1, pre=e2, approach=e3, retreat=e5)
        self.get_logger().info(
            f"IK in {time.time()-t0:.1f}s - errori max: " +
            ", ".join(f"{k} {v*1000:.1f}mm" for k, v in errs.items()))
        if max(errs.values()) > 0.02:
            self.get_logger().error("IK con errore > 2 cm: libro fuori portata da questa posizione, mi fermo.")
            return
        w_pre = w_grasp

        # chiude sullo spessore della COLLISION (piu' stretta della mesh per i libri)
        opening = grasp_opening(collision_size(self.kind, self.key)[1])

        ok = (
            self.gripper(GRIPPER_OPEN, 1.0, "1. apri gripper")
            and self.waist(list(w_grasp), 2.0, "2. vita (pitch) per la presa")
            and self.arm(q_pre, 3.0, "3. braccio pre-grasp")
            and self.arm(path_in, 2.5, "4-5. avvicinamento rettilineo fino al grasp")
            and self.gripper(opening, 1.5, f"6. chiudo le dita a {opening*1000:.1f} mm/lato")
        )
        if not ok:
            return
        self._pause(0.5)
        self.get_logger().info(f"7. ATTACH /{self.entity}/attach")
        if not self.dry_run:
            self.attach_pub.publish(Empty())
        self._pause(0.5)

        ok = (
            self.arm(path_out, 3.0, "8-9. sfilo il libro all'indietro (rettilineo, fuori dallo scaffale)")
            and self.arm(CARRY_ARM, 2.5, "10. posa di trasporto")
            and self.waist([DROP_WAIST[0], 0.0], 3.0, "11. vita verso il tavolo")
            and self.waist(DROP_WAIST, 1.5, "12. busto in avanti")
            and self.arm(DROP_ARM, 2.5, "13. braccio teso sopra il bordo del tavolo")
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
