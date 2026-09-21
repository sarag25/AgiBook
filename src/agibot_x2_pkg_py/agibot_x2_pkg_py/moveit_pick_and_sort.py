#!/usr/bin/env python3
"""
ROS 2 node that clears non-book objects to the table and reorders the books with MoveIt (move_action).
Reads the detections and the sort plan JSON (/tmp/x2_detections.json, /tmp/x2_sort_plan.json).
"""

import json
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene, CollisionObject, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from agibot_x2_pkg.book_placer import (
    test_entities, catalog_entry, SHELF_SURFACES_Z, BOOKS_FRONT_X
)

SHELF_FRONT_X = 0.261
TABLE_RELEASE_X = 0.21
TABLE_RELEASE_Y = -0.37
TABLE_TOP_Z = 0.75

SLOT_WIDTH = 0.055
SLOT_START_Y = -0.22

TCP_X_OFFSET = 0.08   # X distance from the wrist (ee_link) to the finger grasp center

# neutral orientation for shelf approach and table release
GRASP_ORIENTATION_SHELF = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
RELEASE_ORIENTATION_TABLE = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


def enrich_detections(detections, scene="grasp_test"):
    """
    Add world pose and size to detections missing them, matching each one to a scene test entity
    by title/color/class (fallback: first book or first decoration)
    """
    entities = test_entities(scene)
    enriched = []
    for d in detections:
        det = dict(d)
        if "world_x" in det and "world_y" in det and "z_bottom" in det and "z_top" in det:
            enriched.append(det)
            continue

        title = str(det.get("title", "")).lower()
        color = str(det.get("color", "")).lower()
        cls = str(det.get("class", "")).lower()
        slot = det.get("shelf_slot", -1)
        obj_id = det.get("id", -1)

        matched = None
        for name, key, kind, ex, ey in entities:
            if "hunger" in title or ("viola" in color and "hunger" in key):
                if "hunger" in key:
                    matched = (name, key, kind, ex, ey)
                    break
            elif "stephen" in title or "it" in title or ("bianco" in color and "it" in key):
                if "it" in key:
                    matched = (name, key, kind, ex, ey)
                    break
            elif "ballata" in title or "usignolo" in title or ("magenta" in color and "ballata" in key):
                if "ballata" in key:
                    matched = (name, key, kind, ex, ey)
                    break
            elif "alba" in title or ("rosso" in color and "alba" in key) or title == "1":
                if "alba" in key:
                    matched = (name, key, kind, ex, ey)
                    break
            elif not det.get("is_book", True) or cls in ["object", "decoration"]:
                if "pen" in key and (slot == 1 or str(obj_id) in ["5", "4"]):
                    matched = (name, key, kind, ex, ey)
                    break
                elif "globe" in key and (slot == 0 or str(obj_id) in ["6", "4"]):
                    matched = (name, key, kind, ex, ey)
                    break

        if not matched:
            if not det.get("is_book", True):
                decorations = [e for e in entities if e[2] == "decoration"]
                matched = decorations[0] if decorations else entities[-1]
            else:
                books = [e for e in entities if e[2] == "book"]
                matched = books[0] if books else entities[0]

        name, key, kind, ex, ey = matched
        cat = catalog_entry(kind, key)
        sx, sy, sz = cat["size"]
        z_shelf = SHELF_SURFACES_Z[0]

        det["entity_name"] = name
        det["key"] = key
        det["world_x"] = float(ex)
        det["world_y"] = float(ey)
        det["z_bottom"] = float(z_shelf)
        det["z_top"] = float(z_shelf + sz)
        det["length_m"] = float(sx)
        det["thickness_m"] = float(sy)
        det["height_m"] = float(sz)
        enriched.append(det)
    return enriched


class MoveItPickAndSort(Node):
    """
    Pick & place pipeline driven by MoveIt pose goals
    """

    def __init__(self):
        """
        Declare parameters and create the action clients and publishers
        """
        super().__init__("moveit_pick_and_sort")

        self.declare_parameter("detections_file", "/tmp/x2_detections.json")
        self.declare_parameter("sort_plan_file", "/tmp/x2_sort_plan.json")
        self.declare_parameter("scene", "grasp_test")
        self.declare_parameter("group_name", "right_arm_and_waist")
        self.declare_parameter("ee_link", "right_wrist_yaw_link")
        self.declare_parameter("gripper_controller_topic", "/right_gripper_controller/follow_joint_trajectory")
        self.declare_parameter("dry_run", False)

        self.detections_file = self.get_parameter("detections_file").value
        self.sort_plan_file = self.get_parameter("sort_plan_file").value
        self.scene = self.get_parameter("scene").value
        self.group_name = self.get_parameter("group_name").value
        self.ee_link = self.get_parameter("ee_link").value
        gripper_topic = self.get_parameter("gripper_controller_topic").value
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self._move_group_client = ActionClient(self, MoveGroup, "move_action")
        self._gripper_traj_client = ActionClient(self, FollowJointTrajectory, gripper_topic)

        self.attach_pub = self.create_publisher(String, "/gripper/right/attach", 10)
        self.detach_pub = self.create_publisher(Empty, "/gripper/right/detach", 10)
        self.auto_attach_pub = self.create_publisher(Bool, "/gripper/auto_attach", 10)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        self.get_logger().info(f"MoveIt Pick & Sort node started (Group: {self.group_name}, EE: {self.ee_link}).")

    def set_gripper(self, opening: float, duration: float = 0.5):
        """
        Send both right fingers to `opening` via the gripper trajectory controller
        """
        if self.dry_run:
            return True
        if not self._gripper_traj_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(f"Gripper controller on '{self._gripper_traj_client._action_name}' not responding.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "right_gripper_left_finger_joint",
            "right_gripper_right_finger_joint"
        ]
        pt = JointTrajectoryPoint()
        pt.positions = [float(opening), float(opening)]
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points.append(pt)

        fut = self._gripper_traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        return True

    def clear_all_objects_from_scene(self):
        """
        Reset the planning scene to avoid spurious collisions, keeping only the bookshelf back
        """
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        
        # bookshelf back panel
        shelf_obj = CollisionObject()
        shelf_obj.header.frame_id = "world"
        shelf_obj.id = "bookshelf_back"
        shelf_primitive = SolidPrimitive()
        shelf_primitive.type = SolidPrimitive.BOX
        shelf_primitive.dimensions = [0.05, 1.00, 1.50]
        shelf_pose = Pose()
        shelf_pose.position.x = 0.55
        shelf_pose.position.y = 0.0
        shelf_pose.position.z = 0.75
        shelf_pose.orientation.w = 1.0
        shelf_obj.primitives.append(shelf_primitive)
        shelf_obj.primitive_poses.append(shelf_pose)
        shelf_obj.operation = CollisionObject.ADD
        scene_msg.world.collision_objects.append(shelf_obj)

        self.scene_pub.publish(scene_msg)
        time.sleep(0.3)

    def plan_and_execute_pose(self, target_pose: PoseStamped, label: str):
        """
        Plan and execute a pose goal for ee_link, True on success
        """
        self.get_logger().info(
            f"MoveIt [{label}]: Target pos=({target_pose.pose.position.x:.3f}, "
            f"{target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f})"
        )

        if self.dry_run:
            return True

        if not self._move_group_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Action Server 'move_action' not available!")
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 10
        
        # longer planning time for the waist+arm chain
        goal.request.allowed_planning_time = 8.0
        
        goal.request.max_velocity_scaling_factor = 0.4
        goal.request.max_acceleration_scaling_factor = 0.4

        constraints = Constraints()
        
        # position constraint (5 cm box)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = target_pose.header.frame_id
        pos_constraint.link_name = self.ee_link

        tol_box = SolidPrimitive()
        tol_box.type = SolidPrimitive.BOX
        tol_box.dimensions = [0.05, 0.05, 0.05]

        box_pose = Pose()
        box_pose.position = target_pose.pose.position
        box_pose.orientation.w = 1.0

        pos_constraint.constraint_region.primitives.append(tol_box)
        pos_constraint.constraint_region.primitive_poses.append(box_pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        # orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = target_pose.header.frame_id
        orient_constraint.link_name = self.ee_link
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.5
        orient_constraint.absolute_y_axis_tolerance = 0.5
        orient_constraint.absolute_z_axis_tolerance = 0.5
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        goal.request.goal_constraints.append(constraints)

        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=9.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(f"MoveIt [{label}]: timeout while sending the goal.")
            return False

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f"MoveIt [{label}]: goal rejected by MoveIt.")
            return False

        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=12.0)

        if res_future.done() and res_future.result():
            status = res_future.result().status
            if status == 4:  # SUCCEEDED
                self.get_logger().info(f"MoveIt [{label}]: motion completed successfully.")
                return True

        self.get_logger().error(f"MoveIt [{label}]: planning or execution failed.")
        return False

    def execute_pick_and_place(self, item, dest_x, dest_y, dest_z, is_table=False):
        """
        Pre-grasp, open, grasp, close+attach, retreat, transport, detach+open
        """
        item_id = item.get("id", item.get("obj_id"))
        label = item.get("title") or item.get("key") or item.get("entity_name") or f"ID_{item_id}"

        self.get_logger().info(f"=== RUNNING PICK & PLACE: {label} ===")

        x = float(item["world_x"])
        y = float(item["world_y"])
        z = (float(item["z_bottom"]) + float(item["z_top"])) / 2.0

        # X at least 0.28 m to avoid self-collisions
        pre_pose = PoseStamped()
        pre_pose.header.frame_id = "world"
        pre_pose.pose.position.x = x - TCP_X_OFFSET - 0.12
        pre_pose.pose.position.y = y
        pre_pose.pose.position.z = z
        pre_pose.pose.orientation = GRASP_ORIENTATION_SHELF

        # 1. pre-grasp (gripper still closed/compact)
        if not self.plan_and_execute_pose(pre_pose, f"Pre-Grasp {label}"):
            return False

        # open the gripper only after the pre-grasp
        self.set_gripper(0.030, duration=0.5)
        time.sleep(0.3)

        # 2. approach (grasp)
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "world"
        grasp_pose.pose.position.x = x - TCP_X_OFFSET
        grasp_pose.pose.position.y = y
        grasp_pose.pose.position.z = z
        grasp_pose.pose.orientation = GRASP_ORIENTATION_SHELF

        if not self.plan_and_execute_pose(grasp_pose, f"Grasp {label}"):
            return False

        # 3. close and attach
        self.set_gripper(0.002, duration=0.5)
        attach_name = item.get("entity_name", f"obj{item_id}")
        self.attach_pub.publish(String(data=str(attach_name)))
        time.sleep(0.4)

        # 4. extraction
        retreat_pose = PoseStamped()
        retreat_pose.header.frame_id = "world"
        retreat_pose.pose.position.x = SHELF_FRONT_X - TCP_X_OFFSET - 0.05
        retreat_pose.pose.position.y = y
        retreat_pose.pose.position.z = z + 0.02
        retreat_pose.pose.orientation = GRASP_ORIENTATION_SHELF

        self.plan_and_execute_pose(retreat_pose, f"Estrazione {label}")

        # 5. transport to destination
        dest_pose = PoseStamped()
        dest_pose.header.frame_id = "world"
        dest_pose.pose.position.x = dest_x - TCP_X_OFFSET
        dest_pose.pose.position.y = dest_y
        dest_pose.pose.position.z = dest_z
        dest_pose.pose.orientation = RELEASE_ORIENTATION_TABLE if is_table else GRASP_ORIENTATION_SHELF

        self.plan_and_execute_pose(dest_pose, f"Trasporto {label}")

        # 6. detach and reopen fingers
        self.detach_pub.publish(Empty())
        time.sleep(0.3)
        self.set_gripper(0.030, duration=0.5)

        return True

    def run_pipeline(self):
        """
        Phase 1: non-book objects to the table; phase 2: books to their target slots
        """
        if not os.path.exists(self.detections_file):
            raise FileNotFoundError(f"File not found: {self.detections_file}")

        with open(self.detections_file, "r", encoding="utf-8") as f:
            raw_detections = json.load(f)

        detections = enrich_detections(raw_detections, scene=self.scene)
        self.clear_all_objects_from_scene()

        sort_plan = None
        if os.path.exists(self.sort_plan_file):
            with open(self.sort_plan_file, "r", encoding="utf-8") as f:
                sort_plan = json.load(f)

        det_map = {}
        for d in detections:
            det_map[d["id"]] = d
            det_map[str(d["id"])] = d

        # auto-attach off during the sequence to avoid accidental grabs
        self.auto_attach_pub.publish(Bool(data=False))

        # phase 1: clear non-book objects
        self.get_logger().info("=== START PHASE 1: CLEARING NON-BOOK OBJECTS ===")
        
        non_book_ids = []
        if sort_plan and "objects_to_table" in sort_plan:
            non_book_ids = [obj["obj_id"] for obj in sort_plan["objects_to_table"]]
        else:
            non_book_ids = [d["id"] for d in detections if not d.get("is_book", True) or d.get("class") in ["object", "decoration"]]

        for obj_id in non_book_ids:
            if obj_id in det_map:
                item = det_map[obj_id]
                self.execute_pick_and_place(
                    item, 
                    dest_x=TABLE_RELEASE_X, 
                    dest_y=TABLE_RELEASE_Y, 
                    dest_z=TABLE_TOP_Z + 0.12, 
                    is_table=True
                )

        # phase 2: reorder books
        if sort_plan and "insertion_order" in sort_plan:
            self.get_logger().info("=== START PHASE 2: REORDERING BOOKS ===")
            for ins in sort_plan["insertion_order"]:
                book_id = ins["obj_id"]
                target_slot = ins.get("target_slot", 0)
                
                if book_id in det_map:
                    book_item = det_map[book_id]
                    target_y = SLOT_START_Y + (target_slot * SLOT_WIDTH)
                    target_z = (book_item["z_bottom"] + book_item["z_top"]) / 2.0
                    target_x = BOOKS_FRONT_X

                    self.execute_pick_and_place(
                        book_item,
                        dest_x=target_x,
                        dest_y=target_y,
                        dest_z=target_z,
                        is_table=False
                    )

        # re-enable auto-attach
        self.auto_attach_pub.publish(Bool(data=True))
        self.get_logger().info("Procedure completed successfully.")


def main(args=None):
    """
    Run the pipeline once and shut down
    """
    rclpy.init(args=args)
    node = MoveItPickAndSort()
    try:
        node.run_pipeline()
    except Exception as e:
        node.get_logger().error(f"Error during execution: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()