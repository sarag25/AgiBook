#!/usr/bin/env python3
"""
ROS 2 node (grasp_manager) that attaches/detaches objects to the gripper fingers, driven by contacts.
Hides the per-entity gz DetachableJoint topics (/<name>/attach|detach|state, bridged via bridge_config)
behind /gripper/<side>/attach (String: entity or '' = last touched), /gripper/<side>/detach (Empty)
and the latched /gripper/<side>/attached. Joints cannot be spawned at runtime (gz-sim only spawns
models/lights), so a DetachableJoint plugin per entity is the only supported way.
  ros2 run agibot_x2_pkg_py gripper_controller --ros-args -p scene:=grasp_test
"""

import time
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty, String, Bool
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts

from agibot_x2_pkg.book_placer import entity_topics, test_entities

CONTACT_TOPIC = {'left': '/contact_left_tcp', 'right': '/contact_right_tcp'}
FINGER_JOINT = {'left': 'left_gripper_left_finger_joint',
                'right': 'right_gripper_left_finger_joint'}
ATTACH_SIDES = ('left', 'right')   # DetachableJoints to right_gripper_left_finger_link and to the left hand
SIDES = ('left', 'right')


class GraspManagerNode(Node):
    """
    Contact-driven attach/detach manager for the scene entities
    """

    def __init__(self):
        """
        Declare parameters, create DetachableJoint publishers and contact/joint subscriptions
        """
        super().__init__('grasp_manager')
        self.declare_parameter('scene', 'grasp_test')
        self.declare_parameter('robot_model', 'mogi_arm')
        # wall-clock s: freshness independent of RTF (at RTF 10% the 50 Hz sensor gives 5 Hz)
        self.declare_parameter('contact_timeout', 2.0)
        self.declare_parameter('log_period', 1.0)
        self.declare_parameter('auto_attach', True)
        self.declare_parameter('auto_attach_max_opening', 0.030)
        self.declare_parameter('auto_attach_cooldown', 3.0)

        self.scene = self.get_parameter('scene').value
        self.robot_model = self.get_parameter('robot_model').value
        self.contact_timeout = float(self.get_parameter('contact_timeout').value)
        self.log_period = float(self.get_parameter('log_period').value)
        self.auto_attach = bool(self.get_parameter('auto_attach').value)
        self.auto_max_opening = float(self.get_parameter('auto_attach_max_opening').value)
        self.auto_cooldown = float(self.get_parameter('auto_attach_cooldown').value)

        self.entities = [e[0] for e in test_entities(self.scene)]
        _sfx = {'right': '', 'left': '_left'}
        self.attach_pub = {s: {n: self.create_publisher(Empty, entity_topics(n)['attach' + _sfx[s]], 10)
                               for n in self.entities} for s in ATTACH_SIDES}
        self.detach_pub = {s: {n: self.create_publisher(Empty, entity_topics(n)['detach' + _sfx[s]], 10)
                               for n in self.entities} for s in ATTACH_SIDES}
        self.entity_state = {n: 'sconosciuto' for n in self.entities}
        for n in self.entities:
            for s in ATTACH_SIDES:
                self.create_subscription(String, entity_topics(n)['state' + _sfx[s]],
                                         partial(self._state_cb, n), 10)

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.last_contact = {s: None for s in SIDES}     # (model, t) any model but the robot
        self.last_grabbable = {s: None for s in SIDES}   # (entity, t) scene entities only
        self._last_log = {s: 0.0 for s in SIDES}
        self.attached = {s: None for s in SIDES}
        self.attached_pub = {s: self.create_publisher(String, f'/gripper/{s}/attached', latched)
                             for s in SIDES}
        self.finger = {s: (None, None) for s in SIDES}   # finger (position, velocity)
        self._last_auto = {s: 0.0 for s in SIDES}
        self._finger_hist = {s: [] for s in SIDES}
        for s in SIDES:
            self.create_subscription(Contacts, CONTACT_TOPIC[s], partial(self._contact_cb, s), 10)
            self.create_subscription(String, f'/gripper/{s}/attach', partial(self._attach_cb, s), 10)
            self.create_subscription(Empty, f'/gripper/{s}/detach', partial(self._detach_cb, s), 10)
            self._publish_attached(s)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        # runtime switch: pick_test_book turns auto-attach off while it attaches explicitly
        self.create_subscription(Bool, '/gripper/auto_attach', self._auto_attach_cb, 10)

        self.get_logger().info(
            f"GraspManager scene={self.scene} entities={self.entities} "
            f"auto_attach={self.auto_attach} - commands: /gripper/right/attach "
            f"(String: name or ''), /gripper/right/detach (Empty)")

    def _other_model(self, contact):
        """
        Model of collision1/collision2 ('model::link::collision') that is not the robot, None on self-contact
        """
        for ent in (contact.collision1, contact.collision2):
            model = ent.name.split('::')[0]
            if model and model != self.robot_model:
                return model
        return None

    def _contact_cb(self, side, msg):
        """
        Record the touched model, log changes (at most every log_period) and maybe auto-attach
        """
        now = time.monotonic()
        for c in msg.contacts:
            model = self._other_model(c)
            if model is None:
                continue
            prev = self.last_contact[side]
            self.last_contact[side] = (model, now)
            grabbable = model in self.entity_state
            if grabbable:
                self.last_grabbable[side] = (model, now)
            if prev is None or prev[0] != model or now - self._last_log[side] >= self.log_period:
                self._last_log[side] = now
                self.get_logger().info(
                    f"Contact {side}: {model}" + ("" if grabbable else " (not grabbable)"))
            if self.auto_attach and grabbable and side in ATTACH_SIDES and model.startswith('gt_'):
                self._maybe_auto_attach(side, model, now)

    def _joint_cb(self, msg):
        """
        Store finger position/velocity and a 0.6 s position history
        """
        now = time.monotonic()
        for s in SIDES:
            if FINGER_JOINT[s] in msg.name:
                i = msg.name.index(FINGER_JOINT[s])
                vel = msg.velocity[i] if i < len(msg.velocity) else None
                self.finger[s] = (msg.position[i], vel)
                # position history, because /joint_states velocity has spurious spikes while the arm moves
                h = self._finger_hist[s]
                h.append((now, msg.position[i]))
                while h and now - h[0][0] > 0.6:
                    h.pop(0)

    def _finger_closing(self, side, min_travel=1e-3, window=0.5):
        """
        True if the finger closed by at least min_travel (m) within the last window (s)
        """
        h = self._finger_hist[side]
        if len(h) < 2:
            return False
        now, pos_now = h[-1]
        older = [p for t, p in h if now - t >= 0.15 and now - t <= window]
        return bool(older) and (max(older) - pos_now) >= min_travel

    def _auto_attach_cb(self, msg):
        """
        Turn auto-attach on/off from /gripper/auto_attach
        """
        if bool(msg.data) != self.auto_attach:
            self.auto_attach = bool(msg.data)
            self.get_logger().info(f"auto_attach {'ON' if self.auto_attach else 'OFF'} (from /gripper/auto_attach)")

    def _maybe_auto_attach(self, side, model, now):
        """
        Attach `model` only if the fingers are really closing, outside the cooldown
        During the approach the fingers are still and below 30 mm, so opening alone would glue
        a neighbouring book brushed by the sensor finger.
        """
        pos, vel = self.finger[side]
        closing = (pos is not None and pos <= self.auto_max_opening
                   and self._finger_closing(side))          # real closing, not velocity noise
        if (self.attached[side] is None and closing
                and now - self._last_auto[side] > self.auto_cooldown):
            self._last_auto[side] = now
            self.get_logger().info(
                f"auto_attach {side}: finger at {pos*1000:.1f} mm closing on {model}")
            self._attach_cb(side, String(data=model))

    def _fresh_contact(self, side):
        """
        Scene entity touched within contact_timeout, or None
        """
        lg = self.last_grabbable[side]
        if lg and time.monotonic() - lg[1] < self.contact_timeout:
            return lg[0]
        return None

    def _attach_cb(self, side, msg):
        """
        Attach the requested entity (or the last touched one) to the finger of `side`
        """
        if side not in ATTACH_SIDES:
            self.get_logger().error(
                f"attach {side}: side not handled")
            return
        requested = msg.data.strip()
        fresh = self._fresh_contact(side)
        name = requested or fresh
        if not name:
            self.get_logger().warn(
                f"attach {side}: no name and no contact with a scene entity "
                f"in the last {self.contact_timeout:.0f}s - ignored")
            return
        if name not in self.attach_pub[side]:
            self.get_logger().error(
                f"attach {side}: '{name}' is not in scene {self.scene} {self.entities}")
            return
        if requested and fresh and fresh != requested:
            self.get_logger().warn(
                f"attach {side}: requested {requested} but the finger is touching {fresh}")
        elif requested and not fresh:
            self.get_logger().warn(
                f"attach {side}: {requested} without recent finger contact - "
                "the joint is created anyway, but the object may not be between the fingers")
        self.attach_pub[side][name].publish(Empty())
        self.attached[side] = name
        self._publish_attached(side)
        self.get_logger().info(f"ATTACH {side} -> {name} ({self.attach_pub[side][name].topic_name})")

    def _detach_cb(self, side, _msg):
        """
        Detach the attached entity (all entities, with a warning, if none is known)
        """
        name = self.attached[side]
        targets = [name] if name else list(self.detach_pub[side])
        if not name:
            self.get_logger().warn(
                f"detach {side}: no attach recorded, detaching all entities")
        for n in targets:
            self.detach_pub[side][n].publish(Empty())
        self.attached[side] = None
        self._last_auto[side] = time.monotonic()
        self._publish_attached(side)
        self.get_logger().info(f"DETACH {side} -> {targets}")

    def _state_cb(self, name, msg):
        """
        Track DetachableJoint state; on 'detached' start the auto-attach cooldown
        """
        if msg.data != self.entity_state[name]:
            self.get_logger().info(f"State {name}: {msg.data}")
        self.entity_state[name] = msg.data
        if msg.data == 'detached':
            # cooldown: the book just released is still between the fingers and would re-attach
            now = time.monotonic()
            for s in SIDES:
                self._last_auto[s] = now
            for s, n in self.attached.items():
                if n == name:
                    self.attached[s] = None
                    self._publish_attached(s)

    def _publish_attached(self, side):
        """
        Publish the attached entity of `side` ('' if none)
        """
        self.attached_pub[side].publish(String(data=self.attached[side] or ''))


def main(args=None):
    """
    Spin the grasp manager node
    """
    rclpy.init(args=args)
    node = GraspManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
