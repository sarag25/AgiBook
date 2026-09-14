#!/usr/bin/env python3
"""
GraspManagerNode - interfaccia UNICA di attach/detach guidata dai contatti
(2026-09-06, TODO "Implementare detach e attach joint per afferrare libri
con collision"; ripristinato il 2026-09-13 dopo un merge che aveva lasciato
un ibrido senza import).

Il plugin gz DetachableJoint vuole la coppia (oggetto, dito) fissa: c'e' un
plugin per entita' (nel modello del libro, vedi rviz_gaz_control.launch.py
_test_book_urdf) e i suoi topic /<nome>/attach|detach|state vengono
bridgiati automaticamente (bridge_config). Questo nodo nasconde tutto cio'
dietro tre topic globali:

  SUB /gripper/right/attach    std_msgs/String  data = nome entita' (es.
                               'gt_it') oppure '' = l'ultima entita' della
                               scena che il dito ha TOCCATO negli ultimi
                               contact_timeout secondi
  SUB /gripper/right/detach    std_msgs/Empty   stacca l'entita' agganciata
                               (nessuna nota -> detach a tutte, con warn)
  PUB /gripper/right/attached  std_msgs/String  latched: entita' agganciata
                               o '' (letto da chi vuole conferma)

NON si crea il giunto a runtime spawnando un <joint> SDF con SpawnEntity
(tentativo del 2026-09-10): gz-sim accetta come entita' solo modelli/luci,
un giunto fra due modelli diversi non e' spawnabile - il plugin
DetachableJoint per entita' e' l'unica strada supportata.

Contatti: legge /contact_left_tcp e /contact_right_tcp (sensori sulle DITA,
control_file.gazebo) e logga "Contatto <lato>: <modello>" ad ogni cambio di
modello toccato (poi al massimo ogni log_period). Il modello e' quello, fra
collision1/collision2 ('modello::link::collision'), che NON e' il robot
(robot_model).

Solo la mano DESTRA puo' agganciare: tutti i DetachableJoint puntano a
right_gripper_left_finger_link. I contatti del dito sinistro vengono
loggati; un attach sinistro produce un errore esplicito.

auto_attach (default False): se True, quando il dito destro tocca
un'entita' della scena con le dita in chiusura l'attach parte da solo, con
un cooldown dopo ogni detach. Tenuto spento: rischia di incollare l'oggetto
sfiorato durante l'avvicinamento.

Avviato da rviz_gaz_control.launch.py (arg grasp_manager:=true, param
scene). A mano:
  ros2 run agibot_x2_pkg_py gripper_controller --ros-args -p scene:=grasp_test
"""

import time
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty, String
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts

from agibot_x2_pkg.book_placer import entity_topics, test_entities

CONTACT_TOPIC = {'left': '/contact_left_tcp', 'right': '/contact_right_tcp'}
FINGER_JOINT = {'left': 'left_gripper_left_finger_joint',
                'right': 'right_gripper_left_finger_joint'}
# child_link di TUTTI i DetachableJoint = right_gripper_left_finger_link
ATTACH_SIDES = ('right',)
SIDES = ('left', 'right')


class GraspManagerNode(Node):

    def __init__(self):
        super().__init__('grasp_manager')
        self.declare_parameter('scene', 'grasp_test')
        self.declare_parameter('robot_model', 'mogi_arm')
        # wall-clock: giudica "fresco" un contatto in arrivo indipendentemente
        # dall'RTF (a RTF 10% i 50 Hz del sensore diventano 5 Hz reali)
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
        self.attach_pub = {n: self.create_publisher(Empty, entity_topics(n)['attach'], 10)
                           for n in self.entities}
        self.detach_pub = {n: self.create_publisher(Empty, entity_topics(n)['detach'], 10)
                           for n in self.entities}
        self.entity_state = {n: 'sconosciuto' for n in self.entities}
        for n in self.entities:
            self.create_subscription(String, entity_topics(n)['state'],
                                     partial(self._state_cb, n), 10)

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.last_contact = {s: None for s in SIDES}     # (modello, t) qualsiasi != robot
        self.last_grabbable = {s: None for s in SIDES}   # (entita', t) solo entita' della scena
        self._last_log = {s: 0.0 for s in SIDES}
        self.attached = {s: None for s in SIDES}
        self.attached_pub = {s: self.create_publisher(String, f'/gripper/{s}/attached', latched)
                             for s in SIDES}
        self.finger = {s: (None, None) for s in SIDES}   # (posizione, velocita') dito
        self._last_auto = {s: 0.0 for s in SIDES}
        for s in SIDES:
            self.create_subscription(Contacts, CONTACT_TOPIC[s], partial(self._contact_cb, s), 10)
            self.create_subscription(String, f'/gripper/{s}/attach', partial(self._attach_cb, s), 10)
            self.create_subscription(Empty, f'/gripper/{s}/detach', partial(self._detach_cb, s), 10)
            self._publish_attached(s)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        self.get_logger().info(
            f"GraspManager scena={self.scene} entita'={self.entities} "
            f"auto_attach={self.auto_attach} - comandi: /gripper/right/attach "
            f"(String: nome o ''), /gripper/right/detach (Empty)")

    # ─── contatti ────────────────────────────────────────────────────────

    def _other_model(self, contact):
        """Modello che NON e' il robot fra collision1/collision2
        ('modello::link::collision'); None se e' un auto-contatto del robot."""
        for ent in (contact.collision1, contact.collision2):
            model = ent.name.split('::')[0]
            if model and model != self.robot_model:
                return model
        return None

    def _contact_cb(self, side, msg):
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
                    f"Contatto {side}: {model}" + ("" if grabbable else " (non afferrabile)"))
            if self.auto_attach and grabbable and side in ATTACH_SIDES and model.startswith('gt_'):
                self._maybe_auto_attach(side, model, now)

    def _joint_cb(self, msg):
        for s in SIDES:
            if FINGER_JOINT[s] in msg.name:
                i = msg.name.index(FINGER_JOINT[s])
                vel = msg.velocity[i] if i < len(msg.velocity) else None
                self.finger[s] = (msg.position[i], vel)

    def _maybe_auto_attach(self, side, model, now):
        pos, vel = self.finger[side]
        closing = (pos is not None and pos <= self.auto_max_opening
                   and (vel is None or vel <= 1e-4))
        if (self.attached[side] is None and closing
                and now - self._last_auto[side] > self.auto_cooldown):
            self._last_auto[side] = now
            self.get_logger().info(
                f"auto_attach {side}: dito a {pos*1000:.1f} mm in chiusura su {model}")
            self._attach_cb(side, String(data=model))

    # ─── attach / detach globali ─────────────────────────────────────────

    def _fresh_contact(self, side):
        lg = self.last_grabbable[side]
        if lg and time.monotonic() - lg[1] < self.contact_timeout:
            return lg[0]
        return None

    def _attach_cb(self, side, msg):
        if side not in ATTACH_SIDES:
            self.get_logger().error(
                f"attach {side}: nessun DetachableJoint verso il dito {side} "
                "(child_link = right_gripper_left_finger_link). Vedi PickAndPlace.md")
            return
        requested = msg.data.strip()
        fresh = self._fresh_contact(side)
        name = requested or fresh
        if not name:
            self.get_logger().warn(
                f"attach {side}: nessun nome e nessun contatto con un'entita' della "
                f"scena negli ultimi {self.contact_timeout:.0f}s - ignorato")
            return
        if name not in self.attach_pub:
            self.get_logger().error(
                f"attach {side}: '{name}' non e' nella scena {self.scene} {self.entities}")
            return
        if requested and fresh and fresh != requested:
            self.get_logger().warn(
                f"attach {side}: richiesto {requested} ma il dito sta toccando {fresh}")
        elif requested and not fresh:
            self.get_logger().warn(
                f"attach {side}: {requested} senza contatto recente del dito - "
                "il giunto si crea comunque, ma l'oggetto potrebbe non essere fra le dita")
        self.attach_pub[name].publish(Empty())
        self.attached[side] = name
        self._publish_attached(side)
        self.get_logger().info(f"ATTACH {side} -> {name} ({entity_topics(name)['attach']})")

    def _detach_cb(self, side, _msg):
        name = self.attached[side]
        targets = [name] if name else list(self.detach_pub)
        if not name:
            self.get_logger().warn(
                f"detach {side}: nessun aggancio registrato, detach a tutte le entita'")
        for n in targets:
            self.detach_pub[n].publish(Empty())
        self.attached[side] = None
        self._last_auto[side] = time.monotonic()
        self._publish_attached(side)
        self.get_logger().info(f"DETACH {side} -> {targets}")

    def _state_cb(self, name, msg):
        if msg.data != self.entity_state[name]:
            self.get_logger().info(f"Stato {name}: {msg.data}")
        self.entity_state[name] = msg.data
        if msg.data == 'detached':
            for s, n in self.attached.items():
                if n == name:
                    self.attached[s] = None
                    self._publish_attached(s)

    def _publish_attached(self, side):
        self.attached_pub[side].publish(String(data=self.attached[side] or ''))


def main(args=None):
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
