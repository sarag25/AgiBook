#!/usr/bin/env python3
"""
GraspManagerNode - interfaccia UNICA di attach/detach guidata dai contatti
(2026-09-06, TODO "Implementare detach e attach joint per afferrare libri
con collision").

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

Contatti: legge /contact_left_tcp e /contact_right_tcp (sensori sulle DITA,
control_file.gazebo) e logga "Contatto <lato>: <modello>" ad ogni cambio di
modello toccato (poi al massimo ogni log_period). Il modello e' quello, fra
collision1/collision2 ('modello::link::collision'), che NON e' il robot
(robot_model): il vecchio filtro "tcp" not in nome non selezionava piu'
niente da quando i sensori stanno sulle dita.

Solo la mano DESTRA puo' agganciare: tutti i DetachableJoint puntano a
right_gripper_left_finger_link. I contatti del dito sinistro vengono
loggati; un attach sinistro produce un errore esplicito (per estendere:
secondo blocco <plugin> per entita' con child_link
left_gripper_left_finger_link, vedi PickAndPlace.md).

auto_attach (default False): se True, quando il dito destro tocca
un'entita' della scena con le dita in chiusura (posizione <=
auto_attach_max_opening, velocita' <= 0) l'attach parte da solo, con un
cooldown dopo ogni detach. Tenuto spento: rischia di incollare l'oggetto
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
from ros_gz_interfaces.msg import Contacts

class GraspManagerNode(Node):

    def __init__(self):
        super().__init__('grasp_manager')
        self.declare_parameter('scene', 'grasp_test')
        self.declare_parameter('robot_model', 'mogi_arm')
        # wall-clock: giudica "fresco" un contatto in arrivo indipendentemente
        # dall'RTF (a RTF 10% i 50 Hz del sensore diventano 5 Hz reali)
        self.declare_parameter('contact_timeout', 2.0)
        self.declare_parameter('log_period', 1.0)
        self.declare_parameter('auto_attach', False)
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
                    'In attesa di contatto...'
                )

        # Stato degli oggetti rilevati nei TCP
        self.detected_object_left = None
        self.detected_object_right = None
        self.attached_object_left = None
        self.attached_object_right = None

        # Subscriber ai sensori di contatto
        self.create_subscription(Contacts, '/contact_left_tcp', self.left_contact_cb, 10)
        self.create_subscription(Contacts, '/contact_right_tcp', self.right_contact_cb, 10)

        # Topic Globali per comandi manuali o da Behavior Tree / State Machine
        self.create_subscription(String, '/gripper/right/attach', self.attach_right_cb, 10)
        self.create_subscription(Empty, '/gripper/right/detach', self.detach_right_cb, 10)

        # Publisher dinamici verso Gazebo
        self.gz_publishers = {}

    def left_contact_cb(self, msg):
        # Estrae il nome del libro toccato dal messaggio Contacts
        for contact in msg.contacts:
            # Filtra collision1 e collision2 per trovare l'oggetto non-robot
            col1 = contact.collision1.name
            col2 = contact.collision2.name

            self.get_logger().info(
                        'Contatto'
                    )
            
            # Identifica l'oggetto (es. "gt_it", "gt_hunger", ecc.)
            target = col1 if "tcp" not in col1 else col2
            self.detected_object_left = target.split("::")[0] # Prende il nome del modello

    def right_contact_cb(self, msg):
        # Estrae il nome del libro toccato dal messaggio Contacts
        for contact in msg.contacts:
            # Filtra collision1 e collision2 per trovare l'oggetto non-robot
            col1 = contact.collision1.name
            col2 = contact.collision2.name

            self.get_logger().info(
                        'Contatto'
                    )
            
            # Identifica l'oggetto (es. "gt_it", "gt_hunger", ecc.)
            target = col1 if "tcp" not in col1 else col2
            self.detected_object_right = target.split("::")[0] # Prende il nome del modello

    def attach_right_cb(self, msg):
        # Se viene specificato un oggetto lo usa, altrimenti usa quello rilevato via collisione
        target_book = msg.data if msg.data else self.detected_object_right

        self.get_logger().info(
                    'Attaccando'
                )
        
        if target_book:
            topic_name = f'/{target_book}/attach'
            if topic_name not in self.gz_publishers:
                self.gz_publishers[topic_name] = self.create_publisher(Empty, topic_name, 10)
            
            self.gz_publishers[topic_name].publish(Empty())
            self.attached_object_right = target_book
            self.get_logger().info(f'Attached automaticamente: {target_book}')

    def detach_right_cb(self, msg):
        self.get_logger().info(
                    'Staccando'
                )

        if self.attached_object_right:
            topic_name = f'/{self.attached_object_right}/detach'
            if topic_name not in self.gz_publishers:
                self.gz_publishers[topic_name] = self.create_publisher(Empty, topic_name, 10)
                
            self.gz_publishers[topic_name].publish(Empty())
            self.get_logger().info(f'Detached: {self.attached_object_right}')
            self.attached_object_right = None

def main(args=None):
    rclpy.init(args=args)
    node = GraspManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



'''
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock  # o std_msgs.msg.Bool in base alla versione del bridge
from std_msgs.msg import Bool, Empty

class BookGrabberNode(Node):

    def __init__(self):
        super().__init__('book_grabber_node')

        # Stato interno per evitare invii continui dello stesso comando
        self.is_attached = False

        # Subscriber: ascolta se il tcp ha toccato il libro
        self.touch_sub = self.create_subscription(
            Bool, '/contact_right_tcp', self.touch_callback, 10
        )

        # Publisher: invia il segnale di aggancio/sgancio al plugin Gazebo
        self.detach_pub = self.create_publisher(Empty, '/right_hand/detach', 10)

        self.get_logger().info(
            'Nodo BookGrabber avviato. In attesa di contatto con il libro...'
        )

    def touch_callback(self, msg: Bool):
        # Se il sensore rileva il contatto e il libro non è ancora agganciato
        if msg.data and not self.is_attached:
            self.get_logger().info(
                'Contatto rilevato con il libro! Invio comando di aggancio...'
            )
            self.toggle_attach()
            self.is_attached = True

    def toggle_attach(self):
        # Il plugin DetachableJoint commuta lo stato (Attach/Detach) con un messaggio Empty
        msg = Empty()
        self.detach_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BookGrabberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
