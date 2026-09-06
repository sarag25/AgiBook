#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from ros_gz_interfaces.msg import Contacts

class GraspManagerNode(Node):
    def __init__(self):
        super().__init__('grasp_manager_node')

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

'''