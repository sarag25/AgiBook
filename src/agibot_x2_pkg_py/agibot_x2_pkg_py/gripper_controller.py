#!/usr/bin/env python3
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