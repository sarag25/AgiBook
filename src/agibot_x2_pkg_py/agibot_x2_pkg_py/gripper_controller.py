#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from ros_gz_interfaces.msg import Contacts, EntityFactory
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity

class GraspManagerNode(Node):
    def __init__(self):
        super().__init__('grasp_manager_node')

        self.get_logger().info(
                    'Nodo GraspManager avviato. In attesa di contatto...'
                )

        # Client per servizi di Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/world/bookshelf_world/create')
        self.remove_client = self.create_client(DeleteEntity, '/world/bookshelf_world/remove')

        # Stato degli oggetti rilevati nei TCP
        self.detected_object_left = None
        self.detected_object_right = None

        # Subscriber ai sensori di contatto
        self.create_subscription(Contacts, '/contact_left_tcp', self.left_contact_cb, 10)
        self.create_subscription(Contacts, '/contact_right_tcp', self.right_contact_cb, 10)

        # Topic Globali ROS2
        self.create_subscription(String, '/gripper/left/attach', self.attach_left_cb, 10)
        self.create_subscription(Empty, '/gripper/left/detach', self.detach_left_cb, 10)
        self.create_subscription(String, '/gripper/right/attach', self.attach_right_cb, 10)
        self.create_subscription(Empty, '/gripper/right/detach', self.detach_right_cb, 10)

        # Publisher fissi collegati al gz_bridge
        self.pub_left_detach = self.create_publisher(Empty, '/left_hand/detach', 10)
        self.pub_right_detach = self.create_publisher(Empty, '/right_hand/detach', 10)

    def left_contact_cb(self, msg):
        for contact in msg.contacts:
            col1 = contact.collision1.name
            col2 = contact.collision2.name
            target = col1 if "tcp" not in col1 else col2
            self.detected_object_left = target.split("::")[0]

    def right_contact_cb(self, msg):
        for contact in msg.contacts:
            col1 = contact.collision1.name
            col2 = contact.collision2.name
            target = col1 if "tcp" not in col1 else col2
            self.detected_object_right = target.split("::")[0]

    def attach_left_cb(self, msg):
        target_book = msg.data if msg.data else self.detected_object_left
        
        if not target_book or self.attached_joint_name:
            return

        joint_name = f"grasp_joint_{target_book}"
        
        # Generazione SDF del giunto dinamico
        sdf_joint = f"""
        <sdf version='1.6'>
            <joint name='{joint_name}' type='fixed'>
            <parent>mogi_arm::left_gripper_left_finger_link</parent>
            <child>{target_book}::base_link</child>
            </joint>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.entity_factory.sdf = sdf_joint
        req.entity_factory.name = joint_name

        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(f"Giunto creato con successo per {target_book}"))
        self.attached_joint_name = joint_name

    def attach_right_cb(self, msg):
        target_book = msg.data if msg.data else self.detected_object_right

        if not target_book or self.attached_joint_name:
            return

        joint_name = f"grasp_joint_{target_book}"
        
        # Generazione SDF del giunto dinamico
        sdf_joint = f"""
        <sdf version='1.6'>
          <joint name='{joint_name}' type='fixed'>
            <parent>mogi_arm::right_gripper_left_finger_link</parent>
            <child>{target_book}::base_link</child>
          </joint>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.entity_factory.sdf = sdf_joint
        req.entity_factory.name = joint_name

        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(f"Giunto creato con successo per {target_book}"))
        self.attached_joint_name = joint_name

    def detach_left_cb(self, msg):
        if not self.attached_joint_name:
            return

        req = DeleteEntity.Request()
        req.name = self.attached_joint_name
        req.type = 1  # Type 1 identifica la tipologia ENTITY/JOINT in ros_gz_interfaces

        future = self.remove_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info("Giunto rimosso con successo"))
        self.attached_joint_name = None

    def detach_right_cb(self, msg):
        if not self.attached_joint_name:
            return

        req = DeleteEntity.Request()
        req.name = self.attached_joint_name
        req.type = 1  # Type 1 identifica la tipologia ENTITY/JOINT in ros_gz_interfaces

        future = self.remove_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info("Giunto rimosso con successo"))
        self.attached_joint_name = None


def main(args=None):
    rclpy.init(args=args)
    node = GraspManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
