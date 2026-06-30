#!/usr/bin/env python3
"""
scene_publisher.py
==================
Pubblica in RViz2 la libreria e i libri tramite:
  - StaticTransformBroadcaster  → TF world → bookshelf_frame → book_N_frame
  - MarkerArray                 → mesh .dae (libreria) e .glb (libri) con texture

Separato da robot_state_publisher: il robot URDF rimane invariato.

Parametri ROS2:
  book_seed        (int,   42)   seed layout libri
  shelf_x          (float, 1.5)  posizione X libreria nel world frame
  shelf_y          (float, 0.0)
  shelf_yaw_deg    (float, 90.0) yaw libreria in gradi
  book_face_yaw_deg(float, 90.0) rotazione libri attorno a Z per orientare spine
                                 0°   → copertina verso robot
                                 90°  → spine verso robot  (default)
                                 180° → retrocopertina verso robot
"""
import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_rpy(roll: float, pitch: float, yaw: float):
    """Quaternion da roll-pitch-yaw (radianti) come (x, y, z, w)."""
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


def _quat_mul(q1, q2):
    """Prodotto di due quaternioni (x,y,z,w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


class ScenePublisher(Node):

    def __init__(self):
        super().__init__('scene_publisher')

        # dynamic_typing=True: accetta sia INTEGER che DOUBLE (es. :=0 o :=0.0)
        _dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('book_seed',         42,   _dyn)
        self.declare_parameter('shelf_x',           1.5,  _dyn)
        self.declare_parameter('shelf_y',           0.0,  _dyn)
        self.declare_parameter('shelf_yaw_deg',     90.0, _dyn)
        self.declare_parameter('book_face_yaw_deg', 90.0, _dyn)

        seed      = int(  self.get_parameter('book_seed').value)
        shelf_x   = float(self.get_parameter('shelf_x').value)
        shelf_y   = float(self.get_parameter('shelf_y').value)
        shelf_yaw = math.radians(float(self.get_parameter('shelf_yaw_deg').value))
        face_yaw  = math.radians(float(self.get_parameter('book_face_yaw_deg').value))

        from agibot_x2_pkg.book_placer import BookPlacer, BOOK_CATALOG
        placer = BookPlacer(seed=seed, shelf_x=shelf_x, shelf_y=shelf_y, shelf_yaw=shelf_yaw)
        self._placements = placer.generate()
        self._catalog    = BOOK_CATALOG
        self._shelf_x    = shelf_x
        self._shelf_y    = shelf_y
        self._shelf_yaw  = shelf_yaw
        self._face_yaw   = face_yaw

        # Quaternion per la correzione mesh:
        # 1. Rx(+90°): GLTF Y-up → RViz Z-up  (libro in piedi)
        # 2. Rz(face_yaw): ruota libro attorno a Z per orientare spine verso robot
        q_rx = _quat_from_rpy(math.pi / 2, 0.0, 0.0)
        q_rz = _quat_from_rpy(0.0, 0.0, face_yaw)
        self._book_quat = _quat_mul(q_rz, q_rx)   # Rz * Rx

        self._tf_static = StaticTransformBroadcaster(self)
        self._send_transforms()

        self._marker_pub = self.create_publisher(MarkerArray, '/scene_markers', 10)
        # Timer: continua a pubblicare così RViz vede i marker anche se aperto dopo
        self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            f'ScenePublisher pronto: seed={seed}, '
            f'libreria=({shelf_x:.1f},{shelf_y:.1f}) yaw={math.degrees(shelf_yaw):.0f}°, '
            f'libri={len(self._placements)}, face_yaw={math.degrees(face_yaw):.0f}°'
        )

    # ── TF ────────────────────────────────────────────────────────────────────

    def _make_tf(self, parent, child, x, y, z, yaw) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id  = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        qx, qy, qz, qw = _quat_from_rpy(0.0, 0.0, float(yaw))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def _send_transforms(self):
        tfs = []

        # world → bookshelf_link  (stesso nome del link nell'URDF di rsp_scene)
        tfs.append(self._make_tf(
            'world', 'bookshelf_link',
            self._shelf_x, self._shelf_y, 0.0, self._shelf_yaw
        ))

        # bookshelf_link → book_N_link  (coordinate locali)
        for p in self._placements:
            tfs.append(self._make_tf(
                'bookshelf_link', f'{p.name}_link',
                p.local_x, p.local_y, p.local_z,
                p.local_yaw
            ))

        self._tf_static.sendTransform(tfs)
        self.get_logger().info(
            f'Pubblicati {len(tfs)} TF statici ({len(tfs)-1} libri + libreria)'
        )

    # ── Markers ───────────────────────────────────────────────────────────────

    def _make_marker(self, mid: int, frame_id: str, mesh_uri: str,
                     quat=(0.0, 0.0, 0.0, 1.0)) -> Marker:
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id
        m.ns              = 'scene'
        m.id              = mid
        m.type            = Marker.MESH_RESOURCE
        m.action          = Marker.ADD
        m.lifetime        = Duration(sec=0, nanosec=0)   # 0 = permanente
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.x = quat[0]
        m.pose.orientation.y = quat[1]
        m.pose.orientation.z = quat[2]
        m.pose.orientation.w = quat[3]
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.0                   # 0 = usa materiale embedded
        m.mesh_resource              = mesh_uri
        m.mesh_use_embedded_materials = True
        return m

    def _publish_markers(self):
        ma  = MarkerArray()
        mid = 0

        # Libreria (DAE, Z-up nativo, nessuna rotazione)
        ma.markers.append(self._make_marker(
            mid, 'bookshelf_link',
            'package://agibot_x2_pkg/meshes/bookshelf.dae',
            quat=(0.0, 0.0, 0.0, 1.0)
        ))
        mid += 1

        # Libri (GLB Y-up → correzione Rz*Rx)
        for p in self._placements:
            ma.markers.append(self._make_marker(
                mid,
                f'{p.name}_link',
                f'package://agibot_x2_pkg/meshes/books/{p.book_key}.glb',
                quat=self._book_quat,
            ))
            mid += 1

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
