#!/usr/bin/env python3
"""
Nodo ROS2 per simulare il robot X2 che prende libri da una libreria.

Flusso:
  1. Muovi la testa verso la libreria
  2. Pre-posizione braccio sinistro (ganasce del gripper aperte)
  3. Avvicinati al libro (reach)
  4. Chiudi le ganasce del gripper (presa)
  5. Ritira il braccio con il libro
  6. Deposita il libro sul tavolo (riapre le ganasce)

Richiede: ros2_control + joint_trajectory_controller attivi, incluso
left_gripper_controller (vedi config/x2_controllers.yaml e Blender.md).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class PickBookNode(Node):

    def __init__(self):
        super().__init__('pick_book_node')

        # Client per i controller del braccio sinistro
        self._left_arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory'
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_gripper_controller/follow_joint_trajectory'
        )
        self._head_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory'
        )
        self._waist_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/waist_controller/follow_joint_trajectory'
        )

        # Nomi giunti braccio sinistro
        self.left_arm_joints = [
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_yaw_joint',
        ]
        self.left_gripper_joints = [
            'left_gripper_left_finger_joint',
            'left_gripper_right_finger_joint',
        ]

        self.get_logger().info('PickBookNode avviato. In attesa dei controller...')

    def send_trajectory(self, client, joint_names, positions_list, times_list):
        """Invia una traiettoria a un controller e aspetta il risultato."""
        client.wait_for_server(timeout_sec=5.0)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        for positions, t_sec in zip(positions_list, times_list):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(
                sec=int(t_sec),
                nanosec=int((t_sec - int(t_sec)) * 1e9)
            )
            goal.trajectory.points.append(point)

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

    def look_at_shelf(self):
        """Gira la testa verso la libreria (davanti al robot)."""
        self.get_logger().info('[1/5] Guardo la libreria...')
        self.send_trajectory(
            self._head_client,
            joint_names=['head_yaw_joint', 'head_pitch_joint'],
            positions_list=[
                [0.0, 0.0],          # posizione iniziale
                [0.0, -0.3],         # abbassa leggermente verso il ripiano
            ],
            times_list=[1.0, 2.5]
        )

    def pre_grasp_pose(self):
        """
        Porta il braccio sinistro in posizione pre-presa.
        Ripiano 1 della libreria: altezza ~0.62m, distanza ~1.5m.
        Angoli in radianti (approssimati, da raffinare con MoveIt2).
        """
        self.get_logger().info('[2/5] Pre-presa braccio sinistro...')
        # [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw]
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.0,  0.1, 0.0,  0.0,  0.0],   # home
                [0.5,  0.3, 0.0, -0.8,  0.0],   # braccio in avanti/alto
                [0.6,  0.4, 0.0, -1.0,  0.0],   # avvicinamento verticale al libro
            ],
            times_list=[1.0, 3.0, 4.5]
        )

    def reach_book(self):
        """Estendi il braccio per raggiungere il libro."""
        self.get_logger().info('[3/5] Raggiungo il libro...')
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.6,  0.4, 0.0, -1.0,  0.0],   # posizione precedente
                [0.65, 0.45, 0.0, -1.1, 0.2],   # estensione verso libro
            ],
            times_list=[0.5, 2.0]
        )

    def open_gripper(self):
        """Apre le ganasce del gripper in preparazione alla presa."""
        self.send_trajectory(
            self._gripper_client,
            joint_names=self.left_gripper_joints,
            positions_list=[
                [0.037, 0.037],  # tutta aperta (~GRIPPER_MAX_OPENING)
            ],
            times_list=[0.8]
        )

    def grasp(self):
        """Chiude le ganasce del gripper sul dorso del libro."""
        self.get_logger().info('[4/5] Prendo il libro (grasp)...')
        self.send_trajectory(
            self._gripper_client,
            joint_names=self.left_gripper_joints,
            positions_list=[
                [0.037, 0.037],  # ganasce aperte
                [0.013, 0.013],  # ganasce chiuse sul dorso del libro (~30 mm)
            ],
            times_list=[0.3, 1.5]
        )
        self.get_logger().info('Libro preso!')

    def retract_and_deposit(self):
        """Ritira il braccio e deposita il libro sul tavolo dietro al robot."""
        self.get_logger().info('[5/5] Ritiro e deposito libro sul tavolo...')

        # Fase 1: solleva il libro dalla libreria (ganasce restano chiuse)
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.65, 0.45, 0.0, -1.1, 0.2],
                [0.4,  0.3,  0.0, -0.6, 0.2],   # solleva
            ],
            times_list=[0.5, 2.0]
        )

        # Fase 2: ruota il busto (waist) verso il tavolo di deposito
        self.send_trajectory(
            self._waist_client,
            joint_names=['waist_yaw_joint', 'waist_pitch_joint'],
            positions_list=[
                [0.0,  0.0],
                [3.14, 0.0],  # 180° → gira verso tavolo
            ],
            times_list=[0.5, 2.5]
        )

        # Fase 3: abbassa il libro sul tavolo
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.4,  0.3,  0.0, -0.6, 0.2],
                [0.3,  0.3,  0.0, -0.4, 0.2],   # abbassa verso tavolo
            ],
            times_list=[0.5, 2.0]
        )

        # Fase 4: apri le ganasce del gripper (rilascia libro)
        self.send_trajectory(
            self._gripper_client,
            joint_names=self.left_gripper_joints,
            positions_list=[
                [0.013, 0.013],  # ganasce chiuse
                [0.037, 0.037],  # ganasce aperte → rilascio
            ],
            times_list=[0.4, 1.0]
        )

        # Fase 5: torna alla posizione home
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.3,  0.3, 0.0, -0.4, 0.2],
                [0.0,  0.1, 0.0,  0.0, 0.0],   # home
            ],
            times_list=[0.5, 2.5]
        )

        # Riporta il busto in avanti
        self.send_trajectory(
            self._waist_client,
            joint_names=['waist_yaw_joint', 'waist_pitch_joint'],
            positions_list=[
                [3.14, 0.0],
                [0.0,  0.0],
            ],
            times_list=[0.5, 2.5]
        )

        self.get_logger().info('Libro depositato! Sequenza completata.')

    def run_pick_sequence(self):
        """Esegui la sequenza completa di pick-and-place."""
        self.get_logger().info('=== Inizio sequenza PICK AND PLACE ===')
        self.look_at_shelf()
        self.open_gripper()
        self.pre_grasp_pose()
        self.reach_book()
        self.grasp()
        self.retract_and_deposit()
        self.get_logger().info('=== Sequenza completata ===')


def main(args=None):
    rclpy.init(args=args)
    node = PickBookNode()

    # Aspetta 3 secondi per dare tempo ai controller di attivarsi
    time.sleep(3.0)

    node.run_pick_sequence()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
