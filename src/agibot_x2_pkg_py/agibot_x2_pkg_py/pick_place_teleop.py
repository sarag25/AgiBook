"""
Teleop da tastiera per braccio, vita e gripper - per provare a mano un
pick&place (prendere un libro dallo scaffale, spostarlo sul tavolo) senza
aspettare una sequenza scriptata come pick_book_node.py.

A differenza di pick_book_node.py (ActionClient + wait_for_result, blocca
per l'intera durata di ogni traiettoria), qui ogni tasto pubblica subito
un solo punto di traiettoria a breve durata (STEP_TIME_SEC) direttamente
sul topic del controller - stesso stile "diretto" di move_arm.py, non
un'azione - così il movimento risponde quasi in tempo reale, un piccolo
passo alla volta, invece di una sequenza lunga e bloccante.

Uso:
    ros2 run agibot_x2_pkg_py pick_place_teleop

Con il robot già in piedi (rviz_gaz_control.launch.py) e i controller
braccio/vita/gripper attivi. Il braccio destro è quello utile per
raggiungere i libri (vedi Gazebo.md "Raggiungibilita del braccio" - la
config "classic" li mette tutti sul lato destro dello scaffale), quindi è
il braccio attivo di default; 'z' passa al sinistro se serve.

ATTENZIONE AL FOCUS DELLA TASTIERA: i tasti arrivano solo alla finestra
del TERMINALE dove gira questo script, non a quella di Gazebo. Se clicchi
dentro la finestra di Gazebo per orbitare/zoomare la vista, la tastiera
smette di parlare a questo nodo - i tasti premuti dopo non fanno niente
(sembra "bloccato", ma sta solo aspettando che il terminale riabbia il
focus). Se le due finestre sono affiancate, tienile così e lascia il
focus sul terminale: si vede comunque Gazebo aggiornarsi in tempo reale
senza doverci cliccare dentro.

Tasti:
  q/a  shoulder_pitch +/-        r/f  elbow +/-
  w/s  shoulder_roll +/-         t/g  wrist_yaw +/-
  e/d  shoulder_yaw +/-
  y/h  waist_yaw +/-             u/j  waist_pitch +/-
  o    apri gripper (braccio attivo)
  c    chiudi gripper (braccio attivo, grasp)
  z    cambia braccio attivo (destro <-> sinistro)
  x    home (tutti i giunti del braccio/vita attivi a 0, NON il gripper)
  p    stampa le posizioni correnti
  CTRL-C  esci
"""
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

STEP_TIME_SEC = 0.15  # durata di ogni singolo passo (2026-08-11: era 0.3, dimezzata su richiesta "più in tempo reale")
JOINT_STEP = 0.05     # rad per pressione, braccio/vita
KEY_POLL_TIMEOUT_SEC = 0.05  # quanto aspetta get_key() prima di ricontrollare (era 0.5): non aggiunge lag alla singola pressione (select() si sveglia subito appena arriva un tasto), ma rende il loop più reattivo quando si passa da un tasto all'altro rapidamente
GRIPPER_OPEN = 0.037  # tutta aperta (limite superiore del giunto, vedi x2_hand_gazebo.urdf)
GRIPPER_CLOSED = 0.0  # tutta chiusa

# [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw] - stessi limiti di x2_hand_gazebo.urdf
ARM_LIMITS = [
    (-3.08, 2.04),
    (-0.061, 2.993),
    (-2.556, 2.556),
    (-2.3556, 0.0),
    (-2.556, 2.556),
]
WAIST_LIMITS = [(-3.43, 2.382), (-0.314, 0.314)]  # [yaw, pitch]

ARM_JOINT_KEYS = {
    'q': (0, +1), 'a': (0, -1),   # shoulder_pitch
    'w': (1, +1), 's': (1, -1),   # shoulder_roll
    'e': (2, +1), 'd': (2, -1),   # shoulder_yaw
    'r': (3, +1), 'f': (3, -1),   # elbow
    't': (4, +1), 'g': (4, -1),   # wrist_yaw
}
WAIST_JOINT_KEYS = {
    'y': (0, +1), 'h': (0, -1),   # waist_yaw
    'u': (1, +1), 'j': (1, -1),   # waist_pitch
}


def clamp(value, limits):
    lo, hi = limits
    return max(lo, min(hi, value))


class PickPlaceTeleop(Node):

    def __init__(self):
        super().__init__('pick_place_teleop')

        self.arm_names = {
            'right': [
                'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_yaw_joint',
            ],
            'left': [
                'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
                'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_yaw_joint',
            ],
        }
        self.gripper_names = {
            'right': ['right_gripper_left_finger_joint', 'right_gripper_right_finger_joint'],
            'left': ['left_gripper_left_finger_joint', 'left_gripper_right_finger_joint'],
        }

        self.active_side = 'right'  # i libri raggiungibili stanno sul lato destro
        self.arm_pos = {'right': [0.0] * 5, 'left': [0.0] * 5}
        self.waist_pos = [0.0, 0.0]
        self.gripper_pos = {'right': GRIPPER_OPEN, 'left': GRIPPER_OPEN}

        self.arm_pub = {
            side: self.create_publisher(JointTrajectory, f'/{side}_arm_controller/joint_trajectory', 10)
            for side in ('left', 'right')
        }
        self.gripper_pub = {
            side: self.create_publisher(JointTrajectory, f'/{side}_gripper_controller/joint_trajectory', 10)
            for side in ('left', 'right')
        }
        self.waist_pub = self.create_publisher(JointTrajectory, '/waist_controller/joint_trajectory', 10)

        self.get_logger().info(
            f'PickPlaceTeleop avviato. Braccio attivo: {self.active_side}. '
            "Tasti: q/a w/s e/d r/f t/g = braccio, y/h u/j = vita, "
            "o/c = apri/chiudi gripper, z = cambia braccio, x = home, p = stampa, CTRL-C = esci."
        )

    def _publish(self, publisher, joint_names, positions):
        msg = JointTrajectory()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=0, nanosec=int(STEP_TIME_SEC * 1e9))
        msg.points = [point]
        publisher.publish(msg)

    def move_arm_joint(self, index, direction):
        pos = self.arm_pos[self.active_side]
        pos[index] = clamp(pos[index] + direction * JOINT_STEP, ARM_LIMITS[index])
        self._publish(self.arm_pub[self.active_side], self.arm_names[self.active_side], pos)

    def move_waist_joint(self, index, direction):
        self.waist_pos[index] = clamp(self.waist_pos[index] + direction * JOINT_STEP, WAIST_LIMITS[index])
        self._publish(self.waist_pub, ['waist_yaw_joint', 'waist_pitch_joint'], self.waist_pos)

    def set_gripper(self, opening):
        self.gripper_pos[self.active_side] = opening
        self._publish(
            self.gripper_pub[self.active_side],
            self.gripper_names[self.active_side],
            [opening, opening],
        )

    def toggle_side(self):
        self.active_side = 'left' if self.active_side == 'right' else 'right'
        self.get_logger().info(f'Braccio attivo: {self.active_side}')

    def home(self):
        self.arm_pos[self.active_side] = [0.0] * 5
        self.waist_pos = [0.0, 0.0]
        self._publish(self.arm_pub[self.active_side], self.arm_names[self.active_side], self.arm_pos[self.active_side])
        self._publish(self.waist_pub, ['waist_yaw_joint', 'waist_pitch_joint'], self.waist_pos)

    def print_state(self):
        self.get_logger().info(
            f"braccio {self.active_side}={['%.3f' % v for v in self.arm_pos[self.active_side]]}  "
            f"vita={['%.3f' % v for v in self.waist_pos]}  "
            f"gripper {self.active_side}={self.gripper_pos[self.active_side]:.3f}"
        )

    def handle_key(self, key):
        if key in ARM_JOINT_KEYS:
            index, direction = ARM_JOINT_KEYS[key]
            self.move_arm_joint(index, direction)
        elif key in WAIST_JOINT_KEYS:
            index, direction = WAIST_JOINT_KEYS[key]
            self.move_waist_joint(index, direction)
        elif key == 'o':
            self.set_gripper(GRIPPER_OPEN)
        elif key == 'c':
            self.set_gripper(GRIPPER_CLOSED)
        elif key == 'z':
            self.toggle_side()
        elif key == 'x':
            self.home()
        elif key == 'p':
            self.print_state()


def get_key(settings, timeout=0.5):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = PickPlaceTeleop()

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=KEY_POLL_TIMEOUT_SEC)
            if key == '\x03':  # CTRL-C
                break
            if key:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
