"""
ROS 2 node for keyboard teleop of arm, waist and gripper, to try a pick&place by hand.
Each key publishes one short trajectory point (STEP_TIME_SEC) straight to the controller topic, so motion is near real time.
Keys reach only the TERMINAL window: clicking into Gazebo steals the keyboard focus.
Keys: q/a w/s e/d r/f t/g arm, y/h u/j waist, o/c open/close gripper, 1..N target, b grasp+attach,
v attach by contact, n release, z switch arm, x home, p print state, CTRL-C quit.
`ros2 run agibot_x2_pkg_py pick_place_teleop`   (robot up with rviz_gaz_control.launch.py)
"""
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty, String

STEP_TIME_SEC = 0.15  # duration of each step, short for real-time feel
JOINT_STEP = 0.05     # rad per key press, arm/waist
KEY_POLL_TIMEOUT_SEC = 0.05  # select() wakes on a key anyway; short timeout keeps the loop responsive
GRIPPER_OPEN = 0.037  # fully open (joint upper limit, x2_hand_gazebo.urdf)
GRIPPER_CLOSED = 0.0  # fully closed

ATTACH_DELAY_SEC = 3.0   # WALL time between close and attach, sized for low RTF (0.15 s sim = 1.5 s real at 10%)

from agibot_x2_pkg.book_placer import collision_size, test_entities
from agibot_x2_pkg_py.arm_kinematics import grasp_opening

# key '1'..'N' -> (entity name, catalog key, kind), same list the launch spawns (book_placer.test_entities)
TEST_BOOKS = {}


def book_thickness(kind, object_key):
    """
    Thickness to close the fingers on: the COLLISION box one (thinner than the mesh for books)
    """
    return collision_size(kind, object_key)[1]


def clamp(value, limits):
    """
    Clamp value to the (lo, hi) limits
    """
    lo, hi = limits
    return max(lo, min(hi, value))


class PickPlaceTeleop(Node):
    """
    Keyboard teleop node for arm, waist, gripper and GraspManager attach/detach
    Attach/detach go through GraspManager (gripper_controller.py): with grasp_manager:=false b/v/n do nothing
    """

    def __init__(self):
        """
        Load the test entities for the scene and create the publishers
        """
        super().__init__('pick_place_teleop')
        self.declare_parameter('scene', 'grasp_test')  # same default as the launch
        scene = self.get_parameter('scene').value
        TEST_BOOKS.clear()
        TEST_BOOKS.update({str(i + 1): (n, k, kind)
                           for i, (n, k, kind, _x, _y) in enumerate(test_entities(scene))})

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

        self.active_side = 'right'  # reachable books are on the right side
        self.arm_pos = {'right': [0.0] * 5, 'left': [0.0] * 5}
        self.waist_pos = [0.0, 0.0]
        # Gazebo spawns the finger joints at 0 = closed
        self.gripper_pos = {'right': GRIPPER_CLOSED, 'left': GRIPPER_CLOSED}

        # automatic grasp on the test books (keys 1..N, b, n)
        self.target_key = '1'
        # global GraspManager topics: target name in the message ('' = what the finger touches)
        self.attach_pub = self.create_publisher(String, '/gripper/right/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/gripper/right/detach', 10)
        self._attach_timer = None

        # fingers live in {left,right}_arm_controller: every message needs all 7 joints (no partial goals in the JTC)
        self.arm_pub = {
            side: self.create_publisher(JointTrajectory, f'/{side}_arm_controller/joint_trajectory', 10)
            for side in ('left', 'right')
        }
        self.waist_pub = self.create_publisher(JointTrajectory, '/waist_controller/joint_trajectory', 10)

        self.get_logger().info(
            f'PickPlaceTeleop started. Active arm: {self.active_side}. '
            "Keys: q/a w/s e/d r/f t/g = arm, y/h u/j = waist, "
            f"o/c = open/close gripper, 1..{len(TEST_BOOKS)} = target ({', '.join(v[0] for v in TEST_BOOKS.values())}), "
            "b = automatic grasp (close to thickness + attach), v = attach what I touch, n = release, "
            "z = switch arm, x = home, p = print, CTRL-C = quit. "
            "NOTE: the gripper starts CLOSED, press 'o' before approaching a book."
        )

    def _publish(self, publisher, joint_names, positions):
        """
        Publish a single-point trajectory lasting STEP_TIME_SEC
        """
        msg = JointTrajectory()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=0, nanosec=int(STEP_TIME_SEC * 1e9))
        msg.points = [point]
        publisher.publish(msg)

    def _publish_arm(self, side):
        """
        Arm + fingers in one 7-joint message
        """
        g = self.gripper_pos[side]
        self._publish(
            self.arm_pub[side],
            self.arm_names[side] + self.gripper_names[side],
            list(self.arm_pos[side]) + [g, g],
        )

    def move_arm_joint(self, index, direction):
        """
        Step one joint of the active arm
        """
        pos = self.arm_pos[self.active_side]
        pos[index] = clamp(pos[index] + direction * JOINT_STEP, ARM_LIMITS[index])
        self._publish_arm(self.active_side)

    def move_waist_joint(self, index, direction):
        """
        Step one waist joint
        """
        self.waist_pos[index] = clamp(self.waist_pos[index] + direction * JOINT_STEP, WAIST_LIMITS[index])
        self._publish(self.waist_pub, ['waist_yaw_joint', 'waist_pitch_joint'], self.waist_pos)

    def set_gripper(self, opening):
        """
        Set the finger opening of the active arm
        """
        self.gripper_pos[self.active_side] = opening
        self._publish_arm(self.active_side)

    def select_target(self, key):
        """
        Select the target test entity
        """
        self.target_key = key
        name, obj_key, kind = TEST_BOOKS[key]
        self.get_logger().info(
            f'Target: {name} ({obj_key}, thickness {book_thickness(kind, obj_key)*100:.1f} cm)')

    def grasp(self):
        """
        Close the fingers to the target thickness, then attach after ATTACH_DELAY_SEC
        """
        name, obj_key, kind = TEST_BOOKS[self.target_key]
        if self.active_side != 'right':
            self.get_logger().warn(
                "The test books' DetachableJoint points to the RIGHT finger: "
                "switch to the right arm with 'z' before 'b'.")
            return
        opening = grasp_opening(book_thickness(kind, obj_key))
        self.set_gripper(opening)
        self.get_logger().info(
            f"Grasp {name}: fingers at {opening*1000:.1f} mm per side, attach in {ATTACH_DELAY_SEC:.0f}s...")
        if self._attach_timer is not None:
            self._attach_timer.cancel()

        def _do_attach():
            """
            One-shot timer callback that sends the attach
            """
            self._attach_timer.cancel()
            self._attach_timer = None
            self.attach_pub.publish(String(data=name))
            self.get_logger().info(
                f"ATTACH sent to /gripper/right/attach (data='{name}') - "
                "check the grasp_manager log; move the arm, the book follows")

        self._attach_timer = self.create_timer(ATTACH_DELAY_SEC, _do_attach)

    def attach_by_contact(self):
        """
        Attach what the right finger touches (empty name: GraspManager picks it from contacts)
        """
        if self.active_side != 'right':
            self.get_logger().warn("Only the RIGHT finger can attach: 'z' to switch arm.")
            return
        self.attach_pub.publish(String(data=''))
        self.get_logger().info(
            "Contact ATTACH sent to /gripper/right/attach (data='') - "
            "grasp_manager attaches the last touched entity (or warns if none)")

    def release(self):
        """
        Detach whatever is attached and open the gripper
        """
        self.detach_pub.publish(Empty())
        self.set_gripper(GRIPPER_OPEN)
        self.get_logger().info("RELEASE: detach sent to /gripper/right/detach, gripper open")

    def toggle_side(self):
        """
        Switch the active arm
        """
        self.active_side = 'left' if self.active_side == 'right' else 'right'
        self.get_logger().info(f'Active arm: {self.active_side}')

    def home(self):
        """
        Active arm and waist joints to 0 (gripper unchanged)
        """
        self.arm_pos[self.active_side] = [0.0] * 5
        self.waist_pos = [0.0, 0.0]
        self._publish_arm(self.active_side)
        self._publish(self.waist_pub, ['waist_yaw_joint', 'waist_pitch_joint'], self.waist_pos)

    def print_state(self):
        """
        Log current joint positions
        """
        self.get_logger().info(
            f"arm {self.active_side}={['%.3f' % v for v in self.arm_pos[self.active_side]]}  "
            f"waist={['%.3f' % v for v in self.waist_pos]}  "
            f"gripper {self.active_side}={self.gripper_pos[self.active_side]:.3f}"
        )

    def handle_key(self, key):
        """
        Dispatch a key press
        """
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
        elif key in TEST_BOOKS:
            self.select_target(key)
        elif key == 'b':
            self.grasp()
        elif key == 'v':
            self.attach_by_contact()
        elif key == 'n':
            self.release()
        elif key == 'z':
            self.toggle_side()
        elif key == 'x':
            self.home()
        elif key == 'p':
            self.print_state()


def get_key(settings, timeout=0.5):
    """
    Read one key in raw mode, '' on timeout
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    """
    Key loop: handle keys and spin the node until CTRL-C
    """
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
