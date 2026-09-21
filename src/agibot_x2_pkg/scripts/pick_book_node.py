#!/usr/bin/env python3
"""
ROS 2 node that makes the simulated X2 pick a book from the bookshelf with fixed joint trajectories.
Sequence: look at shelf, open gripper, pre-grasp, reach, grasp, retract and deposit on the table.
Requires ros2_control with the joint_trajectory_controllers active, including
left_gripper_controller (see config/x2_controllers.yaml).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class PickBookNode(Node):
    """
    Node running an open-loop pick-and-place sequence on the left arm
    """

    def __init__(self):
        """
        Create the action clients of arm, gripper, head and waist controllers
        """
        super().__init__('pick_book_node')

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

        # left arm joints
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

        self.get_logger().info('PickBookNode started. Waiting for the controllers...')

    def send_trajectory(self, client, joint_names, positions_list, times_list):
        """
        Send a trajectory to a controller and wait for the result
        """
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
        """
        Turn the head towards the bookshelf (in front of the robot)
        """
        self.get_logger().info('[1/5] Looking at the bookshelf...')
        self.send_trajectory(
            self._head_client,
            joint_names=['head_yaw_joint', 'head_pitch_joint'],
            positions_list=[
                [0.0, 0.0],          # start
                [0.0, -0.3],         # tilt slightly down towards the shelf
            ],
            times_list=[1.0, 2.5]
        )

    def pre_grasp_pose(self):
        """
        Move the left arm to the pre-grasp pose
        Target is shelf 1 (height ~0.62 m, distance ~1.5 m); angles in rad are
        rough and meant to be refined with MoveIt 2.
        """
        self.get_logger().info('[2/5] Left arm pre-grasp...')
        # [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw]
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.0,  0.1, 0.0,  0.0,  0.0],   # home
                [0.5,  0.3, 0.0, -0.8,  0.0],   # arm forward/up
                [0.6,  0.4, 0.0, -1.0,  0.0],   # vertical approach to the book
            ],
            times_list=[1.0, 3.0, 4.5]
        )

    def reach_book(self):
        """
        Extend the arm to reach the book
        """
        self.get_logger().info('[3/5] Reaching the book...')
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.6,  0.4, 0.0, -1.0,  0.0],   # previous pose
                [0.65, 0.45, 0.0, -1.1, 0.2],   # extend towards the book
            ],
            times_list=[0.5, 2.0]
        )

    def open_gripper(self):
        """
        Open the gripper jaws before the grasp
        """
        self.send_trajectory(
            self._gripper_client,
            joint_names=self.left_gripper_joints,
            positions_list=[
                [0.037, 0.037],  # fully open (~GRIPPER_MAX_OPENING)
            ],
            times_list=[0.8]
        )

    def grasp(self):
        """
        Close the gripper jaws on the book spine
        """
        self.get_logger().info('[4/5] Grasping the book...')
        self.send_trajectory(
            self._gripper_client,
            joint_names=self.left_gripper_joints,
            positions_list=[
                [0.037, 0.037],  # jaws open
                [0.013, 0.013],  # jaws closed on the book spine (~30 mm)
            ],
            times_list=[0.3, 1.5]
        )
        self.get_logger().info('Book grasped!')

    def retract_and_deposit(self):
        """
        Retract the arm, turn the waist to the table and release the book there
        """
        self.get_logger().info('[5/5] Retracting and placing the book on the table...')

        # 1. lift the book off the shelf (jaws stay closed)
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.65, 0.45, 0.0, -1.1, 0.2],
                [0.4,  0.3,  0.0, -0.6, 0.2],   # lift
            ],
            times_list=[0.5, 2.0]
        )

        # 2. turn the waist to the table on the robot's right: -1.935 rad (~-111 deg)
        # is atan2 of the table centre in the world frame (create_full_scene.py),
        # within the waist_yaw_joint range (-3.43, 2.382)
        self.send_trajectory(
            self._waist_client,
            joint_names=['waist_yaw_joint', 'waist_pitch_joint'],
            positions_list=[
                [0.0,   0.0],
                [-1.935, 0.0],  # turn towards the table (robot's right)
            ],
            times_list=[0.5, 2.5]
        )

        # 3. lower the book onto the table
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.4,  0.3,  0.0, -0.6, 0.2],
                [0.3,  0.3,  0.0, -0.4, 0.2],   # lower towards the table
            ],
            times_list=[0.5, 2.0]
        )

        # 4. open the jaws (release the book)
        self.send_trajectory(
            self._gripper_client,
            joint_names=self.left_gripper_joints,
            positions_list=[
                [0.013, 0.013],  # jaws closed
                [0.037, 0.037],  # jaws open -> release
            ],
            times_list=[0.4, 1.0]
        )

        # 5. back to home
        self.send_trajectory(
            self._left_arm_client,
            joint_names=self.left_arm_joints,
            positions_list=[
                [0.3,  0.3, 0.0, -0.4, 0.2],
                [0.0,  0.1, 0.0,  0.0, 0.0],   # home
            ],
            times_list=[0.5, 2.5]
        )

        # 6. waist back to front
        self.send_trajectory(
            self._waist_client,
            joint_names=['waist_yaw_joint', 'waist_pitch_joint'],
            positions_list=[
                [-1.935, 0.0],
                [0.0,    0.0],
            ],
            times_list=[0.5, 2.5]
        )

        self.get_logger().info('Book placed! Sequence completed.')

    def run_pick_sequence(self):
        """
        Run the full pick-and-place sequence
        """
        self.get_logger().info('=== Starting PICK AND PLACE sequence ===')
        self.look_at_shelf()
        self.open_gripper()
        self.pre_grasp_pose()
        self.reach_book()
        self.grasp()
        self.retract_and_deposit()
        self.get_logger().info('=== Sequence completed ===')


def main(args=None):
    """
    Start the node, wait for the controllers and run the sequence once
    """
    rclpy.init(args=args)
    node = PickBookNode()

    # 3 s for the controllers to become active
    time.sleep(3.0)

    node.run_pick_sequence()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
