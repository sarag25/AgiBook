"""
ROS 2 node that alternates the left arm between two poses every 3.5 s
by publishing to /left_arm_controller/joint_trajectory.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointAnglePublisher(Node):
    """
    Publishes alternating left-arm joint trajectories
    """

    def __init__(self):
        """
        Create the trajectory publisher and the 3.5 s timer
        """
        super().__init__(
            'initial_pose_publisher', 
            #automatically_declare_parameters_from_overrides=True   # allows reading use_sim_time
        )
        
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/left_arm_controller/joint_trajectory', 
            10
        )

        self.joint_names = [
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_yaw_joint',
            'left_gripper_left_finger_joint',
            'left_gripper_right_finger_joint'
        ]

        # which pose to send next
        self.toggle_pose = False

        self.timer = self.create_timer(3.5, self.send_joint_angles)
        self.get_logger().info('Node initialized. Sending alternating trajectories...')

    def send_joint_angles(self):
        """
        Send the next of the two poses and toggle the state
        """
        trajectory_command = JointTrajectory()

        #trajectory_command.header.stamp = self.get_clock().now().to_msg()
        # stamp 0 = execute immediately, so clock-sync issues cannot make the controller drop it
        trajectory_command.header.stamp.sec = 0
        trajectory_command.header.stamp.nanosec = 0
        
        trajectory_command.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        if self.toggle_pose:
            # pose A: shoulder forward (+1.2 rad), elbow bent (-1.2 rad)
            point.positions = [1.2, 0.3, 0.0, -1.2, 0.0, 0.0, 0.0]
            self.get_logger().info('Sent pose A (extended)')
        else:
            # pose B: shoulder and elbow near neutral
            point.positions = [-0.5, 0.0, 0.0, -0.2, 0.0, 0.0, 0.0]
            self.get_logger().info('Sent pose B (flexed)')

        self.toggle_pose = not self.toggle_pose

        point.velocities = [0.0] * len(self.joint_names)
        # reach the pose in 2.5 s
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 500000000 

        trajectory_command.points = [point]

        self.publisher.publish(trajectory_command)


def main(args=None):
    """
    Spin the node until Ctrl+C
    """
    rclpy.init(args=args)
    node = JointAnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
