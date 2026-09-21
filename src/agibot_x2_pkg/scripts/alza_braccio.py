"""
ROS 2 node that raises the X2 right arm once by publishing a single JointTrajectory.
The command is sent 2 s after startup to let the simulation settle.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class AgibotController(Node):
    """
    Node that publishes one trajectory to raise the right arm
    """

    def __init__(self):
        """
        Create the trajectory publisher and the one-shot start timer
        """
        super().__init__('agibot_arm_controller')

        # target topic of the arm/body controller (check with `ros2 topic list`)
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/goal_pose',
            # alternative: '/joint_trajectory_controller/joint_trajectory'
            10
        )

        # wait 2 s so the simulation is stable before commanding
        self.timer = self.create_timer(2.0, self.muovi_braccio_destro)

    def muovi_braccio_destro(self):
        """
        Publish a trajectory that raises the right arm (runs only once)
        """
        self.timer.cancel() # run only once

        msg = JointTrajectory()

        # right shoulder joints
        msg.joint_names = ['right_shoulder_pitch_link', 'right_shoulder_roll_link']

        point = JointTrajectoryPoint()

        # rad: 1.57 ~ 90 deg (arm raised forward or sideways)
        point.positions = [1.57, 0.0]

        # 3 s for a smooth motion
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points.append(point)

        self.get_logger().info('Raising the right arm...')
        self.publisher_.publish(msg)
        self.get_logger().info('Command sent successfully!')

def main(args=None):
    """
    Start the node and spin it
    """
    rclpy.init(args=args)
    nodo = AgibotController()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
