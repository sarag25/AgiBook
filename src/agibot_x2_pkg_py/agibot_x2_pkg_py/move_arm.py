import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__(
            'initial_pose_publisher', 
            #automatically_declare_parameters_from_overrides=True # <--- Consente di leggere use_sim_time
        )

        #Dichiara e abilita il parametro use_sim_time se passato da launch/CLI
        self.declare_parameter('use_sim_time', True)
        
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
            'left_wrist_yaw_joint'
        ]

        # Stato per alternare il movimento
        self.toggle_pose = False

        # Invece di mandarlo una sola volta, eseguiamo la funzione ogni 3.5 secondi
        self.timer = self.create_timer(3.5, self.send_joint_angles)
        self.get_logger().info('Nodo inizializzato. Inizio invio traiettorie alternate...')

    def send_joint_angles(self):
        trajectory_command = JointTrajectory()

        #trajectory_command.header.stamp = self.get_clock().now().to_msg()
        # SOLUZIONE CHIAVE: Lasciando stamp a 0 (default), 
        # il controller esegue la traiettoria "IMMEDIATAMENTE", 
        # evitando che venga scartata per problemi di sincronizzazione oraria.
        trajectory_command.header.stamp.sec = 0
        trajectory_command.header.stamp.nanosec = 0
        
        trajectory_command.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        if self.toggle_pose:
            # Posizione A: Spalla in avanti (+1.2 rad) e gomito piegato (-1.2 rad)
            point.positions = [1.2, 0.3, 0.0, -1.2, 0.0]
            self.get_logger().info('Inviata Posizione A (Esteso)')
        else:
            # Posizione B: Spalla e gomito in posizione quasi neutra/opposta
            point.positions = [-0.5, 0.0, 0.0, -0.2, 0.0]
            self.get_logger().info('Inviata Posizione B (Flesso)')

        # Cambia stato per la prossima chiamata del timer
        self.toggle_pose = not self.toggle_pose

        point.velocities = [0.0] * len(self.joint_names)
        # Completa il movimento in 2.5 secondi
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 500000000 

        trajectory_command.points = [point]

        self.publisher.publish(trajectory_command)


def main(args=None):
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
