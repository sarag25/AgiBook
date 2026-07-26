import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class AgibotController(Node):

    def __init__(self):
        super().__init__('agibot_arm_controller')
        
        # Sostituisci con il topic corretto del controller delle braccia/corpo
        # Puoi verificarlo con: ros2 topic list
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/goal_pose',
            #'/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Aspetta 2 secondi prima di inviare il comando per stabilizzare la simulazione
        self.timer = self.create_timer(2.0, self.muovi_braccio_destro)

    def muovi_braccio_destro(self):
        self.timer.cancel() # Esegui solo una volta
        
        msg = JointTrajectory()
        
        # 1. INSERISCI I NOMI DEI GIUNTI DELLA SPALLA/BRACCIO DESTRO DEL TUO AGIBOT
        msg.joint_names = ['right_shoulder_pitch_link', 'right_shoulder_roll_link'] 
        
        # Creiamo il punto della traiettoria per alzare il braccio
        point = JointTrajectoryPoint()
        
        # 2. Definisci gli angoli in RADIANTI. 
        # Es: 1.57 radianti equivalgono a circa 90 gradi (braccio alzato in avanti o di lato)
        point.positions = [1.57, 0.0]  
        
        # Diamo al robot 3 secondi di tempo per completare il movimento in modo fluido
        point.time_from_start = Duration(sec=3, nanosec=0)
        
        msg.points.append(point)
        
        self.get_logger().info('Sto alzando il braccio destro...')
        self.publisher_.publish(msg)
        self.get_logger().info('Comando inviato con successo!')

def main(args=None):
    rclpy.init(args=args)
    nodo = AgibotController()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()