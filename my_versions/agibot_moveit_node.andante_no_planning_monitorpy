#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.planning_scene_interface import PlanningSceneInterface

def main():
    rclpy.init()

    # 1. Inizializza MoveItPy per il robot AgiBot X2
    agibot = MoveItPy(node_name="agibot_moveit_py_node")

    # 2. Seleziona il componente di pianificazione (es. braccio destro)
    right_arm = agibot.get_planning_component("right_arm")

    # Inserisci il tavolo/scaffale nella Planning Scene di MoveIt
    psi = PlanningSceneInterface()

    # Esempio: aggiunta del box del tavolo (traslato come da full_scene.urdf)
    table_pose = Pose()
    table_pose.position.x = -1.05
    table_pose.position.y = 0.80
    table_pose.position.z = 0.7275
    table_pose.orientation.w = 1.0

    psi.add_box("table", table_pose, size=(1.20, 0.90, 0.045))

    
    # 3. Imposta i parametri di pianificazione
    plan_params = PlanRequestParameters(agibot, "right_arm")
    plan_params.max_velocity_scaling_factor = 0.5  # Modula la velocità
    plan_params.max_acceleration_scaling_factor = 0.5

    # 4. Definisci la posa target per l'End-Effector (TCP)
    target_pose = PoseStamped()
    target_pose.header.frame_id = "torso_link"
    target_pose.pose.position.x = 0.45
    target_pose.pose.position.y = -0.20
    target_pose.pose.position.z = 0.15
    target_pose.pose.orientation.w = 1.0

    # 5. Assegna il goal
    right_arm.set_goal_state(
        pose_stamped=target_pose,
        pose_link="right_wrist_yaw_link"
    )

    # 6. Pianifica la traiettoria
    agibot_node = agibot.get_node()
    agibot_node.get_logger().info("Pianificazione della traiettoria in corso...")

    # dopo aver registrato tutti gli ostacoli del mondo
    plan_result = right_arm.plan(single_plan_parameters=plan_params)

    # 7. Esegui la traiettoria se la pianificazione ha avuto successo
    if plan_result:
        agibot_node.get_logger().info("Pianificazione completata! Esecuzione su Gazebo...")
        agibot.execute(plan_result.trajectory, controllers=[])
    else:
        agibot_node.get_logger().error("Impossibile trovare una traiettoria valida (IK fallita o collisione).")

    rclpy.shutdown()

if __name__ == "__main__":
    main()