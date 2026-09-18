#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_configs_utils import MoveItConfigsBuilder

def main():
    rclpy.init()

    # Parametri MoveIt
    moveit_config = (
        MoveItConfigsBuilder("agibot_x2", package_name="agibot_x2_pkg")
        .robot_description(file_path="urdf/x2_hand_gazebo.urdf")
        .robot_description_semantic(file_path="config/agibot_x2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    config_dict = moveit_config.to_dict()
    config_dict["use_sim_time"] = True
    # Pre-declare ALL /clock QoS override parameters as flat keys.
    # When use_sim_time=True, ROS 2 tries to set these on the moveit_py node;
    # if they are not already declared, rclcpp throws InvalidParameterValueException.
    config_dict["qos_overrides./clock.subscription.durability"] = "volatile"
    config_dict["qos_overrides./clock.subscription.reliability"] = "best_effort"
    config_dict["qos_overrides./clock.subscription.history"] = "keep_last"
    config_dict["qos_overrides./clock.subscription.depth"] = 1
    config_dict["qos_overrides./clock.subscription.deadline"] = 0
    config_dict["qos_overrides./clock.subscription.lifespan"] = 0
    config_dict["qos_overrides./clock.subscription.liveliness"] = "automatic"
    config_dict["qos_overrides./clock.subscription.liveliness_lease_duration"] = 0

    # Pre-declare plan_request_params so PlanRequestParameters() finds them
    # without emitting 'not found in config' warnings
    config_dict["right_arm.plan_request_params.planner_id"] = ""
    config_dict["right_arm.plan_request_params.planning_pipeline"] = "ompl"
    config_dict["right_arm.plan_request_params.planning_time"] = 5.0
    config_dict["right_arm.plan_request_params.planning_attempts"] = 5
    config_dict["right_arm.plan_request_params.max_velocity_scaling_factor"] = 0.5
    config_dict["right_arm.plan_request_params.max_acceleration_scaling_factor"] = 0.5

    # 1. Inizializza MoveItPy per il robot AgiBot X2
    agibot = MoveItPy(
        node_name="agibot_moveit_py_node",
        config_dict=config_dict
    )

    logger = rclpy.logging.get_logger("agibot_moveit_py_node")
    logger.info("MoveItPy inizializzato con successo!")

    # 2. Seleziona il componente di pianificazione (es. braccio destro)
    right_arm = agibot.get_planning_component("right_arm")

    # Inserimento ostacoli nella Planning Scene
    planning_scene_monitor = agibot.get_planning_scene_monitor()

    # Esempio: aggiunta del box del tavolo
    table_object = CollisionObject()
    table_object.header.frame_id = "torso_link"
    table_object.id = "table"

    table_box = SolidPrimitive()
    table_box.type = SolidPrimitive.BOX
    table_box.dimensions = [1.20, 0.90, 0.045]

    table_pose = Pose()
    table_pose.position.x = -1.05
    table_pose.position.y = 0.80
    table_pose.position.z = 0.7275
    table_pose.orientation.w = 1.0

    table_object.primitives.append(table_box)
    table_object.primitive_poses.append(table_pose)
    table_object.operation = CollisionObject.ADD

    with planning_scene_monitor.read_write() as scene:
        scene.apply_collision_object(table_object)

    logger.info("Tavolo aggiunto alla Planning Scene.")

    # 3. Imposta i parametri di pianificazione
    plan_params = PlanRequestParameters(agibot, "right_arm")
    # Set directly to avoid 'not found in config' warnings
    plan_params.planner_id = ""          # empty = use pipeline default
    plan_params.planning_pipeline = "ompl"
    plan_params.planning_time = 10.0    # increased from 5 s
    plan_params.planning_attempts = 5
    plan_params.max_velocity_scaling_factor = 0.5
    plan_params.max_acceleration_scaling_factor = 0.5

    # 4. Definisci la posa target per l'End-Effector (TCP)
    # Arm workspace (torso_link frame, from URDF):
    #   shoulder at y≈-0.19, z≈+0.12 from torso_link
    #   total arm reach ≈ 0.32 m
    # Keep the target within reach: x≈0.20 forward, y≈-0.30 (right side), z≈0.05
    target_pose = PoseStamped()
    target_pose.header.frame_id = "torso_link"
    target_pose.pose.position.x = 0.20
    target_pose.pose.position.y = -0.30
    target_pose.pose.position.z = 0.05
    target_pose.pose.orientation.w = 1.0

    # 5. Assegna il goal
    # Note: keyword is pose_stamped_msg (not pose_stamped)
    right_arm.set_goal_state(
        pose_stamped_msg=target_pose,
        pose_link="right_wrist_yaw_link"
    )

    # 6. Pianifica la traiettoria
    logger.info("Pianificazione della traiettoria in corso...")
    plan_result = right_arm.plan(single_plan_parameters=plan_params)

    # 7. Esegui la traiettoria se la pianificazione ha avuto successo
    if plan_result:
        logger.info("Pianificazione completata! Esecuzione su Gazebo...")
        agibot.execute(plan_result.trajectory, controllers=[])
        logger.info("Traiettoria eseguita con successo.")
    else:
        logger.error("Impossibile trovare una traiettoria valida (IK fallita o collisione).")

    rclpy.shutdown()

if __name__ == "__main__":
    main()