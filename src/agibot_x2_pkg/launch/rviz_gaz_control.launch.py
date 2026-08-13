import math
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg = get_package_share_directory('agibot_x2_pkg')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='config.rviz',
        description='RViz config file'
    )
    # empty.world (2026-08-10, era bookshelf.world): bookshelf.world contiene
    # una libreria/tavolo PLACEHOLDER hardcoded nell'SDF, scollegata dalla
    # scena reale generata da Blender - usarla insieme allo spawn della
    # scena reale qui sotto farebbe comparire libreria e tavolo duplicati,
    # sovrapposti. Passa world:=bookshelf.world per tornare al vecchio
    # comportamento (solo robot, world col placeholder).
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.world',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='x2_hand_gazebo.urdf',
        description='Name of the URDF description to load'
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='x coordinate of spawned robot'
    )
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='y coordinate of spawned robot'
    )
    # z = 0.662 (2026-08-10, era 1.04): quota che porta la pianta dei piedi
    # esattamente a terra (z=0) in posa zero - calcolata dalla catena
    # cinematica pelvis -> left_ankle_roll_link (-0.602 m) + collision del
    # piede (origine -0.04, mezza altezza 0.02) = -0.662 m. Il pelvis è
    # comunque fissato al mondo da world_to_pelvis_joint in
    # x2_hand_gazebo.urdf (base fissa, niente gambe che camminano - vedi
    # Gazebo.md "Base fissa"): questa quota è solo per non farlo comparire
    # a mezz'aria o infossato nel pavimento.
    z_arg = DeclareLaunchArgument(
        'z', default_value='0.662',
        description='z coordinate of spawned robot'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='yaw angle of spawned robot'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # ─── Scena completa (libreria + libri + decorazioni + tavolo) ───────────
    # Spawnata come UN SOLO modello (urdf/full_scene.urdf, visual =
    # meshes/full_scene.glb esportato da environment/export_full_scene_mesh.py)
    # invece che come 25 entità separate via manual_scene.launch.py/
    # ManualScenePlacer (2026-08-11, vedi Bugs.md - il vecchio percorso a
    # oggetti separati era troppo fragile: ogni pezzo doveva essere
    # esportato/posizionato correttamente per conto proprio, un bug nel
    # solo export dei libri bastava a rompere tutto). shelf_x/shelf_y/
    # shelf_yaw_deg posizionano l'intera scena in modo che il robot,
    # spawnato all'origine (x=0,y=0 sopra), la trovi esattamente a
    # ROBOT_TO_SHELF_DISTANCE = 0.25 m davanti a sé (shelf_x =
    # SHELF_FRONT_Y + ROBOT_TO_SHELF_DISTANCE = 0.40) - stessi valori di
    # environment/create_full_scene.py.
    shelf_x_arg = DeclareLaunchArgument(
        'shelf_x', default_value='0.40',
        description='Posizione X della libreria nel world frame (metri)',
    )
    shelf_y_arg = DeclareLaunchArgument(
        'shelf_y', default_value='0.0',
        description='Posizione Y della libreria nel world frame (metri)',
    )
    shelf_yaw_deg_arg = DeclareLaunchArgument(
        'shelf_yaw_deg', default_value='90.0',
        description='Rotazione della libreria attorno Z (gradi)',
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([pkg, "urdf", LaunchConfiguration('model')])

    gz_bridge_params_path = os.path.join(pkg, 'config', 'gz_bridge.yaml')
    
    robot_controllers = PathJoinSubstitution([pkg, 'config', 'x2_controllers.yaml',])
    
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')]),
                TextSubstitution(text=' -r -v -v1 --render-engine ogre')]
        }.items()
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'config',
            LaunchConfiguration('rviz_config')]
            ),],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    # -world bookshelf_world (2026-08-11): senza -world, "create" prova ad
    # auto-rilevare il mondo interrogando la lista dei mondi attivi
    # ("Requesting list of world names" in log) - quella query puo' restare
    # bloccata per sempre in alcuni ambienti Docker/WSL2 (discovery GZ
    # Transport rotta), impedendo qualunque spawn. Stesso fix applicato a
    # manual_scene.launch.py._spawn_node(). "bookshelf_world" e' il nome
    # dichiarato sia in bookshelf.world sia in empty.world.
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "bookshelf_world",
            "-name", "mogi_arm",
            "-topic", "robot_description",
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'), "-z", LaunchConfiguration('z'), "-Y",
            LaunchConfiguration('yaw') # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Spawn del modello unico full_scene.urdf (vedi blocco di commento sopra).
    # shelf_yaw_deg arriva in gradi (stessa convenzione di manual_scene.launch.py,
    # per chi lo passa a mano da riga di comando) e va convertito in radianti
    # per l'argomento -Y di "create" - fatto qui in un OpaqueFunction perché
    # una LaunchConfiguration è una stringa nota solo a runtime, non si può
    # passare a math.radians() al momento del parse.
    def _spawn_full_scene(context, *args, **kwargs):
        shelf_x = LaunchConfiguration('shelf_x').perform(context)
        shelf_y = LaunchConfiguration('shelf_y').perform(context)
        shelf_yaw_rad = math.radians(
            float(LaunchConfiguration('shelf_yaw_deg').perform(context))
        )
        full_scene_urdf = os.path.join(pkg, 'urdf', 'full_scene.urdf')
        return [Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_full_scene",
            arguments=[
                "-world", "bookshelf_world",
                "-name", "full_scene",
                "-file", full_scene_urdf,
                "-x", shelf_x,
                "-y", shelf_y,
                "-z", "0.0",
                "-Y", str(round(shelf_yaw_rad, 6)),
            ],
            output="screen",
        )]

    spawn_full_scene_action = OpaqueFunction(function=_spawn_full_scene)

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['xacro', ' ', urdf_file_path]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
   
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "120", "--switch-timeout", "100"],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_arm_controller',
            'right_arm_controller',
            'head_controller',
            'waist_controller',
            'left_gripper_controller',
            'right_gripper_controller',
            '--param-file', robot_controllers,
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # SOLUZIONE ERRORE MESHES: Diciamo a Gazebo dove cercare il pacchetto agibot_x2_pkg
    set_gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg)  # punta a install/agibot_x2_pkg/share
    )

    # Evento: Esegui i controller delle articolazioni solo DOPO che il broadcaster è terminato con successo
    delay_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    launchDescriptionObject = LaunchDescription()

    # L'impostazione della variabile d'ambiente deve avvenire PRIMA di lanciare Gazebo
    launchDescriptionObject.add_action(set_gz_model_path)

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(z_arg)
    launchDescriptionObject.add_action(yaw_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(shelf_x_arg)
    launchDescriptionObject.add_action(shelf_y_arg)
    launchDescriptionObject.add_action(shelf_yaw_deg_arg)

    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(spawn_full_scene_action)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    #launchDescriptionObject.add_action(joint_state_publisher_gui_node)


    launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    launchDescriptionObject.add_action(joint_trajectory_controller_spawner)

    # Avviamo solo il broadcaster
    #launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    # L'altro spawner verrà chiamato in automatico subito dopo!
    #launchDescriptionObject.add_action(delay_controllers_spawner)
    
    return launchDescriptionObject