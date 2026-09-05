import math
import os
import tempfile
import textwrap
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
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

    # default = ROBOT_SPAWN_X (-0.10, era 0.0 - 2026-08-30): 10 cm piu'
    # indietro dallo scaffale perche' il braccio destro possa fare una presa
    # laterale pulita sui libri di test (vedi book_placer.py).
    from agibot_x2_pkg.book_placer import ROBOT_SPAWN_X
    x_arg = DeclareLaunchArgument(
        'x', default_value=str(ROBOT_SPAWN_X),
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
    # scene (2026-08-30): "full" = scena unica full_scene.urdf (libri dipinti,
    # non afferrabili) + 2 libri fisici di test; "grasp_test" = solo
    # libreria + tavolo (bookshelf.urdf/table.urdf) + 4 libri e 2 oggetti
    # fisici afferrabili sul ripiano alto (book_placer.GRASP_TEST_ENTITIES).
    # default = grasp_test (2026-08-31, era full): e' l'ambiente della
    # pipeline definitiva (src/TODO) - 4 libri (IT + Hunger Games trilogia,
    # Ballata, Mietitura) e 2 oggetti tutti con collision, sul primo
    # scaffale raggiungibile. La scena completa resta disponibile con
    # scene:=full (full_scene.urdf NON e' stato rimosso). Come scegliere
    # l'ambiente: README.md "AMBIENTE (scene)".
    scene_arg = DeclareLaunchArgument(
        'scene', default_value='grasp_test',
        description='grasp_test (default) | full (vedi book_placer.test_entities)',
    )
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
                # --render-engine ogre2 (era "ogre", 2026-08-14): "ogre" non e'
                # un motore valido in Gazebo Harmonic/gz-sim 8 (ships solo
                # ogre2, vedi `gz sim --help` - "ogre" da solo non esiste piu'
                # come plugin). Prima delle camere Gazebo (vedi Gazebo.md
                # "Camere") questo flag era innocuo: nessun sensore forzava
                # mai l'inizializzazione reale del render engine. Con le
                # camere attive, il motore DEVE inizializzarsi per generare le
                # immagini - un valore non valido puo' far fallire/bloccare
                # l'avvio del server gz-sim (non solo della GUI), impedendo al
                # plugin gz_ros2_control di registrare il controller_manager
                # in tempo utile: sintomo osservato, "i controller non
                # c'erano e il robot crollava" al primo lancio con le camere.
                # Allineato al world file (Sensors system gia' su ogre2, vedi
                # worlds/empty.world).
                TextSubstitution(text=' -r -v -v1 --render-engine ogre2')]
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
        scene = LaunchConfiguration('scene').perform(context)
        if scene == 'grasp_test':
            # Libreria e tavolo come modelli separati, stesse pose della
            # scena unica: il tavolo sta a local (-1.05, 0.80) nel frame
            # dello scaffale (create_full_scene.py TABLE_POSITION), ruotato
            # come lo scaffale.
            sx, sy = float(shelf_x), float(shelf_y)
            c, s = math.cos(shelf_yaw_rad), math.sin(shelf_yaw_rad)
            lx, ly = -1.05, 0.80
            tx, ty = sx + c * lx - s * ly, sy + s * lx + c * ly
            return [
                Node(package="ros_gz_sim", executable="create", name="spawn_bookshelf",
                     arguments=["-world", "bookshelf_world", "-name", "bookshelf",
                                "-file", os.path.join(pkg, 'urdf', 'bookshelf.urdf'),
                                "-x", shelf_x, "-y", shelf_y, "-z", "0.0",
                                "-Y", str(round(shelf_yaw_rad, 6))],
                     output="screen"),
                Node(package="ros_gz_sim", executable="create", name="spawn_table",
                     arguments=["-world", "bookshelf_world", "-name", "table",
                                "-file", os.path.join(pkg, 'urdf', 'table.urdf'),
                                "-x", str(round(tx, 4)), "-y", str(round(ty, 4)), "-z", "0.0",
                                "-Y", str(round(shelf_yaw_rad, 6))],
                     output="screen"),
            ]
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

    # ─── Libri fisici di test (2026-08-29) ──────────────────────────────────
    # I libri della scena unica (full_scene.glb) sono SOLO visual: nessuna
    # collision individuale, non afferrabili - compromesso documentato in
    # Gazebo.md "Scena unica". Per provare un pick&place VERO, spawniamo
    # qui 2 libri come entita' dinamiche separate (stesse mesh .glb con
    # texture reali di copertina/dorso/retro gia' nel repo, dimensioni/masse
    # da BOOK_CATALOG - stessa fonte di manual_scene_placer._make_urdf) in
    # slot LIBERI del ripiano alto, lato destro (l'unico raggiungibile,
    # vedi Gazebo.md "Raggiungibilita del braccio").
    #
    # Ogni libro porta un plugin DetachableJoint (pattern del repo di
    # riferimento MOGI-ROS) verso il dito del gripper DESTRO: publicando
    # su /<nome>/attach il libro si "incolla" al dito (presa affidabile,
    # niente scivolamenti da attrito), su /<nome>/detach si stacca.
    # ATTENZIONE: il plugin nasce ATTACCATO (comportamento di gz-sim, non
    # configurabile) - il TimerAction piu' sotto pubblica il detach
    # iniziale a +30s dal lancio, prima che i controller comincino a
    # muovere qualcosa.
    # TEST_BOOKS e ROBOT_SPAWN_X vivono in agibot_x2_pkg.book_placer (fonte
    # unica condivisa con pick_place_teleop.py e pick_test_book.py, 2026-08-30):
    # posizioni scelte con la cinematica inversa, vedi commento la'.
    from agibot_x2_pkg.book_placer import TEST_BOOKS
    SHELF_TOP_SURFACE_Z = 0.993  # piano d'appoggio ripiano alto (book_placer)

    def _glb_center(glb_path: str):
        """Centro del bounding box dei vertici di un GLB (unione di tutte le
        primitive). Serve perche' i GLB dei libri sono esportati con la
        posizione che il libro aveva nella fila della scena Blender COTTA nei
        vertici (verificato 2026-08-30: emma_book centrata a x=+5.2 m,
        werther +0.8, hunger_games +0.4; solo it_book per caso a 0): senza
        compensazione il visual si disegna a metri di distanza dal corpo
        fisico (visto a schermo: il "libro" di Emma era un puntino sul
        pavimento a 5 m dallo scaffale). Parser minimale del container GLB
        (header 12 byte + chunk JSON), nessuna dipendenza esterna."""
        import struct, json as _json
        with open(glb_path, "rb") as f:
            f.read(12)
            clen, _ = struct.unpack("<I4s", f.read(8))
            data = _json.loads(f.read(clen))
        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        for mesh in data.get("meshes", []):
            for prim in mesh.get("primitives", []):
                acc = data["accessors"][prim["attributes"]["POSITION"]]
                for i in range(3):
                    lo[i] = min(lo[i], acc["min"][i])
                    hi[i] = max(hi[i], acc["max"][i])
        return [(a + b) / 2.0 for a, b in zip(lo, hi)]

    def _test_book_urdf(entity_name: str, object_key: str, kind: str = "book") -> str:
        from agibot_x2_pkg.book_placer import catalog_entry, collision_size, MESH_DIR
        info = catalog_entry(kind, object_key)
        mesh_dir = MESH_DIR[kind]
        sx, sy, sz = info["size"]
        # Collision: per i libri box piu' STRETTO della mesh lungo lo
        # spessore (TODO "box interna con collision, di larghezza minore
        # della mesh, lunghezza ok" - 2026-08-31, vedi
        # book_placer.collision_size); inerzia sempre dalla mesh piena.
        cx_, cy_, cz_ = collision_size(kind, object_key)
        m = info["mass"]
        # Compensazione offset mesh (vedi _glb_center): il visual applica
        # prima la rotazione Rx(+90) (Y-up -> Z-up) e poi trasla; per
        # centrare il libro sull'origine del link serve origin = -Rx90*c,
        # con Rx90: (x,y,z) -> (x,-z,y).
        cx, cy, cz = _glb_center(
            os.path.join(pkg, 'meshes', mesh_dir, f'{object_key}.glb'))
        vox, voy, voz = -cx, cz, -cy
        ixx = m / 12.0 * (sy**2 + sz**2)
        iyy = m / 12.0 * (sx**2 + sz**2)
        izz = m / 12.0 * (sx**2 + sy**2)
        # DINAMICO (niente <static>) a differenza di manual_scene_placer:
        # deve poter essere sollevato. Il DetachableJoint vive nel modello
        # del libro e punta al dito del robot (child_model = "mogi_arm",
        # il nome con cui rviz_gaz_control spawna il robot).
        return textwrap.dedent(f"""\
            <?xml version="1.0" encoding="utf-8"?>
            <robot name="{entity_name}">
              <link name="base_link">
                <inertial>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <mass value="{m}"/>
                  <inertia ixx="{ixx:.6f}" ixy="0" ixz="0"
                           iyy="{iyy:.6f}" iyz="0"
                           izz="{izz:.6f}"/>
                </inertial>
                <visual>
                  <!-- rpy Rx(+90): i GLB dei libri sono esportati Y-up
                       (stessa correzione "libro in piedi" di
                       scene_publisher.py) - senza, il visual appare
                       sdraiato mentre la collision resta dritta (visto
                       al secondo lancio reale). -->
                  <origin xyz="{vox:.4f} {voy:.4f} {voz:.4f}" rpy="1.5707963 0 0"/>
                  <geometry>
                    <mesh filename="package://agibot_x2_pkg/meshes/{mesh_dir}/{object_key}.glb"/>
                  </geometry>
                </visual>
                <collision>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <geometry>
                    <box size="{cx_} {cy_} {cz_}"/>
                  </geometry>
                </collision>
              </link>
              <gazebo reference="base_link">
                <mu1>1.2</mu1>
                <mu2>1.2</mu2>
                <kp>1e6</kp>
                <kd>100.0</kd>
              </gazebo>
              <gazebo>
                <plugin filename="gz-sim-detachable-joint-system"
                        name="gz::sim::systems::DetachableJoint">
                  <parent_link>base_link</parent_link>
                  <child_model>mogi_arm</child_model>
                  <child_link>right_gripper_left_finger_link</child_link>
                  <attach_topic>/{entity_name}/attach</attach_topic>
                  <detach_topic>/{entity_name}/detach</detach_topic>
                  <output_topic>/{entity_name}/state</output_topic>
                </plugin>
              </gazebo>
            </robot>
        """)

    def _spawn_test_books(context, *args, **kwargs):
        if LaunchConfiguration('spawn_test_books').perform(context).lower() \
                not in ('true', '1', 'yes'):
            return []
        from agibot_x2_pkg.book_placer import catalog_entry, test_entities
        scene = LaunchConfiguration('scene').perform(context)
        entities = test_entities(scene)
        actions = []
        for name, key, kind, wx, wy in entities:
            sz = catalog_entry(kind, key)["size"][2]
            path = os.path.join(tempfile.gettempdir(), f"{name}.urdf")
            with open(path, "w") as f:
                f.write(_test_book_urdf(name, key, kind))
            actions.append(Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{name}",
                arguments=[
                    "-world", "bookshelf_world",
                    "-name", name,
                    "-file", path,
                    "-x", str(wx), "-y", str(wy),
                    # appoggiato sul piano del ripiano alto + 2 mm di aria
                    "-z", str(round(SHELF_TOP_SURFACE_Z + sz / 2 + 0.002, 4)),
                    # libri: dorso verso il robot (yaw pi); oggetti: come sono
                    "-Y", "3.141593" if kind == "book" else "0.0",
                ],
                output="screen",
            ))
        # Detach iniziale dei DetachableJoint (nascono attaccati, vedi sopra):
        # +25 s dal lancio, --times 10 a 1 Hz per coprire la corsa all'avvio
        # (un colpo singolo si perdeva: libro che fluttuava attaccato al dito).
        actions.append(TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '--times', '10',
                         f'/{name}/detach', 'std_msgs/msg/Empty', '{}'],
                    output='screen',
                )
                for name, _k, _kind, _x, _y in entities
            ],
        ))
        return actions

    spawn_test_books_arg = DeclareLaunchArgument(
        'spawn_test_books', default_value='true',
        description='Spawna le entita\' fisiche afferrabili della scena '
                    '(book_placer.test_entities; false = nessuna: con '
                    'scene:=grasp_test restano solo libreria e tavolo)',
    )
    spawn_test_books_action = OpaqueFunction(function=_spawn_test_books)

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

    # Bridge immagini camera (testa RGBD + TCP gripper destro + tavolo, vedi
    # control_file.gazebo/full_scene.urdf): ros_gz_bridge/parameter_bridge
    # sopra gestisce solo camera_info (gz_bridge.yaml), le immagini vanno
    # tramite ros_gz_image che gestisce anche la ricompressione JPEG
    # automatica - stesso pattern usato per il bridge camera del repo di
    # riferimento MOGI-ROS/Week-9-10-Simple-arm.
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/rgbd_head_front/image",
            "/rgbd_head_front/depth_image",
            "/tcp_camera_left/image",
            "/tcp_camera_right/image",
            "/table_camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'rgbd_head_front.image.compressed.jpeg_quality': 75,
             'tcp_camera_left.image.compressed.jpeg_quality': 75,
             'tcp_camera_right.image.compressed.jpeg_quality': 75,
             'table_camera.image.compressed.jpeg_quality': 75},
        ],
    )

    # Relay camera_info su <topic>/image/camera_info: convenzione attesa da
    # strumenti come il plugin Camera di RViz2 (image+camera_info sotto lo
    # stesso namespace).
    relay_head_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_head_camera_info',
        output='screen',
        arguments=['rgbd_head_front/camera_info', 'rgbd_head_front/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_tcp_left_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tcp_left_camera_info',
        output='screen',
        arguments=['tcp_camera_left/camera_info', 'tcp_camera_left/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_tcp_right_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tcp_right_camera_info',
        output='screen',
        arguments=['tcp_camera_right/camera_info', 'tcp_camera_right/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_table_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_table_camera_info',
        output='screen',
        arguments=['table_camera/camera_info', 'table_camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # --service-call-timeout 180 su entrambi gli spawner (2026-08-29, prima 60): con le
    # camere attive e il rendering software, nei primi minuti dopo il lancio
    # il server gz-sim e' saturo (inizializzazione sensori/shader llvmpipe,
    # RTF quasi zero) e controller_manager - che cicla sul clock SIMULATO -
    # risponde ai servizi con ritardi ben oltre i 10 s di default: gli
    # spawner morivano con "Could not successfully call service ... after 3
    # attempts" pur con controller_manager sano (vedi Bugs.md 2026-08-29).
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster",
                   "--controller-manager-timeout", "120",
                   "--switch-timeout", "100",
                   "--service-call-timeout", "180"],
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
            '--param-file', robot_controllers,
            '--controller-manager-timeout', '120',
            '--switch-timeout', '100',
            '--service-call-timeout', '180',
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
    launchDescriptionObject.add_action(scene_arg)
    launchDescriptionObject.add_action(shelf_x_arg)
    launchDescriptionObject.add_action(shelf_y_arg)
    launchDescriptionObject.add_action(shelf_yaw_deg_arg)

    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(spawn_full_scene_action)
    launchDescriptionObject.add_action(spawn_test_books_arg)
    launchDescriptionObject.add_action(spawn_test_books_action)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(relay_head_camera_info_node)
    launchDescriptionObject.add_action(relay_tcp_left_camera_info_node)
    launchDescriptionObject.add_action(relay_tcp_right_camera_info_node)
    launchDescriptionObject.add_action(relay_table_camera_info_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    #launchDescriptionObject.add_action(joint_state_publisher_gui_node)


    # Spawner SERIALIZZATI, non piu' in parallelo (2026-08-29): i due spawner
    # condividono un lock globale di ros2_control - lanciati insieme, mentre
    # uno lavora (lentamente, server saturo dalle camere all'avvio) l'altro
    # fallisce 5 tentativi di acquisire il lock e muore ("Failed to acquire
    # lock in 20 seconds ... after multiple attempts", visto in un run reale).
    # Prima solo il broadcaster; i controller partono via OnProcessExit
    # quando il broadcaster ha finito (delay_controllers_spawner sopra, che
    # esisteva gia' predisposto e commentato).
    launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    launchDescriptionObject.add_action(delay_controllers_spawner)

    return launchDescriptionObject