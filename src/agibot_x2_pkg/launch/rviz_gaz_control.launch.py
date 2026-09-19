import math
import os
import tempfile
import textwrap
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution, Command,
                                  TextSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg = get_package_share_directory('agibot_x2_pkg')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='false', # true to start rviz
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='config.rviz',
        description='RViz config file'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.world',  # world:=<file>.world to change
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='x2_hand_gazebo.urdf',
        description='Name of the URDF robot description to load'
    )

    # ROBOT_SPAWN_X = -0.10 (era 0.0): con la cinematica ricavata dall'URDF
    # (agibot_x2_pkg_py/arm_kinematics.py) il braccio destro riesce a fare una
    # presa laterale pulita (dita lungo Y, avvicinamento quasi orizzontale)
    # solo se il dorso del libro sta a ~0.30 m davanti al robot: a 0.25 m la
    # spalla e' troppo vicina e alta e il gripper arriva solo da sopra.
    # I libri stanno a x=0.28 (6 cm davanti alla fila dipinta a 0.40, con
    # ~2 cm di sovrapposizione visiva sul retro, accettata) e a y=-0.20/-0.30:
    # la zona davanti alla spalla destra, l'unica in cui il braccio destro
    # lavora bene - il roll della spalla e' limitato a +0.061 rad e non puo'
    # portare il braccio verso il centro del corpo.
    # Camminata (2026-09-13): il robot nasce walk_distance metri PIU'
    # INDIETRO della posa di lavoro (book_placer.ROBOT_SPAWN_X = -0.10) e la
    # raggiunge con `ros2 run agibot_x2_pkg_py walk_to_shelf` (base mobile
    # cinematica, vedi Gazebo.md "Camminata"). walk_distance:=0 = nasce
    # gia' davanti allo scaffale come prima. x:=... esplicito vince comunque.
    from agibot_x2_pkg.book_placer import ROBOT_SPAWN_X, WALK_DISTANCE
    walk_distance_arg = DeclareLaunchArgument(
        'walk_distance', default_value=str(WALK_DISTANCE),
        description='metri di camminata dallo spawn alla posa di lavoro (0 = niente camminata)'
    )
    x_arg = DeclareLaunchArgument(
        'x', default_value=PythonExpression([str(ROBOT_SPAWN_X), ' - ',
                                             LaunchConfiguration('walk_distance')]),
        description='x coordinate of spawned robot (default: ROBOT_SPAWN_X - walk_distance)'
    )

    # Attrito delle dita (2026-09-13): 1.0 realistico; finger_mu:=15 = valore
    # vecchio, per il test A/B "attrito o moto coordinato?" (x2_hand_gazebo.urdf)
    # Video della simulazione (2026-09-13): video:=true spawna la camera
    # "regista" urdf/video_camera.urdf (vista obliqua dall'alto, 1280x720 @
    # 20 Hz simulati, /video_camera/image). Da usare con gz_gui:=false
    # rviz:=false e il registratore `ros2 run agibot_x2_pkg_py record_video`
    # (README "VIDEO PER LA PRESENTAZIONE").
    video_arg = DeclareLaunchArgument(
        'video', default_value='false',
        description='spawna la camera regista per registrare il video (video_camera.urdf)'
    )
    spawn_video_camera = Node(
        package="ros_gz_sim", executable="create", name="spawn_video_camera",
        arguments=["-world", "bookshelf_world", "-name", "video_camera",
                   "-file", os.path.join(pkg, 'urdf', 'video_camera.urdf'),
                   "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
        condition=IfCondition(LaunchConfiguration('video')))

    finger_mu_arg = DeclareLaunchArgument(
        'finger_mu', default_value='1.0',
        description='coefficiente di attrito mu1/mu2 delle dita del gripper'
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

    robot_controllers = PathJoinSubstitution([pkg, 'config', 'x2_controllers.yaml',])
    
    # SERVER e GUI di Gazebo separati (2026-09-06): con `gz sim <world>` il
    # server non carica il mondo da solo ma ASPETTA che la GUI glielo mandi
    # su /gazebo/starting_world; sotto carico (RViz+GUI in avvio sui 4 core)
    # quel messaggio si perde e il server resta in attesa per sempre
    # ("Waiting for service /world/bookshelf_world/create" all'infinito,
    # GUI che ripete "requesting list of world names"). Con `-s` il server
    # carica il mondo subito; la GUI (`gz sim -g`) si aggancia dopo, e si
    # puo' anche non aprire (gz_gui:=false) per risparmiare un core.
    # Server avviato DIRETTAMENTE (2026-09-13, era l'include di
    # ros_gz_sim/gz_sim.launch.py): quel launch esegue `sh -c "ruby gz sim
    # ..."` e al Ctrl+C il SIGTERM uccide la shell ma NON il server, che
    # resta ORFANO. Al launch successivo due server sullo stesso mondo si
    # contendevano /clock e i servizi del controller_manager: spawner in
    # timeout, broadcaster "unconfigured", `gz topic` che non risponde
    # (vedi Bugs.md). Qui `gz` riceve i segnali in prima persona e, se non
    # esce in 5 s, il SIGKILL di launch arriva a lui. Stesse variabili
    # d'ambiente che impostava gz_sim.launch.py (plugin di sistema cercati
    # in LD_LIBRARY_PATH: gz_ros2_control-system, ros_gz_sim).
    from launch.substitutions import EnvironmentVariable
    gz_plugin_path_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        [EnvironmentVariable('LD_LIBRARY_PATH', default_value=''), ':',
         EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')])

    def _check_no_orphan_server(context, *args, **kwargs):
        import subprocess as _sp
        out = _sp.run(['pgrep', '-af', 'gz sim -s'], capture_output=True, text=True).stdout.strip()
        if out:
            raise RuntimeError(
                "C'e' gia' un server Gazebo in esecuzione (orfano di un launch precedente?):\n"
                f"{out}\nFermalo prima: pkill -f 'gz sim -s'   (poi rilancia)")
        return []

    world_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v1', '--render-engine', 'ogre2',
             PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])],
        name='gazebo', output='screen', sigterm_timeout='5', sigkill_timeout='5')

    gz_gui_arg = DeclareLaunchArgument(
        'gz_gui', default_value='true',
        description='Apri la GUI di Gazebo (false = solo server, meno CPU)')
    gz_gui_process = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['gz', 'sim', '-g', '-v1', '--render-engine', 'ogre2'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gz_gui')),
        )],
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
    # dichiarato in empty.world.
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
        if scene in ('grasp_test', 'ocr_test'):   # ocr_test (2026-09-17): stessa scena, 15 libri
            # Libreria e tavolo come modelli separati, stesse pose della
            # scena unica: il tavolo sta a local (-1.05, 0.80) nel frame
            # dello scaffale (create_full_scene.py TABLE_POSITION), ruotato
            # come lo scaffale.
            sx, sy = float(shelf_x), float(shelf_y)
            c, s = math.cos(shelf_yaw_rad), math.sin(shelf_yaw_rad)
            # lx -1.05 -> -0.93 (2026-09-06): con -1.05 il bordo vicino del
            # tavolo stava a y=-0.45, esattamente dove arriva il TCP a
            # braccio teso (FK: y=-0.447, z=0.94) - il libro veniva
            # lasciato cadere sul BORDO. Un primo tentativo a -0.87 (bordo a
            # y=-0.27) metteva lo spigolo del tavolo CONTRO la mano destra a
            # riposo (FK a casa: TCP y=-0.272, z=0.69 -> "Contatto right:
            # table" continuo e braccio impacciato). Ora bordo a y=-0.33:
            # 3 cm dalla mano a riposo, rilascio IK di pick_test_book a
            # y=-0.40, 7 cm dentro il piano.
            lx, ly = -0.93, 0.80
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
        (header 12 byte + chunk JSON), nessuna dipendenza esterna.

        Le DECORAZIONI hanno l'offset in un altro posto (scoperto 2026-09-06):
        vertici centrati, ma il NODO della scena ha una `translation`
        (coffee_mug [1,0,0], desk_globe [0.5,0,0], potted_plant [0.75,0,0],
        coaster [1.25,0,0], paperweight [0.25,0,0] - la fila della scena
        Blender, stavolta salvata come trasformazione del nodo). Gazebo la
        applica al visual: la tazza si vedeva 1 m dietro la libreria mentre
        il corpo fisico stava al suo posto (creduta "catapultata" per due
        giorni), il mappamondo era dentro il pannello posteriore. Ora la
        traslazione dei nodi (accumulata lungo i genitori) entra nel centro."""
        import struct, json as _json
        with open(glb_path, "rb") as f:
            f.read(12)
            clen, _ = struct.unpack("<I4s", f.read(8))
            data = _json.loads(f.read(clen))
        nodes = data.get("nodes", [])
        parent = {}
        for pi, n in enumerate(nodes):
            for ci in n.get("children", []):
                parent[ci] = pi

        def node_offset(ni):
            off = [0.0, 0.0, 0.0]
            while ni is not None:
                n = nodes[ni]
                if any(k in n for k in ("rotation", "scale", "matrix")):
                    print(f"[_glb_center] {glb_path}: nodo '{n.get('name')}' "
                          "con rotation/scale/matrix - non gestito, il "
                          "visual potrebbe risultare spostato")
                t = n.get("translation", [0.0, 0.0, 0.0])
                off = [a + b for a, b in zip(off, t)]
                ni = parent.get(ni)
            return off

        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        mesh_nodes = [(ni, n["mesh"]) for ni, n in enumerate(nodes) if "mesh" in n]
        if not mesh_nodes:   # GLB senza scena: solo le mesh, nessun offset
            mesh_nodes = [(None, mi) for mi in range(len(data.get("meshes", [])))]
        for ni, mi in mesh_nodes:
            off = node_offset(ni) if ni is not None else [0.0, 0.0, 0.0]
            for prim in data["meshes"][mi].get("primitives", []):
                acc = data["accessors"][prim["attributes"]["POSITION"]]
                for i in range(3):
                    lo[i] = min(lo[i], acc["min"][i] + off[i])
                    hi[i] = max(hi[i], acc["max"][i] + off[i])
        return [(a + b) / 2.0 for a, b in zip(lo, hi)]

    def _test_book_urdf(entity_name: str, object_key: str, kind: str = "book") -> str:
        from agibot_x2_pkg.book_placer import (catalog_entry, collision_size,
                                               entity_topics, MESH_DIR)
        info = catalog_entry(kind, object_key)
        mesh_dir = MESH_DIR[kind]
        # Stessi topic che bridge_config mette nel bridge generato
        topics = entity_topics(entity_name)
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
                <!-- mu 1.2 -> 0.5 (2026-09-13): carta/cartone o plastica su
                     legno; 1.2 era sovrastimato. Basta a tenere i libri in
                     piedi e fermi sul ripiano e sul tavolo. -->
                <mu1>0.5</mu1>
                <mu2>0.5</mu2>
                <kp>1e6</kp>
                <kd>100.0</kd>
              </gazebo>
              <gazebo>
                <plugin filename="gz-sim-detachable-joint-system"
                        name="gz::sim::systems::DetachableJoint">
                  <parent_link>base_link</parent_link>
                  <child_model>mogi_arm</child_model>
                  <child_link>right_gripper_left_finger_link</child_link>
                  <attach_topic>{topics["attach"]}</attach_topic>
                  <detach_topic>{topics["detach"]}</detach_topic>
                  <output_topic>{topics["state"]}</output_topic>
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
                    # appoggiato sul ripiano + 0.5 mm di aria (era 2 mm:
                    # meno caduta di assestamento = meno strappo sui
                    # DetachableJoint ancora attaccati alla nascita)
                    "-z", str(round(SHELF_TOP_SURFACE_Z + sz / 2 + 0.0005, 4)),
                    # libri: dorso verso il robot (yaw pi); oggetti: come sono
                    "-Y", "3.141593" if kind == "book" else "0.0",
                ],
                output="screen",
            ))
        # Detach PRIMA dello spawn (2026-09-06, terza iterazione): il plugin
        # DetachableJoint non ha un'opzione "nasci staccato" (verificato con
        # strings sul .so: solo attach/detach/output_topic), quindi le
        # entita' nascono incollate al dito. Con la raffica DOPO lo spawn
        # (prima +25s x10, poi +5s x35) bastava l'assestamento di pochi mm
        # sul ripiano a far litigare i 6 vincoli rigidi verso lo stesso dito
        # e CATAPULTARE l'oggetto piu' leggero (tazza ritrovata in cima al
        # mobile). Ora i publisher partono subito (5 Hz per 30 s, i colpi
        # senza bridge/plugin si perdono senza danni) e le entita' vengono
        # spawnate 2 s dopo: il primo detach arriva entro ~0.2 s
        # dall'attach, prima che l'assestamento carichi il vincolo.
        detach_pubs = [
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--times', '150', '-r', '5',
                     f'/{name}/detach', 'std_msgs/msg/Empty', '{}'],
                output='log',
            )
            for name, _k, _kind, _x, _y in entities
        ]
        return detach_pubs + [TimerAction(period=2.0, actions=actions)]

    spawn_test_books_arg = DeclareLaunchArgument(
        'spawn_test_books', default_value='true',
        description='Spawna le entita\' fisiche afferrabili della scena '
                    '(book_placer.test_entities; false = nessuna: con '
                    'scene:=grasp_test restano solo libreria e tavolo)',
    )

    # GraspManagerNode (agibot_x2_pkg_py/gripper_controller.py, 2026-09-06):
    # interfaccia UNICA di attach/detach - /gripper/right/attach (String:
    # nome entita' o '' = cio' che il dito sta toccando), /gripper/right/
    # detach (Empty), /gripper/right/attached (String latched) - e log dei
    # contatti dito-oggetto. Inoltra ai topic per entita' /<nome>/attach|
    # detach generati dal bridge (vedi _start_bridges).
    grasp_manager_arg = DeclareLaunchArgument(
        'grasp_manager', default_value='true',
        description="Avvia GraspManagerNode (attach/detach globale "
                    "/gripper/right/attach|detach, log dei contatti)")
    grasp_manager_node = Node(
        package='agibot_x2_pkg_py',
        executable='gripper_controller',
        name='grasp_manager',
        output='screen',
        condition=IfCondition(LaunchConfiguration('grasp_manager')),
        parameters=[{'scene': LaunchConfiguration('scene'),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    def _start_bridges(context, *args, **kwargs):
        """Bridge Gazebo<->ROS con la config GENERATA per la scena (base
        statica config/gz_bridge.yaml + attach/detach/state per ogni entita'
        di book_placer.test_entities) - niente piu' liste per entita' scritte
        a mano nel file statico (2026-09-06). Se il file statico non e' YAML
        valido il launch fallisce qui con traceback, invece di un bridge
        morto in silenzio."""
        from agibot_x2_pkg.bridge_config import write_bridge_config
        scene = LaunchConfiguration('scene').perform(context)
        spawn = LaunchConfiguration('spawn_test_books').perform(context).lower() \
            in ('true', '1', 'yes')
        cfg = write_bridge_config(scene, spawn)
        gz_bridge_node = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=['--ros-args', '-p', f'config_file:={cfg}'],
            output="screen",
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
        return [LogInfo(msg=f'bridge config generata: {cfg}'),
                gz_bridge_node, gz_image_bridge_node, grasp_manager_node]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['xacro', ' ', urdf_file_path,
                         ' finger_mu:=', LaunchConfiguration('finger_mu')]), value_type=str),
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

    # TODO eliminare le camere superflue che non vengono usate, perchè rallentano troppo gazebo
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
            "/head_camera/image",          # camera della testa, rgbd a scatto (2026-09-16)
            "/head_camera/depth_image",
            "/video_camera/image",      # camera regista (video:=true), altrimenti muta
            "/tcp_camera_left/image",
            "/tcp_camera_right/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'head_camera.image.compressed.jpeg_quality': 90,
             'tcp_camera_left.image.compressed.jpeg_quality': 75,
             'tcp_camera_right.image.compressed.jpeg_quality': 75,
             },
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
        arguments=['head_camera/camera_info', 'head_camera/image/camera_info'],
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

    relay_shelf_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_shelf_camera_info',
        output='screen',
        arguments=['shelf_camera/camera_info', 'shelf_camera/image/camera_info'],
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
            'left_gripper_controller',
            'right_gripper_controller',
            'head_controller',
            'waist_controller',
            'waist_roll_controller',
            # camminata cinematica (2026-09-13): base virtuale + gambe
            'base_controller',
            'legs_controller',
            '--param-file', robot_controllers,
            '--controller-manager-timeout', '120',
            '--switch-timeout', '100',
            '--service-call-timeout', '180',
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Spawn RITARDATO a controller attivi (2026-09-06): le entita' nascono
    # ATTACCATE al dito destro (DetachableJoint, non configurabile) e nei
    # primi secondi - braccio non ancora tenuto dai controller, che cede
    # sotto gravita' - i vincoli rigidi le strattonavano: la tazza (leggera
    # e tonda) finiva regolarmente fuori dallo scaffale anche col detach
    # anticipato a +8s. Ora si spawna SOLO quando lo spawner dei controller
    # e' uscito (OnProcessExit scatta anche se fallisce: in quel caso si
    # spawna comunque, semplicemente piu' tardi): a braccio fermo i vincoli
    # nascono in un mondo immobile e il detach li scioglie con calma.
    spawn_test_books_action = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[OpaqueFunction(function=_spawn_test_books)],
        )
    )

    # Riscaldamento della head_camera (2026-09-17): il PRIMO render di un
    # sensore rgbd esce senza colore (nero) per un difetto d'ordine in
    # gz-sensors: la connessione al point cloud colorato viene creata dentro
    # RgbdCameraSensor::Update(), ma Ogre2DepthCamera::PreRender (chiamato
    # prima da Sensors::RunOnce -> scene->PreRender) ha gia' impostato il
    # pass colore su "clear" perche' non vedeva connessioni. Dal secondo
    # render in poi e' tutto normale. Con una camera a scatto il primo
    # render sarebbe la prima foto utile: la scattiamo qui a vuoto quando i
    # controller sono pronti (image_bridge gia' sottoscritto, altrimenti il
    # sensore non renderizza). Vedi Bugs.md.
    head_camera_warmup = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[TimerAction(period=5.0, actions=[ExecuteProcess(
                cmd=['gz', 'topic', '-t', '/head_camera/trigger', '-m', 'gz.msgs.Boolean',
                     '-p', 'data: true'],
                name='head_camera_warmup', output='screen')])],
        )
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
    launchDescriptionObject.add_action(walk_distance_arg)
    launchDescriptionObject.add_action(finger_mu_arg)
    launchDescriptionObject.add_action(video_arg)
    launchDescriptionObject.add_action(spawn_video_camera)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(z_arg)
    launchDescriptionObject.add_action(yaw_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(scene_arg)
    launchDescriptionObject.add_action(shelf_x_arg)
    launchDescriptionObject.add_action(shelf_y_arg)
    launchDescriptionObject.add_action(shelf_yaw_deg_arg)

    launchDescriptionObject.add_action(OpaqueFunction(function=_check_no_orphan_server))
    launchDescriptionObject.add_action(gz_plugin_path_env)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(gz_gui_arg)
    launchDescriptionObject.add_action(gz_gui_process)
    # RViz dopo 15 s: in avvio prende un core intero (rendering software) e
    # affamava il server Gazebo proprio mentre caricava il mondo.
    launchDescriptionObject.add_action(TimerAction(period=15.0, actions=[rviz_node]))
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(spawn_full_scene_action)
    launchDescriptionObject.add_action(spawn_test_books_arg)
    launchDescriptionObject.add_action(spawn_test_books_action)
    launchDescriptionObject.add_action(head_camera_warmup)
    # Bridge Gazebo<->ROS avviati SOLO dopo lo spawn del robot (2026-09-06):
    # partendo insieme a Gazebo, sotto carico la sottoscrizione gz-transport
    # del parameter_bridge a /clock poteva non agganciarsi mai (processo
    # vivo, publisher ROS presente, zero messaggi) -> RViz a ROS Time 0.00,
    # robot_state_publisher senza orologio non pubblica /tf, RobotModel
    # "No transform" su tutti i link. `create` esce solo a mondo pronto,
    # quindi i bridge trovano i topic gz gia' avvertiti.
    launchDescriptionObject.add_action(grasp_manager_arg)
    launchDescriptionObject.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_urdf_node,
            on_exit=[OpaqueFunction(function=_start_bridges)],
        )
    ))
    launchDescriptionObject.add_action(relay_head_camera_info_node)
    launchDescriptionObject.add_action(relay_tcp_left_camera_info_node)
    launchDescriptionObject.add_action(relay_tcp_right_camera_info_node)
    launchDescriptionObject.add_action(relay_table_camera_info_node)
    launchDescriptionObject.add_action(relay_shelf_camera_info_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    #launchDescriptionObject.add_action(joint_state_publisher_gui_node)


    # ANTI-PAUSA (2026-09-13): trovata la simulazione in pausa (paused: true
    # su /stats) subito dopo il launch, con RViz e GUI aperte: niente /clock,
    # controller_manager fermo, spawner in timeout ("Failed getting a result
    # from calling /controller_manager/switch_controller in 180.0"). Causa
    # non individuata (click sul play della GUI o la GUI stessa che si
    # aggancia al server). 20 s dopo il launch mandiamo comunque un
    # "pause: false" al world: innocuo se gia' in esecuzione. A mano:
    #   gz service -s /world/bookshelf_world/control --reqtype gz.msgs.WorldControl \
    #     --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false'
    unpause_world = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/bookshelf_world/control',
                 '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '5000', '--req', 'pause: false'],
            name='unpause_world', output='screen')])
    launchDescriptionObject.add_action(unpause_world)

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