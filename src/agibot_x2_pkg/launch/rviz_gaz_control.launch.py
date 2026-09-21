"""
Launch file that starts Gazebo (server + optional GUI), the AgiBot X2 with ros2_control,
the bookshelf scene with graspable test entities, the Gazebo<->ROS bridges, GraspManagerNode and RViz.
  ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py
  ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py video:=true gz_gui:=false rviz:=false
"""
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
    """
    Declare all arguments and build the full simulation launch description
    """
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

    # ROBOT_SPAWN_X = -0.10: the right arm gets a clean side grasp only with the book spine ~0.30 m in
    # front of the robot (at 0.25 m the shoulder is too close and high). Books sit at y=-0.20/-0.30,
    # in front of the right shoulder: shoulder roll is limited to +0.061 rad and cannot reach the center.
    # The robot spawns walk_distance m behind the working pose and reaches it with
    # `ros2 run agibot_x2_pkg_py walk_to_shelf`; walk_distance:=0 spawns it in place, explicit x:= wins.
    from agibot_x2_pkg.book_placer import ROBOT_SPAWN_X, WALK_DISTANCE
    walk_distance_arg = DeclareLaunchArgument(
        'walk_distance', default_value=str(WALK_DISTANCE),
        description='walking distance in m from spawn to the working pose (0 = no walking)'
    )
    x_arg = DeclareLaunchArgument(
        'x', default_value=PythonExpression([str(ROBOT_SPAWN_X), ' - ',
                                             LaunchConfiguration('walk_distance')]),
        description='x coordinate of spawned robot (default: ROBOT_SPAWN_X - walk_distance)'
    )

    # finger_mu: 1.0 is realistic, 15 was the old value (A/B friction test).
    # video:=true spawns the director camera urdf/video_camera.urdf (oblique top view, 1280x720 @ 20 Hz
    # sim, /video_camera/image); use with gz_gui:=false rviz:=false and `ros2 run agibot_x2_pkg_py record_video`.
    video_arg = DeclareLaunchArgument(
        'video', default_value='false',
        description='spawn the director camera to record the video (video_camera.urdf)'
    )
    spawn_video_camera = Node(
        package="ros_gz_sim", executable="create", name="spawn_video_camera",
        arguments=["-world", "bookshelf_world", "-name", "video_camera",
                   "-file", os.path.join(pkg, 'urdf', 'video_camera.urdf'),
                   "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
        condition=IfCondition(LaunchConfiguration('video')))

    spawn_video_camera_table = Node(
        package="ros_gz_sim", executable="create", name="spawn_video_camera_table",
        arguments=["-world", "bookshelf_world", "-name", "video_camera_table",
                   "-file", os.path.join(pkg, 'urdf', 'video_camera_table.urdf'),
                   "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
        condition=IfCondition(LaunchConfiguration('video')))

    finger_mu_arg = DeclareLaunchArgument(
        'finger_mu', default_value='1.0',
        description='friction coefficient mu1/mu2 of the gripper fingers'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='y coordinate of spawned robot'
    )

    # z = 0.662: puts the soles exactly on the ground in zero pose (pelvis -> left_ankle_roll_link
    # -0.602 m + foot collision -0.06 m); the pelvis is fixed to the world anyway (world_to_pelvis_joint).
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

    # scene: "grasp_test" (default) = bookshelf.urdf + table.urdf + graspable books/objects with collision
    # (book_placer.GRASP_TEST_ENTITIES); "full" = single model full_scene.urdf (painted, non-graspable books,
    # more robust than 25 separate entities) + test books. shelf_x = SHELF_FRONT_Y + ROBOT_TO_SHELF_DISTANCE
    # = 0.40, same values as environment/create_full_scene.py.
    scene_arg = DeclareLaunchArgument(
        'scene', default_value='grasp_test',
        description='grasp_test (default) | full (see book_placer.test_entities)',
    )

    shelf_x_arg = DeclareLaunchArgument(
        'shelf_x', default_value='0.40',
        description='Bookshelf X position in the world frame (m)',
    )

    shelf_y_arg = DeclareLaunchArgument(
        'shelf_y', default_value='0.0',
        description='Bookshelf Y position in the world frame (m)',
    )

    shelf_yaw_deg_arg = DeclareLaunchArgument(
        'shelf_yaw_deg', default_value='90.0',
        description='Bookshelf rotation about Z (deg)',
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([pkg, "urdf", LaunchConfiguration('model')])

    robot_controllers = PathJoinSubstitution([pkg, 'config', 'x2_controllers.yaml',])
    
    # Separate Gazebo server and GUI: with `-s` the server loads the world itself instead of waiting for
    # the GUI message on /gazebo/starting_world, which gets lost under load; gz_gui:=false saves a core.
    # The server is run directly (not via gz_sim.launch.py's `sh -c`) so Ctrl+C reaches it and no orphan
    # server stays alive to fight over /clock; same plugin path env vars as gz_sim.launch.py.
    from launch.substitutions import EnvironmentVariable
    gz_plugin_path_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        [EnvironmentVariable('LD_LIBRARY_PATH', default_value=''), ':',
         EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')])

    def _check_no_orphan_server(context, *args, **kwargs):
        """
        Abort the launch if a Gazebo server (orphan of a previous launch) is already running
        """
        import subprocess as _sp
        out = _sp.run(['pgrep', '-af', 'gz sim -s'], capture_output=True, text=True).stdout.strip()
        if out:
            raise RuntimeError(
                "A Gazebo server is already running (orphan of a previous launch?):\n"
                f"{out}\nStop it first: pkill -f 'gz sim -s'   (then relaunch)")
        return []

    world_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v1', '--render-engine', 'ogre2',
             PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])],
        name='gazebo', output='screen', sigterm_timeout='5', sigkill_timeout='5')

    gz_gui_arg = DeclareLaunchArgument(
        'gz_gui', default_value='true',
        description='Open the Gazebo GUI (false = server only, less CPU)')
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
    # explicit -world: auto-detection can hang forever when GZ Transport discovery is broken (Docker/WSL2)
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

    def _spawn_full_scene(context, *args, **kwargs):
        """
        Spawn the scene (shelf + table, or full_scene.urdf); an OpaqueFunction because
        shelf_yaw_deg is only known at runtime and must be converted to radians for -Y
        """
        shelf_x = LaunchConfiguration('shelf_x').perform(context)
        shelf_y = LaunchConfiguration('shelf_y').perform(context)
        shelf_yaw_rad = math.radians(
            float(LaunchConfiguration('shelf_yaw_deg').perform(context))
        )
        scene = LaunchConfiguration('scene').perform(context)
        if scene in ('grasp_test', 'ocr_test'):   # ocr_test: same scene, 15 books
            # bookshelf and table as separate models, table at TABLE_LOCAL_XY in the shelf frame
            sx, sy = float(shelf_x), float(shelf_y)
            c, s = math.cos(shelf_yaw_rad), math.sin(shelf_yaw_rad)
            # lx = -0.93: near table edge at y=-0.33, 3 cm from the resting right hand, and the IK
            # release at y=-0.40 lands 7 cm inside the table top instead of on the edge
            from agibot_x2_pkg.scene_config import TABLE_LOCAL_XY
            lx, ly = TABLE_LOCAL_XY      # -0.93, 0.80 (scene_config is the single source)
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

    # Physical entities: dynamic models (real .glb meshes, size/mass from BOOK_CATALOG) on free slots
    # of the reachable shelf, since the full_scene books are visual only. Each carries DetachableJoint
    # plugins towards the gripper fingers (/<name>/attach, /<name>/detach) for a reliable grasp; the
    # plugin is born ATTACHED (gz-sim behavior), so a detach is published at spawn time.
    from agibot_x2_pkg.book_placer import TEST_BOOKS
    SHELF_TOP_SURFACE_Z = 0.993  # top shelf surface (book_placer)

    def _glb_center(glb_path: str):
        """
        Center of the vertex bounding box of a GLB (all primitives, plus node translations along parents)
        The book GLBs have their Blender row position baked into the vertices and decorations have it
        as a node translation: without compensation the visual is drawn metres away from the body.
        Minimal GLB container parser (12-byte header + JSON chunk), no external dependencies.
        """
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
            """
            Translation of node ni accumulated along its parents
            """
            off = [0.0, 0.0, 0.0]
            while ni is not None:
                n = nodes[ni]
                if any(k in n for k in ("rotation", "scale", "matrix")):
                    print(f"[_glb_center] {glb_path}: node '{n.get('name')}' "
                          "with rotation/scale/matrix - not handled, the "
                          "visual may be offset")
                t = n.get("translation", [0.0, 0.0, 0.0])
                off = [a + b for a, b in zip(off, t)]
                ni = parent.get(ni)
            return off

        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        mesh_nodes = [(ni, n["mesh"]) for ni, n in enumerate(nodes) if "mesh" in n]
        if not mesh_nodes:   # GLB without scene: meshes only, no offset
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
        """
        URDF of a dynamic, graspable test entity with DetachableJoint plugins towards both grippers
        """
        from agibot_x2_pkg.book_placer import (catalog_entry, collision_size,
                                               entity_topics, MESH_DIR)
        info = catalog_entry(kind, object_key)
        mesh_dir = MESH_DIR[kind]
        # same topics that bridge_config puts in the generated bridge
        topics = entity_topics(entity_name)
        sx, sy, sz = info["size"]
        # collision box thinner than the mesh for books (book_placer.collision_size); inertia from the full mesh
        cx_, cy_, cz_ = collision_size(kind, object_key)
        m = info["mass"]
        # mesh offset compensation (see _glb_center): the visual rotates Rx(+90) (Y-up -> Z-up) and then
        # translates, so origin = -Rx90*c, with Rx90: (x,y,z) -> (x,-z,y)
        cx, cy, cz = _glb_center(
            os.path.join(pkg, 'meshes', mesh_dir, f'{object_key}.glb'))
        vox, voy, voz = -cx, cz, -cy
        ixx = m / 12.0 * (sy**2 + sz**2)
        iyy = m / 12.0 * (sx**2 + sz**2)
        izz = m / 12.0 * (sx**2 + sy**2)
        # DYNAMIC (no <static>) so it can be lifted; child_model "mogi_arm" is the spawned robot's name
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
                  <!-- rpy Rx(+90): book GLBs are exported Y-up (same
                       correction as scene_publisher.py), otherwise the
                       visual lies flat while the collision stands upright -->
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
                <!-- mu 0.5: paper/cardboard or plastic on wood, enough to
                     keep books upright and still on shelf and table -->
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
                <!-- same for the LEFT hand (objects at y>0) -->
                <plugin filename="gz-sim-detachable-joint-system"
                        name="gz::sim::systems::DetachableJoint">
                  <parent_link>base_link</parent_link>
                  <child_model>mogi_arm</child_model>
                  <child_link>left_gripper_left_finger_link</child_link>
                  <attach_topic>{topics["attach_left"]}</attach_topic>
                  <detach_topic>{topics["detach_left"]}</detach_topic>
                  <output_topic>{topics["state_left"]}</output_topic>
                </plugin>
              </gazebo>
            </robot>
        """)

    def _spawn_test_books(context, *args, **kwargs):
        """
        Start the detach publishers, then spawn the scene's test entities 2 s later
        """
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
                    # on the shelf + 0.5 mm of air: less settling drop, less pull on the born-attached joints
                    "-z", str(round(SHELF_TOP_SURFACE_Z + sz / 2 + 0.0005, 4)),
                    # books: spine towards the robot (yaw pi); objects: as they are
                    "-Y", "3.141593" if kind == "book" else "0.0",
                ],
                output="screen",
            ))
        # Detach BEFORE spawning: DetachableJoint has no "born detached" option, and settling while
        # attached made the rigid constraints catapult light objects. Publishers start now (5 Hz for 30 s,
        # messages without bridge/plugin are harmlessly lost), entities spawn 2 s later, so the first
        # detach arrives within ~0.2 s of the attach.
        detach_pubs = [
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--times', '150', '-r', '5',
                     f'/{name}/{suffix}', 'std_msgs/msg/Empty', '{}'],
                output='log',
            )
            for name, _k, _kind, _x, _y in entities
            for suffix in ('detach', 'detach_left')      # both DetachableJoints
        ]
        return detach_pubs + [TimerAction(period=2.0, actions=actions)]

    spawn_test_books_arg = DeclareLaunchArgument(
        'spawn_test_books', default_value='true',
        description='Spawn the graspable physical entities of the scene '
                    '(book_placer.test_entities; false = none: with '
                    'scene:=grasp_test only bookshelf and table remain)',
    )

    # GraspManagerNode (agibot_x2_pkg_py/gripper_controller.py): single attach/detach interface
    # (/gripper/right/attach String: entity name or '' = what the finger touches, /gripper/right/detach
    # Empty, /gripper/right/attached latched) plus finger contact log; forwards to /<name>/attach|detach.
    grasp_manager_arg = DeclareLaunchArgument(
        'grasp_manager', default_value='true',
        description="Start GraspManagerNode (global attach/detach "
                    "/gripper/right/attach|detach, contact log)")
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
        """
        Start the Gazebo<->ROS bridge with the config generated for the scene (static
        config/gz_bridge.yaml + attach/detach/state of each test entity), plus image bridge and grasp manager
        An invalid static YAML fails the launch here instead of leaving a silently dead bridge.
        """
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
        return [LogInfo(msg=f'bridge config generated: {cfg}'),
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

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/head_camera/image",          # head camera, triggered rgbd
            "/head_camera/depth_image",
            "/video_camera/image",      # director camera (video:=true), silent otherwise
            "/video_camera_table/image",  # table camera (video:=true), silent otherwise
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

    # relay camera_info to <topic>/image/camera_info, the layout expected by the RViz2 Camera plugin
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
            'waist_roll_controller',
            # kinematic walking: virtual base + legs
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

    # Test entities spawn (attached to the finger) only after the controller spawner exits
    spawn_test_books_action = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[OpaqueFunction(function=_spawn_test_books)],
        )
    )

    head_camera_warmup = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[TimerAction(period=5.0, actions=[ExecuteProcess(
                cmd=['gz', 'topic', '-t', '/head_camera/trigger', '-m', 'gz.msgs.Boolean',
                     '-p', 'data: true'],
                name='head_camera_warmup', output='screen')])],
        )
    )

    # tell Gazebo where to find the agibot_x2_pkg meshes
    set_gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg)  # points to install/agibot_x2_pkg/share
    )

    # start the joint controllers only AFTER the broadcaster spawner has exited
    delay_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    launchDescriptionObject = LaunchDescription()

    # the environment variable must be set BEFORE launching Gazebo
    launchDescriptionObject.add_action(set_gz_model_path)

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(walk_distance_arg)
    launchDescriptionObject.add_action(finger_mu_arg)
    launchDescriptionObject.add_action(video_arg)
    launchDescriptionObject.add_action(spawn_video_camera)
    launchDescriptionObject.add_action(spawn_video_camera_table)
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
    # RViz after 15 s: at startup it takes a full core and starves the Gazebo server loading the world
    launchDescriptionObject.add_action(TimerAction(period=15.0, actions=[rviz_node]))
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(spawn_full_scene_action)
    launchDescriptionObject.add_action(spawn_test_books_arg)
    launchDescriptionObject.add_action(spawn_test_books_action)
    launchDescriptionObject.add_action(head_camera_warmup)
    # bridges start ONLY after the robot spawn: started with Gazebo, under load the /clock subscription
    # could never connect (ROS time 0, no /tf); `create` exits only once the world is ready
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


    unpause_world = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/bookshelf_world/control',
                 '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '5000', '--req', 'pause: false'],
            name='unpause_world', output='screen')])
    launchDescriptionObject.add_action(unpause_world)

    # spawners SERIALIZED: they share a global ros2_control lock and, run together on a saturated server,
    # one fails to acquire it and dies; broadcaster first, controllers via delay_controllers_spawner
    launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    launchDescriptionObject.add_action(delay_controllers_spawner)

    return launchDescriptionObject