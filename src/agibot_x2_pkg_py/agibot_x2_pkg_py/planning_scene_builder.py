#!/usr/bin/env python3
"""
planning_scene_builder - la planning scene di MoveIt dalla percezione
(2026-09-18).

Mette nella planning scene di move_group (servizio /apply_planning_scene):

  * gli OGGETTI misurati dalla depth della head_camera: ogni voce di
    /tmp/x2_detections.json (o del topic /library_manager/detections) con
    misure 3D diventa un box "obj<id>": larghezza = thickness_m,
    profondita' = length_m (0 = non visibile -> DEFAULT_DEPTH), altezza
    z_top - z_bottom, faccia frontale a world_x, centro laterale a world_y.
    Il file viene riletto quando cambia (nuova foto), il topic quando arriva.
  * la LIBRERIA: ripiani, pareti e fondo. Le quote dei ripiani vengono da
    book_placer.SHELF_SURFACES_Z e la posa (x 0.40, yaw 90) e' quella di
    default del launch: NON ancora misurate (limite noto, vedi Stato.md).
    Il ripiano su cui stanno gli oggetti rilevati viene pero' ALLINEATO alla
    quota misurata (min z_bottom delle detection) quando disponibile.
  * il TAVOLO: piano + 4 gambe, posa di default del launch (limite noto).

Oggetto in mano: pick_test_book pubblica su /scene_builder/attach
"<lato>:obj<id>" quando incolla l'oggetto (DetachableJoint) e su
/scene_builder/detach "<lato>" quando lo lascia: il box passa da oggetto del
mondo ad AttachedCollisionObject di <lato>_gripper_base_link (dita come
touch_links), cosi' il trasporto e' pianificato tenendo conto dell'oggetto;
al detach torna nel mondo dov'e' stato lasciato. /scene_builder/remove
"obj<id>" toglie un oggetto (es. posato sul tavolo e non piu' d'interesse).

  ros2 run agibot_x2_pkg_py planning_scene_builder
"""
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
import tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK
from moveit_msgs.msg import PlanningSceneComponents

from agibot_x2_pkg.book_placer import (SHELF_SURFACES_Z, SHELF_HALF_INNER_WIDTH, ROBOT_SPAWN_X, WALK_DISTANCE,
                                       BOOK_COLLISION_SIDE_MARGIN)

# Il frame "world" di MoveIt e' la RADICE dell'URDF, che Gazebo mette nella
# posa di spawn del modello (launch: x = ROBOT_SPAWN_X - walk_distance, z
# 0.662). Le detection e la geometria qui sotto sono in coordinate Gazebo:
# vanno traslate di -spawn prima di entrare nella scena (2026-09-18: senza
# questa traslazione la scena stava 1.6 m dietro e 66 cm sopra il robot e
# MoveIt ha pianificato dentro lo scaffale).
SPAWN_DEFAULT = (ROBOT_SPAWN_X - WALK_DISTANCE, 0.0, 0.662)

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# ─── geometria della libreria (posa di default del launch: x 0.40, yaw 90) ───
SHELF_X_FRONT = 0.25        # bordo anteriore dei ripiani (PLANK_FRONT_X di pick_test_book)
SHELF_X_BACK = 0.55         # retro del mobile
SHELF_BACK_PANEL_X = 0.528  # faccia interna del fondo
SHELF_HALF_OUTER = 0.40     # meta' larghezza esterna
SHELF_PLANK_T = 0.022       # spessore dei ripiani
SHELF_TOP_Z = 1.35          # cima del mobile
DEFAULT_DEPTH = 0.16        # profondita' di un libro con la faccia superiore non visibile

# ─── tavolo (launch: lx -0.93, ly 0.80 nel frame libreria -> world) ───
TABLE_CENTER = (-0.40, -0.93)          # centro del piano nel mondo
TABLE_SIZE = (0.90, 1.20, 0.045)       # dopo la rotazione di 90 gradi: x 0.9, y 1.2
TABLE_TOP_CENTER_Z = 0.7275
TABLE_LEG = (0.04, 0.04, 0.705)
TABLE_LEG_XY = [(0.40, 0.55), (-0.40, 0.55), (0.40, -0.55), (-0.40, -0.55)]   # nel mondo (gia' ruotati)


_SPAWN = list(SPAWN_DEFAULT)      # aggiornato dal nodo (parametri robot_spawn_x/y/z)


def _matrix_to_quat(R):
    """Quaternione geometry_msgs/Quaternion da una matrice di rotazione 3x3."""
    from geometry_msgs.msg import Quaternion
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        r = math.sqrt(1.0 + t); w = 0.5 * r; r = 0.5 / r
        x, y, z = (R[2][1] - R[1][2]) * r, (R[0][2] - R[2][0]) * r, (R[1][0] - R[0][1]) * r
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        r = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]); x = 0.5 * r; r = 0.5 / r
        w, y, z = (R[2][1] - R[1][2]) * r, (R[0][1] + R[1][0]) * r, (R[0][2] + R[2][0]) * r
    elif R[1][1] > R[2][2]:
        r = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]); y = 0.5 * r; r = 0.5 / r
        w, x, z = (R[0][2] - R[2][0]) * r, (R[0][1] + R[1][0]) * r, (R[1][2] + R[2][1]) * r
    else:
        r = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]); z = 0.5 * r; r = 0.5 / r
        w, x, y = (R[1][0] - R[0][1]) * r, (R[0][2] + R[2][0]) * r, (R[1][2] + R[2][1]) * r
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


def _quat_to_matrix(x, y, z, w):
    """Matrice di rotazione 3x3 da un quaternione (x,y,z,w)."""
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return [
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ]


def _box(name, size, center, frame="world"):
    """center in coordinate GAZEBO; il box viene espresso nel frame radice."""
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    pose = Pose()
    pose.position.x = float(center[0]) - _SPAWN[0]
    pose.position.y = float(center[1]) - _SPAWN[1]
    pose.position.z = float(center[2]) - _SPAWN[2]
    pose.orientation.w = 1.0
    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def _stack(name, profile, x_front, y_c, length, band=0.01, frame="world"):
    """CollisionObject con una primitiva per fascia del profilo [(z, larghezza)].
    Oggetto "tondo" (la larghezza varia lungo z di oltre il 25 %: sfera su
    base, vaso...): CILINDRI di diametro = larghezza della fascia, cosi' gli
    spigoli inesistenti non bloccano il palmo della pinza che arriva di
    sbieco (2026-09-18: box 6.2x6.2 cm per l'equatore del mappamondo ->
    l'angolo del palmo, ruotato di 32 gradi, "entrava" nell'angolo del box
    dove la sfera non c'e'). Oggetto a sezione costante (portapenne): BOX,
    che copre anche gli spigoli di una pianta quadrata."""
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    widths = [float(w) for _z, w in profile if float(w) > 0.003]
    round_obj = bool(widths) and (max(widths) - min(widths)) / max(widths) > 0.25
    for z, w in profile:
        w = float(w)
        if w <= 0.003:
            continue
        depth = float(length) if length > 0.0 else w
        prim = SolidPrimitive()
        if round_obj:
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [band, max(w, depth) / 2.0]     # [altezza, raggio]
            depth = max(w, depth)
        else:
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [depth, w, band]
        pose = Pose()
        pose.position.x = float(x_front) + depth / 2.0 - _SPAWN[0]
        pose.position.y = float(y_c) - _SPAWN[1]
        pose.position.z = float(z) - _SPAWN[2]
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


class PlanningSceneBuilder(Node):
    _quat_to_matrix = staticmethod(_quat_to_matrix)

    def __init__(self):
        super().__init__("planning_scene_builder")
        self.declare_parameter("detections_file", "/tmp/x2_detections.json")
        self.declare_parameter("poll_s", 2.0)
        self.declare_parameter("static_scene", True)
        self.declare_parameter("robot_spawn_x", float(SPAWN_DEFAULT[0]))
        self.declare_parameter("robot_spawn_y", float(SPAWN_DEFAULT[1]))
        self.declare_parameter("robot_spawn_z", float(SPAWN_DEFAULT[2]))
        _SPAWN[:] = [float(self.get_parameter(f"robot_spawn_{a}").value) for a in "xyz"]
        self.get_logger().info(f"frame radice dell'URDF (world di MoveIt) alla posa di spawn {_SPAWN} (coord. Gazebo)")
        self.det_file = str(self.get_parameter("detections_file").value)
        self._seeded = False
        self._pending_topic = None
        self._mtime = None
        self._objects = {}          # id -> CollisionObject (nel mondo)
        self._attached = {}         # lato -> obj id
        self._plank_z = None
        cbg = ReentrantCallbackGroup()   # 2026-09-18: _apply() blocca sincrono su threading.Event
        # TF (2026-09-18 sera): serve per esprimere la posa dell'oggetto
        # AGGANCIATO nel frame della mano, non nel mondo - vedi _attach_cb.
        self._fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene", callback_group=cbg)
        self.create_subscription(String, "/library_manager/detections", self._det_cb, LATCHED, callback_group=cbg)
        self.create_subscription(String, "/scene_builder/attach", self._attach_cb, 10, callback_group=cbg)
        self.create_subscription(String, "/scene_builder/detach", self._detach_cb, 10, callback_group=cbg)
        self.create_subscription(String, "/scene_builder/place", self._place_cb, 10, callback_group=cbg)
        self._placed = {}
        self.create_subscription(String, "/scene_builder/remove", self._remove_cb, 10, callback_group=cbg)
        self.create_subscription(Empty, "/scene_builder/refresh", lambda _m: self._reload(force=True), 10, callback_group=cbg)
        self.pub_state = self.create_publisher(String, "/scene_builder/state", LATCHED)
        self._ready = False
        # 2026-09-19: orologio REALE (steady), non il tempo simulato: con use_sim_time e la
        # simulazione rallentata dal rilevatore (RTF < 0.05) i 2 s del timer diventavano minuti,
        # e il builder non arrivava mai a "move_group pronto" (scena vuota per MoveIt).
        self.create_timer(float(self.get_parameter("poll_s").value), self._tick, callback_group=cbg,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"planning_scene_builder: aspetto /apply_planning_scene; detection da {self.det_file}")

    # ─── servizio ────────────────────────────────────────────────────────
    def _apply(self, scene: PlanningScene, what: str, timeout_s: float = 10.0) -> bool:
        """Chiamata SINCRONA al vero risultato del servizio, senza mai
        richiamare rclpy.spin*() da dentro un callback (l'executor e' gia'
        occupato a spinnare: vedi nota sopra). Un threading.Event, risolto
        dal done-callback che gira su un ALTRO thread del
        MultiThreadedExecutor, sblocca l'attesa qui."""
        scene.is_diff = True
        if not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{what}: /apply_planning_scene non disponibile (move_group non ancora su?)")
            return False
        fut = self.cli.call_async(ApplyPlanningScene.Request(scene=scene))
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        got = done.wait(timeout=timeout_s)
        if not got:
            self.get_logger().error(f"planning scene: {what} -> TIMEOUT ({timeout_s:.0f} s, nessuna risposta)")
            return False
        try:
            ok = bool(fut.result() is not None and fut.result().success)
        except Exception as e:
            ok = False
            self.get_logger().error(f"planning scene: {what}: {e}")
        if ok:
            self.get_logger().info(f"planning scene: {what} -> ok")
        else:
            self.get_logger().error(f"planning scene: {what} -> FALLITO")
        return ok

    # ─── scena statica ───────────────────────────────────────────────────
    def _static_objects(self):
        objs = []
        surfaces = list(SHELF_SURFACES_Z)
        if self._plank_z is not None:
            # allinea alla quota misurata il ripiano piu' vicino
            k = min(range(len(surfaces)), key=lambda i: abs(surfaces[i] - self._plank_z))
            if abs(surfaces[k] - self._plank_z) < 0.05:
                surfaces[k] = self._plank_z
        depth = SHELF_X_BACK - SHELF_X_FRONT
        xc = (SHELF_X_FRONT + SHELF_X_BACK) / 2.0
        for i, z in enumerate(surfaces):
            objs.append(_box(f"shelf_plank_{i}", (depth, 2 * SHELF_HALF_OUTER, SHELF_PLANK_T),
                             (xc, 0.0, z - SHELF_PLANK_T / 2.0)))
        objs.append(_box("shelf_top", (depth, 2 * SHELF_HALF_OUTER, SHELF_PLANK_T),
                         (xc, 0.0, SHELF_TOP_Z - SHELF_PLANK_T / 2.0)))
        wall_t = SHELF_HALF_OUTER - SHELF_HALF_INNER_WIDTH
        for name, y in (("shelf_wall_plus", SHELF_HALF_INNER_WIDTH + wall_t / 2.0),
                        ("shelf_wall_minus", -(SHELF_HALF_INNER_WIDTH + wall_t / 2.0))):
            objs.append(_box(name, (depth, wall_t, SHELF_TOP_Z), (xc, y, SHELF_TOP_Z / 2.0)))
        back_t = SHELF_X_BACK - SHELF_BACK_PANEL_X
        objs.append(_box("shelf_back", (back_t, 2 * SHELF_HALF_OUTER, SHELF_TOP_Z),
                         (SHELF_BACK_PANEL_X + back_t / 2.0, 0.0, SHELF_TOP_Z / 2.0)))
        objs.append(_box("table_top", TABLE_SIZE, (TABLE_CENTER[0], TABLE_CENTER[1], TABLE_TOP_CENTER_Z)))
        for k, (dx, dy) in enumerate(TABLE_LEG_XY):
            objs.append(_box(f"table_leg_{k}", TABLE_LEG,
                             (TABLE_CENTER[0] + dx, TABLE_CENTER[1] + dy, TABLE_LEG[2] / 2.0)))
        return objs

    # ─── oggetti dalla percezione ────────────────────────────────────────
    def _objects_from(self, dets):
        out = {}
        zb = []
        for d in dets:
            th = float(d.get("thickness_m", 0.0) or 0.0)
            if th <= 0.0:
                continue
            h = float(d["z_top"]) - float(d["z_bottom"])
            depth = float(d.get("length_m", 0.0) or 0.0) or DEFAULT_DEPTH
            if d.get("is_book"):
                # In Gazebo la collision dei libri e' piu' stretta della mesh
                # di BOOK_COLLISION_SIDE_MARGIN per lato (book_placer): la
                # depth misura la mesh, la scena deve riprodurre la geometria
                # fisica della simulazione (2026-09-18: con lo spessore della
                # mesh il palmo della pinza, largo 10 cm, "toccava" Alba di
                # 3 mm al pre-grasp del portapenne; con la collision vera
                # resta ~1 mm). Non e' un margine allentato: e' la collision.
                th = max(0.03, th - 2.0 * BOOK_COLLISION_SIDE_MARGIN)
            else:
                # oggetti: pianta ~quadrata se la profondita' non e' misurata
                depth = float(d.get("length_m", 0.0) or 0.0) or th
            x_front = float(d["world_x"])
            name = f"obj{int(d['id'])}"
            prof = d.get("width_profile") or []
            if not d.get("is_book") and len(prof) >= 3:
                # Oggetto NON libro con profilo di larghezza (2026-09-18): pila
                # di box, uno per fascia di 1 cm, largo quanto misurato a quella
                # quota. Una sfera su base non e' un parallelepipedo: con il box
                # unico il palmo della pinza "toccava" gli spigoli alti che non
                # esistono. Profondita': la larghezza della fascia (pianta ~
                # quadrata) se la profondita' non e' misurata.
                out[name] = _stack(name, prof, x_front, float(d["world_y"]),
                                   float(d.get("length_m", 0.0) or 0.0))
            else:
                out[name] = _box(name, (depth, th, h),
                                 (x_front + depth / 2.0, float(d["world_y"]), (float(d["z_top"]) + float(d["z_bottom"])) / 2.0))
            zb.append(float(d["z_bottom"]))
        if zb:
            self._plank_z = float(min(zb))
        return out

    def _publish_all(self, what):
        scene = PlanningScene()
        # oggetti spariti dal nuovo JSON: rimossi (tranne quelli in mano)
        attached_ids = set(self._attached.values())
        scene.world.collision_objects = []
        for co in self._static_objects():
            scene.world.collision_objects.append(co)
        for name, co in self._objects.items():
            if name not in attached_ids:
                scene.world.collision_objects.append(co)
        ok = self._apply(scene, what)
        self._publish_state(ok)
        return ok

    def _publish_state(self, ok=True):
        self.pub_state.publish(String(data=json.dumps({
            "objects": sorted(self._objects.keys()), "attached": dict(self._attached),
            "plank_z": self._plank_z, "placed": dict(self._placed), "ok": ok})))

    def _load_json(self, text, what):
        try:
            dets = json.loads(text)
        except Exception as e:
            self.get_logger().error(f"{what}: JSON non valido ({e})")
            return False
        new = self._objects_from(dets)
        # 2026-09-19: gli oggetti gia' POSATI sul tavolo (place) non compaiono piu' nelle
        # foto successive (la seconda foto e' solo libri): senza questo venivano
        # rimossi dalla scena e MoveIt non li vedeva piu' quando il busto ruota verso
        # il tavolo per mostrare un libro alla testa.
        gone = [n for n in self._objects if n not in new and n not in self._attached.values()
                and n not in self._placed]
        if gone:
            scene = PlanningScene()
            for n in gone:
                co = CollisionObject(); co.id = n; co.header.frame_id = "world"; co.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(co)
            self._apply(scene, f"rimuovo {gone}")
        for n in list(self._attached.values()):
            if n in self._objects:
                new[n] = self._objects[n]          # l'oggetto in mano non cambia
            elif n in new:
                pass                               # resta quello del JSON (non ripubblicato: e' attaccato)
        for n in self._placed:
            if n in self._objects:
                new[n] = self._objects[n]          # resta dov'e' stato posato, non dov'era sullo scaffale
        self._objects = new
        self.get_logger().info(f"{what}: {len(new)} oggetti -> {sorted(new)} (ripiano a z={self._plank_z})")
        return self._publish_all(what)

    def _reload(self, force=False):
        try:
            m = os.path.getmtime(self.det_file)
        except OSError:
            return
        if force or m != self._mtime:
            self._mtime = m
            with open(self.det_file, encoding="utf-8") as f:
                self._load_json(f.read(), f"detection da {os.path.basename(self.det_file)}")

    def _det_cb(self, msg):
        if not getattr(self, "_seeded", False):
            self._pending_topic = msg.data      # prima il seed degli oggetti attaccati
            return
        self._load_json(msg.data, "detection dal topic")

    def _seed_attached(self):
        """All'avvio: cosa e' GIA' attaccato alla pinza nella scena di
        move_group (es. builder riavviato con l'oggetto in mano): non va
        rimesso nel mondo."""
        cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        if not cli.wait_for_service(timeout_sec=2.0):
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        fut = cli.call_async(req)

        def _done(f):
            try:
                for aco in f.result().scene.robot_state.attached_collision_objects:
                    side = "left" if aco.link_name.startswith("left") else "right"
                    self._attached[side] = aco.object.id
                    self.get_logger().info(f"scena esistente: {aco.object.id} attaccato a {aco.link_name}")
            except Exception as e:
                self.get_logger().warn(f"get_planning_scene: {e}")
            self._seeded = True
            pend = getattr(self, "_pending_topic", None)
            if pend:
                self._load_json(pend, "detection dal topic")
            else:
                self._reload(force=True)
        fut.add_done_callback(_done)

    def _tick(self):
        if not self._ready:
            if not self.cli.wait_for_service(timeout_sec=0.1):
                return
            self._ready = True
            self.get_logger().info("move_group pronto: pubblico libreria, tavolo e oggetti")
            self._seed_attached()
            return
            if not self._objects:
                self._publish_all("scena statica")
            return
        self._reload()

    # ─── attach / detach ─────────────────────────────────────────────────
    def _attach_cb(self, msg):
        # 2026-09-19: log d'ingresso - trovato dal vivo un ATTACH senza
        # traccia ne' di successo ne' di FALLITO/TIMEOUT: serve sapere se il
        # callback e' mai stato invocato prima di indagare piu' a fondo.
        self.get_logger().info(f"attach richiesto: '{msg.data}'")
        try:
            side, name = msg.data.strip().split(":", 1)
        except ValueError:
            self.get_logger().error(f"attach: atteso '<lato>:obj<id>', ricevuto '{msg.data}'")
            return
        co = self._objects.get(name)
        if co is None:
            self.get_logger().error(f"attach: '{name}' non e' nella scena {sorted(self._objects)}")
            return
        link = f"{side}_gripper_base_link"
        # 2026-09-18 (sera): trovato dal vivo - MoveIt NON trasforma la posa da
        # solo anche se header.frame_id = "world": la (ri)etichetta col nome
        # del link e usa i numeri COSI' COM'ERANO (coordinate mondo prese alla
        # lettera come offset dalla mano) -> l'oggetto agganciato finiva a 1.9 m
        # dalla mano, dentro al tavolo per puro caso. Serve la trasformazione
        # vera: guardo la TF mondo -> link ADESSO e riesprimo la posa lì.
        try:
            # 2026-09-19 (sera): il frame TF "world" NON esiste (radice TF = pelvis): il lookup
            # falliva a OGNI attach ("world passed to lookupTransform ... does not exist") e
            # l'oggetto finiva nella scena a ~1.9 m dalla mano, quindi durante sfilata, mostra e
            # rimessa MoveIt non vedeva il libro vero (e dopo il detach lo lasciava in un punto
            # a caso: obj2/obj3 fantasma). Ora la posa della mano nel frame di pianificazione
            # ("world" di MoveIt, lo stesso degli oggetti) viene da /compute_fk di move_group.
            Rm, tv = self._world_to_link(link)
            poses_link = []
            for p in co.primitive_poses:
                wp = (p.position.x, p.position.y, p.position.z)
                lp = tuple(Rm[i][0] * wp[0] + Rm[i][1] * wp[1] + Rm[i][2] * wp[2] + tv[i] for i in range(3))
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = lp
                # 2026-09-19 (sera): l'orientamento NON va lasciato com'e' (identita' del mondo) ma
                # espresso nel frame del link: con la pinza ruotata di 40-55 gradi (presa tipica) il
                # box del libro agganciato risultava inclinato di altrettanto rispetto al libro vero,
                # quindi i controlli di collisione in mano non lo proteggevano (libri 1-3 buttati a
                # terra nel rientro del libro 2). R_link<-mondo x R_box(mondo).
                qb = p.orientation
                Rb = _quat_to_matrix(qb.x, qb.y, qb.z, qb.w)
                Rn = [[sum(Rm[i][k] * Rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
                pose.orientation = _matrix_to_quat(Rn)
                poses_link.append(pose)
        except Exception as e:
            self.get_logger().error(f"attach {name}: TF {link}<-world non disponibile ({e}) - "
                                    "l'oggetto agganciato NON avra' una posa corretta nella scena")
            poses_link = co.primitive_poses
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object = CollisionObject()
        aco.object.id = name
        aco.object.header.frame_id = link
        aco.object.operation = CollisionObject.ADD
        aco.object.primitives = co.primitives
        aco.object.primitive_poses = poses_link
        aco.touch_links = [link, f"{side}_gripper_left_finger_link",
                           f"{side}_gripper_right_finger_link", f"{side}_wrist_yaw_link"]
        # via dal mondo, poi attaccato, in DUE chiamate separate (2026-09-19:
        # ogni ATTACH tentato finora e' FALLITO - success=False dal servizio,
        # nessuna eccezione/timeout, per oggetti diversi e su entrambi i lati,
        # in ore/riavvii diversi - vedi Bugs.md. L'unica cosa in comune a
        # TUTTI i fallimenti e assente da TUTTI i diff che invece riescono
        # sempre (world-only, per le detection) e' un REMOVE dal mondo e un
        # ADD agganciato per lo STESSO id nello stesso messaggio di diff.
        # Ipotesi da verificare dal vivo: il servizio non gestisce bene le
        # due meta' del diff insieme - qui le separo in due chiamate sincrone.
        # SOLO se era davvero un oggetto del mondo (2026-09-18: se e' gia'
        # agganciato, es. dopo un riavvio del nodo con il robot a meta'
        # presa, un REMOVE su un oggetto che move_group non ha nel mondo fa
        # fallire il diff, "does not exist in this scene").
        if name not in self._attached.values():
            rm = CollisionObject(); rm.id = name; rm.header.frame_id = "world"; rm.operation = CollisionObject.REMOVE
            scene_rm = PlanningScene(); scene_rm.world.collision_objects = [rm]
            if not self._apply(scene_rm, f"rimuovo {name} dal mondo (prima dell'aggancio)"):
                return
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [aco]
        scene.robot_state.is_diff = True
        if self._apply(scene, f"ATTACH {name} a {aco.link_name}"):
            self._attached[side] = name
            self._publish_state()

    def _world_to_link(self, link, timeout_s=5.0):
        """(R, t) tali che p_link = R @ p_world + t, con la posa di `link` nel frame di pianificazione
        letta da /compute_fk (stato corrente della scena di move_group)."""
        cli = self._fk_cli
        if not cli.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("/compute_fk non disponibile")
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = [link]
        fut = cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout_s) or fut.result() is None or not fut.result().pose_stamped:
            raise RuntimeError("/compute_fk senza risposta")
        ps = fut.result().pose_stamped[0].pose
        q = ps.orientation
        Rlw = _quat_to_matrix(q.x, q.y, q.z, q.w)                     # link -> world
        Rwl = [[Rlw[j][i] for j in range(3)] for i in range(3)]       # world -> link (trasposta)
        pw = (ps.position.x, ps.position.y, ps.position.z)
        tv = tuple(-sum(Rwl[i][k] * pw[k] for k in range(3)) for i in range(3))
        self.get_logger().info(f"attach: {link} in world a ({pw[0]:+.3f}, {pw[1]:+.3f}, {pw[2]:+.3f}) da /compute_fk")
        return Rwl, tv

    def _detach_cb(self, msg):
        self.get_logger().info(f"detach richiesto: '{msg.data}'")
        side = msg.data.strip()
        name = self._attached.get(side)
        if not name:
            self.get_logger().warn(f"detach {side}: niente in mano nella scena")
            return
        aco = AttachedCollisionObject()
        aco.link_name = f"{side}_gripper_base_link"
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE     # torna nel mondo dov'e'
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [aco]
        scene.robot_state.is_diff = True
        if self._apply(scene, f"DETACH {name} da {aco.link_name}"):
            del self._attached[side]
            self._publish_state()

    @staticmethod
    def _extents(co):
        """(xmin, xmax, ymin, ymax, zmin) di un CollisionObject (frame radice)."""
        xs, ys, zs = [], [], []
        for prim, pose in zip(co.primitives, co.primitive_poses):
            d = list(prim.dimensions)
            if prim.type == SolidPrimitive.CYLINDER:
                hx = hy = d[1]; hz = d[0] / 2.0
            else:
                hx, hy, hz = d[0] / 2.0, d[1] / 2.0, d[2] / 2.0
            xs += [pose.position.x - hx, pose.position.x + hx]
            ys += [pose.position.y - hy, pose.position.y + hy]
            zs.append(pose.position.z - hz)
        return min(xs), max(xs), min(ys), max(ys), min(zs)

    def _place_cb(self, msg):
        """"<nome>:<x>:<y>:<z_base>" (coordinate GAZEBO): mette l'oggetto (NON in
        mano) dove sta DAVVERO dopo il rilascio - base sul piano del tavolo,
        centro sotto la pinza. 2026-09-19: dopo il DETACH move_group lo
        lascia dov'era la mano al rilascio (3 cm SOPRA il tavolo, TABLE_RELEASE_AIR)
        mentre in Gazebo cade sul piano: nella scena il mappamondo era 3 cm
        piu' alto del vero e toccava la spalla destra nella posa di rilascio
        del portapenne (obj5<->right_shoulder_yaw_link, "Unable to sample any
        valid states for goal tree")."""
        self.get_logger().info(f"place richiesto: '{msg.data}'")
        try:
            name, xs, ys, zs = msg.data.strip().split(":")
            x, y, zb = float(xs), float(ys), float(zs)
        except ValueError:
            self.get_logger().error(f"place: atteso '<nome>:<x>:<y>:<z_base>', ricevuto '{msg.data}'")
            return
        co = self._objects.get(name)
        if co is None:
            self.get_logger().error(f"place: '{name}' non e' nella scena {sorted(self._objects)}")
            return
        if name in self._attached.values():
            self.get_logger().warn(f"place: {name} e' ancora in mano, non lo sposto")
            return
        x0, x1, y0, y1, z0 = self._extents(co)
        dx = (x - _SPAWN[0]) - (x0 + x1) / 2.0
        dy = (y - _SPAWN[1]) - (y0 + y1) / 2.0
        dz = (zb - _SPAWN[2]) - z0
        new = CollisionObject()
        new.header.frame_id = co.header.frame_id
        new.id = name
        new.operation = CollisionObject.ADD          # ADD su un id esistente lo sostituisce
        new.primitives = co.primitives
        poses = []
        for p in co.primitive_poses:
            q = Pose()
            q.position.x, q.position.y, q.position.z = p.position.x + dx, p.position.y + dy, p.position.z + dz
            q.orientation = p.orientation
            poses.append(q)
        new.primitive_poses = poses
        scene = PlanningScene()
        scene.world.collision_objects = [new]
        if self._apply(scene, f"PLACE {name} sul tavolo (spostato di {dx*100:+.1f}, {dy*100:+.1f}, {dz*100:+.1f} cm)"):
            self._objects[name] = new
            self._placed[name] = [x, y, zb]
            self._publish_state()

    def _remove_cb(self, msg):
        name = msg.data.strip()
        rm = CollisionObject(); rm.id = name; rm.header.frame_id = "world"; rm.operation = CollisionObject.REMOVE
        scene = PlanningScene(); scene.world.collision_objects = [rm]
        if self._apply(scene, f"rimuovo {name}"):
            self._objects.pop(name, None)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneBuilder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
