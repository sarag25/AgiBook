"""
shelf_geometry - dalla profondita' della shelf_camera alla geometria 3D dei
libri (2026-09-13).

La shelf_camera (bookshelf.urdf) e' un sensore rgbd A SCATTO fisso alla
libreria: colore e profondita' hanno gli stessi intrinseci e la stessa
posa, nota per costruzione (posa della libreria nel mondo + origine del
joint della camera). Per ogni oggetto rilevato (bbox in pixel, da SAM3 /
YOLO / DepthShelfDetector) i pixel della bbox vengono riproiettati in
punti 3D nel mondo e da li' si misurano:

  world_x    x mondo della faccia frontale (il dorso rivolto al robot)
  world_y    y mondo del centro del dorso
  thickness  spessore (estensione laterale della faccia frontale)
  z_bottom / z_top / height
  free_plus / free_minus   spazio libero ai lati fino al vicino o alla parete

cioe' esattamente cio' che pick_test_book prendeva da book_placer (posa e
misure note a priori). Con queste misure la presa non dipende piu' da
quale libro e' ne' da dove sta.

Convenzioni Gazebo: il frame del sensore ha X in avanti (asse ottico), Y a
sinistra, Z in alto; la colonna u cresce verso -Y, la riga v verso -Z; la
depth e' la distanza LUNGO l'asse ottico (non lungo il raggio). Il frame
della libreria ha +y locale = lato aperto (verso il robot), x locale =
asse laterale; con shelf_yaw = 90 gradi, x locale -> +y mondo e +y locale
-> -x mondo (il robot guarda +x).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# bookshelf.urdf: shelf_camera_joint (origine nel frame della libreria)
SHELF_CAM_XYZ = (-0.16, 0.75, 1.50)
SHELF_CAM_RPY = (0.0, 0.55, -1.5707963)
SHELF_CAM_HFOV = 1.0
SHELF_CAM_SIZE = (960, 720)
# control_file.gazebo: head_camera (rgbd a scatto sulla testa, 2026-09-16)
HEAD_CAM_HFOV = 1.0
HEAD_CAM_SIZE = (1920, 1440)
HEAD_CAM_LINK = "rgbd_head_front_link"
HEAD_SENSOR_RPY = (-1.5707963, -1.5707963, 0.0)   # <pose> del sensore nel link

# bookshelf.urdf: geometria interna (frame libreria)
SHELF_HALF_INNER_WIDTH = 0.378      # pareti interne a x_local = +-0.378
SHELF_BACK_INNER_Y = -0.128         # faccia interna del pannello posteriore
SHELF_FRONT_Y = 0.150               # bordo anteriore dei ripiani
SHELF_SURFACES_Z = (0.022, 0.349, 0.671, 0.993)   # piani d'appoggio
SHELF_BOARD = 0.022
SHELF_COMPARTMENT_H = 0.300         # altezza libera sopra ogni piano (0.335 l'ultimo)


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


@dataclass
class ObjectGeometry:
    world_x: float          # faccia frontale (dorso) - x mondo
    world_y: float          # centro laterale - y mondo
    z_bottom: float
    z_top: float
    thickness: float
    height: float
    depth_m: float          # distanza media dalla camera
    n_points: int
    length: float = 0.0     # profondita' del libro (dorso -> taglio), 0 = faccia superiore non visibile
    lateral: float = 0.0    # coordinata laterale nel frame libreria (x locale)
    front: float = 0.0      # coordinata "verso il robot" nel frame libreria (y locale)
    lat_min: float = 0.0
    lat_max: float = 0.0
    free_plus: float = 0.0  # spazio libero verso +y mondo (= +lateral con yaw 90)
    free_minus: float = 0.0


class ShelfGeometry:

    def __init__(self, shelf_x: float = 0.40, shelf_y: float = 0.0, shelf_yaw: float = math.pi / 2,
                 cam_xyz=SHELF_CAM_XYZ, cam_rpy=SHELF_CAM_RPY, hfov: float = SHELF_CAM_HFOV,
                 size=SHELF_CAM_SIZE):
        self.shelf_xy = np.array([shelf_x, shelf_y], dtype=float)
        self.shelf_yaw = float(shelf_yaw)
        c, s = math.cos(shelf_yaw), math.sin(shelf_yaw)
        self.R_ws = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])   # libreria -> mondo
        self.t_ws = np.array([shelf_x, shelf_y, 0.0])
        R_sc = rpy_matrix(*cam_rpy)                                          # camera -> libreria
        self.R_wc = self.R_ws @ R_sc                                          # camera -> mondo
        self.t_wc = self.t_ws + self.R_ws @ np.asarray(cam_xyz, dtype=float)
        self.W, self.H = size
        self.fx = (self.W / 2.0) / math.tan(hfov / 2.0)
        self.fy = self.fx
        self.cx, self.cy = self.W / 2.0, self.H / 2.0
        self.depth: np.ndarray | None = None
        # assi della libreria nel mondo: laterale (x locale) e frontale (y locale)
        self.lat_axis = self.R_ws[:, 0]
        self.front_axis = self.R_ws[:, 1]

    # ─── modello di camera ────────────────────────────────────────────────

    def set_camera(self, R_wc: np.ndarray, t_wc, hfov: float, size):
        """Camera in una posa QUALSIASI nel mondo (2026-09-16): R_wc
        camera->mondo (colonne: asse ottico, sinistra, alto immagine), t_wc
        posizione. Serve per la camera della testa, la cui posa cambia a
        ogni scatto (vedi HeadCameraPose); la shelf_camera fissa resta
        quella del costruttore."""
        self.R_wc = np.asarray(R_wc, dtype=float).copy()
        self.t_wc = np.asarray(t_wc, dtype=float).copy()
        self.W, self.H = size
        self.fx = (self.W / 2.0) / math.tan(hfov / 2.0)
        self.fy = self.fx
        self.cx, self.cy = self.W / 2.0, self.H / 2.0
        self.depth = None

    def set_depth(self, depth: np.ndarray):
        """depth: HxW float32 in metri lungo l'asse ottico (inf/nan/0 = nessun ritorno)."""
        if depth.shape != (self.H, self.W):
            # intrinseci riscalati se la risoluzione e' diversa da quella nominale
            sy, sx = depth.shape[0] / self.H, depth.shape[1] / self.W
            self.fx *= sx; self.fy *= sy; self.cx *= sx; self.cy *= sy
            self.H, self.W = depth.shape
        self.depth = depth.astype(np.float32)

    def pixels_to_world(self, u: np.ndarray, v: np.ndarray, d: np.ndarray) -> np.ndarray:
        """(u, v, depth) -> Nx3 punti nel mondo."""
        X = d
        Y = -(u - self.cx) * d / self.fx
        Z = -(v - self.cy) * d / self.fy
        pc = np.stack([X, Y, Z], axis=-1)
        return pc @ self.R_wc.T + self.t_wc

    def world_to_pixels(self, pw: np.ndarray):
        """Nx3 mondo -> (u, v, depth); serve ai test e al debug."""
        pc = (np.asarray(pw, dtype=float) - self.t_wc) @ self.R_wc
        d = pc[:, 0]
        u = self.cx - self.fx * pc[:, 1] / d
        v = self.cy - self.fy * pc[:, 2] / d
        return u, v, d

    def world_cloud(self, bbox=None, step: int = 1):
        """Punti mondo (Nx3) dei pixel validi dentro bbox (x1,y1,x2,y2) o di tutta l'immagine."""
        if self.depth is None:
            raise RuntimeError("depth non impostata (set_depth)")
        if bbox is None:
            x1, y1, x2, y2 = 0, 0, self.W, self.H
        else:
            x1, y1, x2, y2 = [int(round(b)) for b in bbox]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(self.W, x2), min(self.H, y2)
        d = self.depth[y1:y2:step, x1:x2:step]
        vv, uu = np.mgrid[y1:y2:step, x1:x2:step]
        ok = np.isfinite(d) & (d > 0.05)
        return self.pixels_to_world(uu[ok].astype(float), vv[ok].astype(float), d[ok].astype(float)), d[ok]

    def shelf_local(self, pw: np.ndarray) -> np.ndarray:
        """Nx3 mondo -> Nx3 frame libreria (lateral, front, z)."""
        return (np.asarray(pw) - self.t_ws) @ self.R_ws

    # ─── misure ───────────────────────────────────────────────────────────

    @staticmethod
    def lateral_extent(lat: np.ndarray, cell: float = 0.002, rel_min: float = 0.08):
        """Estensione laterale dell'oggetto: intervallo CONNESSO di celle
        occupate (istogramma a `cell` metri) attorno alla mediana. Serve
        perche' la camera guarda di sbieco: dentro la bbox di un libro
        finiscono anche pixel della copertina del vicino, che nel frame
        della libreria stanno oltre lo spazio vuoto fra i due -> con i
        percentili lo spessore usciva sbagliato, con l'intervallo connesso
        no (lo spazio vuoto e' una fila di celle vuote)."""
        lo, hi = float(lat.min()), float(lat.max())
        nb = max(1, int(math.ceil((hi - lo) / cell)) + 1)
        counts, edges = np.histogram(lat, bins=nb, range=(lo, lo + nb * cell))
        thr = max(2.0, rel_min * counts.max())
        med = int(min(nb - 1, (np.median(lat) - lo) // cell))
        a = b = med
        while a > 0 and counts[a - 1] >= thr:
            a -= 1
        while b < nb - 1 and counts[b + 1] >= thr:
            b += 1
        return float(edges[a]), float(edges[b + 1])

    def measure(self, bbox, face_depth: float = 0.015, shrink_v: float = 0.10) -> ObjectGeometry | None:
        """Geometria dell'oggetto dentro bbox (x1,y1,x2,y2 pixel). shrink_v
        = frazione di bbox tolta sopra e sotto (bordi = sfondo/ripiano);
        lateralmente si tiene tutto e ci pensa lateral_extent."""
        x1, y1, x2, y2 = bbox
        h = y2 - y1
        inner = (x1 + 1, y1 + shrink_v * h, x2 - 1, y2 - shrink_v * h)
        pw, d = self.world_cloud(inner)
        loc = self.shelf_local(pw) if len(pw) else np.zeros((0, 3))
        # SOLO punti dentro lo scaffale (2026-09-13, prova dal vivo): nella
        # bbox del mappamondo c'erano i pixel della testa del robot, davanti
        # alla libreria, e la "faccia frontale" usciva a x=-0.055 (la testa)
        keep = self.inside_shelf(loc)
        pw, d, loc = pw[keep], d[keep], loc[keep]
        if len(pw) < 30:
            return None
        lat, front, z = loc[:, 0], loc[:, 1], loc[:, 2]
        # faccia frontale = i punti piu' vicini al robot (front massimo)
        front_face = np.percentile(front, 90)
        face = front >= front_face - face_depth
        lat_min, lat_max = self.lateral_extent(lat[face])
        # quota: SOLO i punti della faccia frontale dentro l'estensione
        # laterale (i pixel della copertina del vicino alto finivano nella
        # bbox e gonfiavano l'altezza), su tutta la bbox in verticale
        pw_full, _ = self.world_cloud((x1 + 1, y1, x2 - 1, y2))
        loc_f = self.shelf_local(pw_full) if len(pw_full) else loc
        loc_f = loc_f[self.inside_shelf(loc_f)] if len(loc_f) else loc
        own = (loc_f[:, 1] >= front_face - face_depth) & (loc_f[:, 0] >= lat_min - 0.002) & (loc_f[:, 0] <= lat_max + 0.002)
        z_all = loc_f[own, 2] if own.sum() >= 20 else z
        z_bottom, z_top = float(np.percentile(z_all, 1)), float(np.percentile(z_all, 99))
        # lunghezza (dorso -> taglio): dalla faccia superiore, visibile dall'alto
        # solo se nessun ripiano la copre; sotto 3 cm di estensione = ignota (0)
        own_lat = (loc_f[:, 0] >= lat_min + 0.002) & (loc_f[:, 0] <= lat_max - 0.002) \
            & (loc_f[:, 2] >= z_top - 0.02)
        length = 0.0
        if own_lat.sum() >= 20:
            ext = float(front_face - np.percentile(loc_f[own_lat, 1], 2))
            if ext >= 0.03:
                length = ext
        lateral = float((lat_min + lat_max) / 2.0)
        front_c = float(front_face)
        # nel mondo: faccia frontale e centro laterale
        p_face = self.t_ws + self.R_ws @ np.array([lateral, front_c, (z_bottom + z_top) / 2])
        return ObjectGeometry(
            world_x=float(p_face[0]), world_y=float(p_face[1]),
            z_bottom=z_bottom, z_top=z_top,
            thickness=float(lat_max - lat_min), height=float(z_top - z_bottom),
            depth_m=float(np.median(d)), n_points=int(len(pw)), length=length,
            lateral=lateral, front=front_c, lat_min=float(lat_min), lat_max=float(lat_max))

    def free_space(self, geoms: list, row_tol: float = 0.06):
        """Riempie free_plus/free_minus (verso +/- lateral, = +/-y mondo con
        yaw 90) usando i vicini dello stesso ripiano e le pareti interne."""
        items = [g for g in geoms if g is not None]
        for g in items:
            same_row = [o for o in items if o is not g and abs(o.z_bottom - g.z_bottom) < row_tol]
            right = [o.lat_min for o in same_row if o.lat_min >= g.lat_max - 0.005]
            left = [o.lat_max for o in same_row if o.lat_max <= g.lat_min + 0.005]
            wall_plus = SHELF_HALF_INNER_WIDTH - g.lat_max
            wall_minus = g.lat_min + SHELF_HALF_INNER_WIDTH
            g.free_plus = float(max(0.0, min([wall_plus] + [r - g.lat_max for r in right])))
            g.free_minus = float(max(0.0, min([wall_minus] + [g.lat_min - l for l in left])))
        return geoms

    @staticmethod
    def inside_shelf(loc: np.ndarray, wall_margin: float = 0.006) -> np.ndarray:
        """Maschera dei punti (frame libreria) che stanno dentro uno scomparto:
        fra le pareti, davanti al pannello posteriore, sopra un piano e
        sotto il ripiano successivo. Usata dal detector e da measure()."""
        if len(loc) == 0:
            return np.zeros(0, dtype=bool)
        lat, front, z = loc[:, 0], loc[:, 1], loc[:, 2]
        inside = (np.abs(lat) < SHELF_HALF_INNER_WIDTH - wall_margin) \
            & (front > SHELF_BACK_INNER_Y + 0.02) & (front < SHELF_FRONT_Y + 0.06)
        above = np.zeros_like(inside)
        for s in SHELF_SURFACES_Z:
            above |= (z > s + 0.004) & (z < s + SHELF_COMPARTMENT_H - 0.01)
        return inside & above

    @staticmethod
    def surface_below(z: float):
        below = [s for s in SHELF_SURFACES_Z if s <= z + 0.005]
        return max(below) if below else None


class HeadCameraPose:
    """Posa nel mondo della camera della testa a partire dai giunti
    (2026-09-16): catena world -> base_x/y/z/yaw (base mobile cinematica) ->
    vita -> testa -> rgbd_head_front_link, dall'URDF (ArmKinematics con
    tip=HEAD_CAM_LINK), piu' la posa di SPAWN del robot (il link 'world'
    del modello): x = ROBOT_SPAWN_X - walk_distance del launch, y 0, yaw 0.
    In simulazione e' esatta: niente calibrazione. Usa la stessa
    convenzione di ShelfGeometry (X ottico, Y sinistra, Z alto)."""

    JOINTS = ("base_x_joint", "base_y_joint", "base_z_joint", "base_yaw_joint",
              "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
              "head_yaw_joint", "head_pitch_joint")

    def __init__(self, spawn_xyz=(0.0, 0.0, 0.0), spawn_yaw: float = 0.0, urdf_path: str | None = None):
        from agibot_x2_pkg_py.arm_kinematics import ArmKinematics   # pacchetto gemello, a runtime
        self.kin = (ArmKinematics.from_urdf(urdf_path, tip=HEAD_CAM_LINK) if urdf_path
                    else ArmKinematics.from_package(tip=HEAD_CAM_LINK))
        self.spawn_xyz = np.asarray(spawn_xyz, dtype=float)
        self.R_spawn = rpy_matrix(0.0, 0.0, float(spawn_yaw))
        self.R_sensor = rpy_matrix(*HEAD_SENSOR_RPY)

    def world_pose(self, joints: dict):
        """joints: {nome: valore} (da /joint_states; assenti = 0). Ritorna
        (R_wc, t_wc). I prismatici della base non sono nella FK della catena
        (ignorati da fk_link): la loro traslazione, che precede base_yaw,
        viene aggiunta a mano."""
        q = {k: float(joints.get(k, 0.0)) for k in self.JOINTS}
        p, R = self.kin.fk_link(q)
        p = p + np.array([q["base_x_joint"], q["base_y_joint"], q["base_z_joint"]])
        t_wc = self.spawn_xyz + self.R_spawn @ p
        R_wc = self.R_spawn @ R @ self.R_sensor
        return R_wc, t_wc

