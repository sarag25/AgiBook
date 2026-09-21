"""
Helpers that turn the shelf_camera depth into the 3D geometry of the books
(world_x, world_y, thickness, z_bottom/z_top/height, free space on the sides), so a grasp needs no known pose.
Gazebo sensor frame: X optical axis, Y left, Z up; u grows towards -Y, v towards -Z; depth is along
the optical axis (not the ray). Shelf frame: local +y = open side (towards the robot), local x = lateral;
with shelf_yaw = 90 deg local x -> world +y and local +y -> world -x (the robot looks along +x).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# bookshelf.urdf: shelf_camera_joint (origin in the shelf frame)
SHELF_CAM_XYZ = (-0.16, 0.75, 1.50)
SHELF_CAM_RPY = (0.0, 0.55, -1.5707963)
SHELF_CAM_HFOV = 1.0
SHELF_CAM_SIZE = (960, 720)
# control_file.gazebo: head_camera (triggered rgbd on the head)
HEAD_CAM_HFOV = 1.0
HEAD_CAM_SIZE = (1920, 1440)
HEAD_CAM_LINK = "rgbd_head_front_link"
HEAD_SENSOR_RPY = (-1.5707963, -1.5707963, 0.0)   # sensor <pose> in the link

# bookshelf.urdf: inner geometry (shelf frame)
SHELF_HALF_INNER_WIDTH = 0.378      # inner walls at x_local = +-0.378
SHELF_BACK_INNER_Y = -0.128         # inner face of the back panel
SHELF_FRONT_Y = 0.150               # front edge of the shelves
SHELF_SURFACES_Z = (0.022, 0.349, 0.671, 0.993)   # shelf surfaces
SHELF_BOARD = 0.022
SHELF_COMPARTMENT_H = 0.300         # clearance above each surface (0.335 the top one)


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """
    Rotation matrix from URDF roll/pitch/yaw (Rz @ Ry @ Rx)
    """
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


@dataclass
class ObjectGeometry:
    """
    Measured 3D geometry of one object (world and shelf-frame coordinates, m)
    """
    world_x: float          # world x of the front face (spine)
    world_y: float          # world y of the lateral center
    z_bottom: float
    z_top: float
    thickness: float
    height: float
    depth_m: float          # median distance from the camera
    n_points: int
    length: float = 0.0     # book depth (spine -> fore edge), 0 = top face not visible
    lateral: float = 0.0    # lateral coordinate in the shelf frame (local x)
    front: float = 0.0      # "towards the robot" coordinate in the shelf frame (local y)
    lat_min: float = 0.0
    lat_max: float = 0.0
    free_plus: float = 0.0  # free space towards world +y (= +lateral with yaw 90)
    # [(band_center_z, lateral width)] bottom-up, to choose WHERE to grip a non-flat object
    width_profile: list = field(default_factory=list)
    free_minus: float = 0.0


class ShelfGeometry:
    """
    Camera model and measurements of objects in the bookshelf
    """

    def __init__(self, shelf_x: float = 0.40, shelf_y: float = 0.0, shelf_yaw: float = math.pi / 2,
                 cam_xyz=SHELF_CAM_XYZ, cam_rpy=SHELF_CAM_RPY, hfov: float = SHELF_CAM_HFOV,
                 size=SHELF_CAM_SIZE):
        """
        Shelf pose in the world and the fixed shelf_camera model (pinhole from hfov)
        """
        self.shelf_xy = np.array([shelf_x, shelf_y], dtype=float)
        self.shelf_yaw = float(shelf_yaw)
        c, s = math.cos(shelf_yaw), math.sin(shelf_yaw)
        self.R_ws = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])   # shelf -> world
        self.t_ws = np.array([shelf_x, shelf_y, 0.0])
        R_sc = rpy_matrix(*cam_rpy)                                          # camera -> shelf
        self.R_wc = self.R_ws @ R_sc                                          # camera -> world
        self.t_wc = self.t_ws + self.R_ws @ np.asarray(cam_xyz, dtype=float)
        self.W, self.H = size
        self.fx = (self.W / 2.0) / math.tan(hfov / 2.0)
        self.fy = self.fx
        self.cx, self.cy = self.W / 2.0, self.H / 2.0
        self.depth: np.ndarray | None = None
        # shelf axes in the world: lateral (local x) and front (local y)
        self.lat_axis = self.R_ws[:, 0]
        self.front_axis = self.R_ws[:, 1]

    # ─── camera model ─────────────────────────────────────────────────────

    def set_camera(self, R_wc: np.ndarray, t_wc, hfov: float, size):
        """
        Set a camera at ANY world pose, e.g. the head camera (see HeadCameraPose).
        R_wc: camera -> world (columns: optical axis, left, image up), t_wc: position
        """
        self.R_wc = np.asarray(R_wc, dtype=float).copy()
        self.t_wc = np.asarray(t_wc, dtype=float).copy()
        self.W, self.H = size
        self.fx = (self.W / 2.0) / math.tan(hfov / 2.0)
        self.fy = self.fx
        self.cx, self.cy = self.W / 2.0, self.H / 2.0
        self.depth = None

    def set_depth(self, depth: np.ndarray):
        """
        Set the HxW float32 depth in m along the optical axis (inf/nan/0 = no return)
        """
        if depth.shape != (self.H, self.W):
            # rescale intrinsics if resolution differs from nominal
            sy, sx = depth.shape[0] / self.H, depth.shape[1] / self.W
            self.fx *= sx; self.fy *= sy; self.cx *= sx; self.cy *= sy
            self.H, self.W = depth.shape
        self.depth = depth.astype(np.float32)

    def pixels_to_world(self, u: np.ndarray, v: np.ndarray, d: np.ndarray) -> np.ndarray:
        """
        (u, v, depth) -> Nx3 world points
        """
        X = d
        Y = -(u - self.cx) * d / self.fx
        Z = -(v - self.cy) * d / self.fy
        pc = np.stack([X, Y, Z], axis=-1)
        return pc @ self.R_wc.T + self.t_wc

    def world_to_pixels(self, pw: np.ndarray):
        """
        Nx3 world points -> (u, v, depth), for tests and debugging
        """
        pc = (np.asarray(pw, dtype=float) - self.t_wc) @ self.R_wc
        d = pc[:, 0]
        u = self.cx - self.fx * pc[:, 1] / d
        v = self.cy - self.fy * pc[:, 2] / d
        return u, v, d

    def world_cloud(self, bbox=None, step: int = 1):
        """
        World points (Nx3) and depths of the valid pixels in bbox (x1,y1,x2,y2) or the whole image
        """
        if self.depth is None:
            raise RuntimeError("depth not set (set_depth)")
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
        """
        Nx3 world points -> Nx3 shelf frame (lateral, front, z)
        """
        return (np.asarray(pw) - self.t_ws) @ self.R_ws

    # ─── measurements ─────────────────────────────────────────────────────

    @staticmethod
    def lateral_extent(lat: np.ndarray, cell: float = 0.002, rel_min: float = 0.08):
        """
        Lateral extent of the object: CONNECTED run of occupied `cell`-m histogram bins around the median.
        Percentiles fail because the oblique camera puts the neighbour's cover inside the bbox;
        the gap between the two is a run of empty bins
        """
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
        """
        Geometry of the object in bbox (x1,y1,x2,y2 px), None if too few points.
        shrink_v = bbox fraction cut at top and bottom (background/shelf); laterally
        everything is kept and lateral_extent does the filtering
        """
        x1, y1, x2, y2 = bbox
        h = y2 - y1
        inner = (x1 + 1, y1 + shrink_v * h, x2 - 1, y2 - shrink_v * h)
        pw, d = self.world_cloud(inner)
        loc = self.shelf_local(pw) if len(pw) else np.zeros((0, 3))
        # only points inside the shelf: the robot head in front of it can fall in the bbox
        keep = self.inside_shelf(loc)
        pw, d, loc = pw[keep], d[keep], loc[keep]
        if len(pw) < 30:
            return None
        lat, front, z = loc[:, 0], loc[:, 1], loc[:, 2]
        # front face = points closest to the robot (max front)
        front_face = np.percentile(front, 90)
        face = front >= front_face - face_depth
        lat_min, lat_max = self.lateral_extent(lat[face])
        # height: only front-face points within the lateral extent (a taller neighbour's cover inflates it), full bbox height
        pw_full, _ = self.world_cloud((x1 + 1, y1, x2 - 1, y2))
        loc_f = self.shelf_local(pw_full) if len(pw_full) else loc
        loc_f = loc_f[self.inside_shelf(loc_f)] if len(loc_f) else loc
        own = (loc_f[:, 1] >= front_face - face_depth) & (loc_f[:, 0] >= lat_min - 0.002) & (loc_f[:, 0] <= lat_max + 0.002)
        z_all = loc_f[own, 2] if own.sum() >= 20 else z
        z_bottom, z_top = float(np.percentile(z_all, 1)), float(np.percentile(z_all, 99))
        # length (spine -> fore edge) from the top face, visible only if no shelf covers it; < 3 cm = unknown (0)
        own_lat = (loc_f[:, 0] >= lat_min + 0.002) & (loc_f[:, 0] <= lat_max - 0.002) \
            & (loc_f[:, 2] >= z_top - 0.02)
        length = 0.0
        if own_lat.sum() >= 20:
            ext = float(front_face - np.percentile(loc_f[own_lat, 1], 2))
            if ext >= 0.03:
                length = ext
        # width profile per 1 cm band (same points as for the height)
        width_profile = []
        if own.sum() >= 20:
            pts_o = loc_f[own]
            band = 0.01
            z0 = z_bottom
            while z0 < z_top - 1e-6:
                sel = (pts_o[:, 2] >= z0) & (pts_o[:, 2] < z0 + band)
                if sel.sum() >= 5:
                    lo_, hi_ = np.percentile(pts_o[sel, 0], [2, 98])
                    width_profile.append((round(float(z0 + band / 2), 4), round(float(hi_ - lo_), 4)))
                z0 += band
        lateral = float((lat_min + lat_max) / 2.0)
        front_c = float(front_face)
        # front face and lateral center in the world
        p_face = self.t_ws + self.R_ws @ np.array([lateral, front_c, (z_bottom + z_top) / 2])
        return ObjectGeometry(
            world_x=float(p_face[0]), world_y=float(p_face[1]),
            z_bottom=z_bottom, z_top=z_top,
            thickness=float(lat_max - lat_min), height=float(z_top - z_bottom),
            depth_m=float(np.median(d)), n_points=int(len(pw)), length=length,
            lateral=lateral, front=front_c, lat_min=float(lat_min), lat_max=float(lat_max),
            width_profile=width_profile)

    def free_space(self, geoms: list, row_tol: float = 0.06):
        """
        Fill free_plus/free_minus (towards +/- lateral, = +/- world y with yaw 90)
        from the neighbours on the same shelf and the inner walls
        """
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
    def inside_shelf(loc: np.ndarray, wall_margin: float = 0.006, floor_margin: float = 0.004) -> np.ndarray:
        """
        Mask of the shelf-frame points inside a compartment: between the walls, in front
        of the back panel, above a surface and below the next board
        """
        if len(loc) == 0:
            return np.zeros(0, dtype=bool)
        lat, front, z = loc[:, 0], loc[:, 1], loc[:, 2]
        inside = (np.abs(lat) < SHELF_HALF_INNER_WIDTH - wall_margin) \
            & (front > SHELF_BACK_INNER_Y + 0.02) & (front < SHELF_FRONT_Y + 0.06)
        above = np.zeros_like(inside)
        for s in SHELF_SURFACES_Z:
            above |= (z > s + floor_margin) & (z < s + SHELF_COMPARTMENT_H - 0.01)
        return inside & above

    @staticmethod
    def surface_below(z: float):
        """
        Highest shelf surface at or below z (5 mm tolerance), None if none
        """
        below = [s for s in SHELF_SURFACES_Z if s <= z + 0.005]
        return max(below) if below else None


class HeadCameraPose:
    """
    World pose of the head camera from the joints (exact in simulation, no calibration).
    Chain world -> base_x/y/z/yaw -> waist -> head -> rgbd_head_front_link from the URDF,
    plus the robot SPAWN pose (x = ROBOT_SPAWN_X - walk_distance of the launch, y 0, yaw 0).
    Same convention as ShelfGeometry (X optical, Y left, Z up)
    """

    JOINTS = ("base_x_joint", "base_y_joint", "base_z_joint", "base_yaw_joint",
              "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
              "head_yaw_joint", "head_pitch_joint")

    def __init__(self, spawn_xyz=(0.0, 0.0, 0.0), spawn_yaw: float = 0.0, urdf_path: str | None = None):
        """
        Build the FK chain to HEAD_CAM_LINK and store the spawn pose
        """
        from agibot_x2_pkg_py.arm_kinematics import ArmKinematics   # sibling package, imported at runtime
        self.kin = (ArmKinematics.from_urdf(urdf_path, tip=HEAD_CAM_LINK) if urdf_path
                    else ArmKinematics.from_package(tip=HEAD_CAM_LINK))
        self.spawn_xyz = np.asarray(spawn_xyz, dtype=float)
        self.R_spawn = rpy_matrix(0.0, 0.0, float(spawn_yaw))
        self.R_sensor = rpy_matrix(*HEAD_SENSOR_RPY)

    def world_pose(self, joints: dict):
        """
        (R_wc, t_wc) from joints {name: value} (from /joint_states, missing = 0).
        fk_link ignores the prismatic base joints, so their translation (before base_yaw) is added here
        """
        q = {k: float(joints.get(k, 0.0)) for k in self.JOINTS}
        p, R = self.kin.fk_link(q)
        p = p + np.array([q["base_x_joint"], q["base_y_joint"], q["base_z_joint"]])
        t_wc = self.spawn_xyz + self.R_spawn @ p
        R_wc = self.R_spawn @ R @ self.R_sensor
        return R_wc, t_wc

