#!/usr/bin/env python3
"""
Scene geometry (bookshelf and table) in one place, shared by book_placer, the planning scene and the pick nodes.
Sizes are read from the URDFs Gazebo loads (urdf/bookshelf.urdf, urdf/table.urdf), the world pose uses the
same formula and inputs (SHELF_POSE, TABLE_LOCAL_XY) as rviz_gaz_control.launch.py; FALLBACK_* values are used
if the URDFs are missing, as reported by SOURCE.
The robot does not perceive shelf and table: they are a known map; photos only measure the books.
"""
import math
import os
import xml.etree.ElementTree as ET

# scene pose (launch: shelf_x, shelf_y, shelf_yaw_deg; table given in the bookshelf frame)
SHELF_POSE = (0.40, 0.0, 90.0)        # x, y [m], yaw [deg]: bookshelf 40 cm in front of the robot
# table in the bookshelf frame: lx -0.93 leaves 3 cm to the resting hand and the release 4 cm inside the top
# (farther puts the near edge at the TCP reach limit, closer hits the resting hand)
TABLE_LOCAL_XY = (-0.93, 0.80)

# fallback values, used only if the URDFs cannot be read
FALLBACK_SHELF = dict(surfaces_z=[0.022, 0.349, 0.671, 0.993], half_inner=0.378, plank_front_local=0.150,
                      back_inner_local=-0.128, back_outer_local=-0.150, half_outer=0.400, plank_t=0.022, top_z=1.350)
FALLBACK_TABLE = dict(top_size=(1.200, 0.900, 0.045), top_z=0.7275, leg=(0.04, 0.04, 0.705),
                      leg_xy=[(0.55, 0.40), (-0.55, 0.40), (0.55, -0.40), (-0.55, -0.40)])


def _urdf(name):
    """
    Path of a package URDF (installed share, then source tree), or None
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("agibot_x2_pkg"), "urdf", name)
        if os.path.exists(p):
            return p
    except Exception:
        pass
    p = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "urdf", name))
    return p if os.path.exists(p) else None


def _boxes(path):
    """
    [(size (x,y,z), origin (x,y,z))] of the <collision> boxes of a URDF
    """
    out = []
    for col in ET.parse(path).getroot().iter("collision"):
        b = col.find("geometry/box")
        if b is None:
            continue
        o = col.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        out.append((tuple(float(v) for v in b.get("size").split()), tuple(xyz)))
    return out


def _world(xl, yl, pose=SHELF_POSE):
    """
    Point (xl, yl) in the bookshelf frame -> world, with the same rotation as the launch
    """
    c, s = math.cos(math.radians(pose[2])), math.sin(math.radians(pose[2]))
    return pose[0] + c * xl - s * yl, pose[1] + s * xl + c * yl


def _load_shelf():
    """
    Bookshelf geometry from bookshelf.urdf and its source label
    """
    p = _urdf("bookshelf.urdf")
    if not p:
        return dict(FALLBACK_SHELF), "valori storici (bookshelf.urdf non trovato)"
    try:
        bx = _boxes(p)
        planks = sorted((o[2] + s[2] / 2.0, o[1] + s[1] / 2.0) for s, o in bx
                        if abs(s[2] - 0.022) < 1e-6 and s[0] > 0.5 and o[2] < 1.2)        # shelves (without the top)
        sides = [s for s, o in bx if abs(s[0] - 0.022) < 1e-6 and s[2] > 1.0 and abs(o[0]) > 0.3]
        side_x = max(abs(o[0]) for s, o in bx if abs(s[0] - 0.022) < 1e-6 and s[2] > 1.0 and abs(o[0]) > 0.3)
        back = [(s, o) for s, o in bx if s[1] == 0.022 and s[2] > 1.0]
        d = dict(surfaces_z=[round(z, 4) for z, _y in planks], half_inner=round(side_x - 0.011, 4),
                 plank_front_local=round(planks[0][1], 4), plank_t=0.022, half_outer=round(side_x + 0.011, 4),
                 back_inner_local=round(back[0][1][1] + 0.011, 4), back_outer_local=round(back[0][1][1] - 0.011, 4),
                 top_z=round(back[0][0][2], 4))
        assert len(d["surfaces_z"]) == 4 and sides
        return d, "bookshelf.urdf"
    except Exception as e:                                   # unexpected URDF layout: fall back
        return dict(FALLBACK_SHELF), f"valori storici (bookshelf.urdf non letto: {type(e).__name__})"


def _load_table():
    """
    Table geometry from table.urdf and its source label
    """
    p = _urdf("table.urdf")
    if not p:
        return dict(FALLBACK_TABLE), "valori storici (table.urdf non trovato)"
    try:
        bx = _boxes(p)
        top = max(bx, key=lambda b: b[0][0] * b[0][1])
        legs = [b for b in bx if b is not top]
        d = dict(top_size=top[0], top_z=top[1][2], leg=legs[0][0], leg_xy=[(o[0], o[1]) for _s, o in legs])
        assert len(legs) == 4
        return d, "table.urdf"
    except Exception as e:
        return dict(FALLBACK_TABLE), f"valori storici (table.urdf non letto: {type(e).__name__})"


_shelf, SHELF_SOURCE = _load_shelf()
_table, TABLE_SOURCE = _load_table()

# bookshelf (WORLD frame: yaw 90 -> front at x = 0.40 - 0.15 = 0.25)
SHELF_SURFACES_Z = list(_shelf["surfaces_z"])                   # z of the 4 shelf surfaces
SHELF_HALF_INNER_WIDTH = _shelf["half_inner"]                   # inner half width: walls at y = +-0.378
SHELF_HALF_OUTER = _shelf["half_outer"]
SHELF_PLANK_T = _shelf["plank_t"]
SHELF_TOP_Z = _shelf["top_z"]
PLANK_FRONT_X = round(_world(0.0, _shelf["plank_front_local"])[0], 4)          # front edge of the shelves (0.25)
# side panels end 11 mm beyond the shelf edge: the straight exit must clear this plane before the waist turns
SHELF_FRONT_X = round(PLANK_FRONT_X + 0.011, 4)
SHELF_BACK_PANEL_X = round(_world(0.0, _shelf["back_inner_local"])[0], 4)      # inner face of the back panel (0.528)
SHELF_X_BACK = round(_world(0.0, _shelf["back_outer_local"])[0], 4)            # back of the case (0.55)

# table (WORLD frame)
_tc = _world(*TABLE_LOCAL_XY)
TABLE_CENTER = (round(_tc[0], 4), round(_tc[1], 4))                            # top center: (-0.40, -0.93)
_ts = _table["top_size"]
_yaw = math.radians(SHELF_POSE[2])
TABLE_SIZE = (round(abs(math.cos(_yaw)) * _ts[0] + abs(math.sin(_yaw)) * _ts[1], 4),
              round(abs(math.sin(_yaw)) * _ts[0] + abs(math.cos(_yaw)) * _ts[1], 4), _ts[2])   # (0.90, 1.20, 0.045)
TABLE_TOP_CENTER_Z = _table["top_z"]
TABLE_TOP_Z = round(TABLE_TOP_CENTER_Z + _ts[2] / 2.0, 4)                      # top surface: 0.75
TABLE_LEG = _table["leg"]
TABLE_LEG_XY = [(round(math.cos(_yaw) * x - math.sin(_yaw) * y, 4), round(math.sin(_yaw) * x + math.cos(_yaw) * y, 4))
                for x, y in _table["leg_xy"]]                                  # offsets from the center, already rotated
TABLE_NEAR_EDGE_Y = round(TABLE_CENTER[1] + TABLE_SIZE[1] / 2.0, 4)            # edge near the robot: -0.33
TABLE_X_MAX = round(TABLE_CENTER[0] + TABLE_SIZE[0] / 2.0, 4)                  # edge towards the bookshelf: +0.05

TABLE_RELEASE_INSET = 0.04   # arm reach limit, not scene: at z=0.80 IK error is 0.3 mm at y -0.35, 7-10 mm at -0.38, 30 mm at -0.41
TABLE_RELEASE_Y = round(TABLE_NEAR_EDGE_Y - TABLE_RELEASE_INSET, 4)            # -0.37

SOURCE = f"scaffale: {SHELF_SOURCE}; tavolo: {TABLE_SOURCE}"

if __name__ == "__main__":
    print(SOURCE)
    for k in ("SHELF_SURFACES_Z", "SHELF_HALF_INNER_WIDTH", "PLANK_FRONT_X", "SHELF_FRONT_X", "SHELF_BACK_PANEL_X",
              "SHELF_X_BACK", "SHELF_TOP_Z", "TABLE_CENTER", "TABLE_SIZE", "TABLE_TOP_CENTER_Z", "TABLE_TOP_Z",
              "TABLE_NEAR_EDGE_Y", "TABLE_X_MAX", "TABLE_RELEASE_Y", "TABLE_LEG_XY"):
        print(f"{k:24s} {globals()[k]}")
