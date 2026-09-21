"""
Tests for scene_config: values derived from the URDFs must match the reference values and the launch file.
"""
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "agibot_x2_pkg"))


def test_valori_derivati_uguali_a_quelli_storici():
    """
    Values derived from bookshelf.urdf and table.urdf match the reference numbers
    """
    from agibot_x2_pkg import scene_config as c
    assert c.SHELF_SOURCE == "bookshelf.urdf" and c.TABLE_SOURCE == "table.urdf", c.SOURCE
    assert c.SHELF_SURFACES_Z == [0.022, 0.349, 0.671, 0.993]
    assert c.SHELF_HALF_INNER_WIDTH == 0.378 and c.PLANK_FRONT_X == 0.25 and c.SHELF_FRONT_X == 0.261
    assert c.SHELF_BACK_PANEL_X == 0.528 and c.SHELF_X_BACK == 0.55 and c.SHELF_TOP_Z == 1.35
    assert c.TABLE_CENTER == (-0.4, -0.93) and c.TABLE_SIZE == (0.9, 1.2, 0.045)
    assert c.TABLE_TOP_Z == 0.75 and c.TABLE_TOP_CENTER_Z == 0.7275
    assert c.TABLE_NEAR_EDGE_Y == -0.33 and c.TABLE_X_MAX == 0.05 and c.TABLE_RELEASE_Y == -0.37
    assert sorted(c.TABLE_LEG_XY) == sorted([(0.4, 0.55), (-0.4, 0.55), (0.4, -0.55), (-0.4, -0.55)])


def test_il_launch_usa_gli_stessi_numeri():
    """
    The launch file imports the table from scene_config and defaults to the same shelf pose
    """
    from agibot_x2_pkg import scene_config as c
    src = open(os.path.join(ROOT, "agibot_x2_pkg", "launch", "rviz_gaz_control.launch.py"), encoding="utf-8").read()
    assert "from agibot_x2_pkg.scene_config import TABLE_LOCAL_XY" in src
    x = re.search(r"'shelf_x',\s*default_value='([-0-9.]+)'", src)
    yaw = re.search(r"'shelf_yaw_deg',\s*default_value='([-0-9.]+)'", src)
    assert x and yaw, "argomenti del launch non trovati"
    assert float(x.group(1)) == c.SHELF_POSE[0] and float(yaw.group(1)) == c.SHELF_POSE[2], (x.group(1), yaw.group(1))


def test_book_placer_e_planning_scene_leggono_da_scene_config():
    """
    book_placer reads its shelf geometry from scene_config
    """
    from agibot_x2_pkg import book_placer, scene_config
    assert book_placer.SHELF_SURFACES_Z is scene_config.SHELF_SURFACES_Z
    assert book_placer.SHELF_HALF_INNER_WIDTH == scene_config.SHELF_HALF_INNER_WIDTH
