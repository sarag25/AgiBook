"""
Helpers that generate the ros_gz_bridge/parameter_bridge YAML at launch time.
Static base (config/gz_bridge.yaml: clock, contacts, camera_info) plus the DetachableJoint
attach/detach/state entries of every scene entity; book_placer is the single source of
entities and topic names, so /gt_*/ entries must not be added by hand to the static file.
Kept outside the launch file so it can be tested without Gazebo:
  python3 -c "from agibot_x2_pkg.bridge_config import write_bridge_config as w; print(w('grasp_test'))"
"""

from __future__ import annotations
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from agibot_x2_pkg.book_placer import entity_topics, test_entities


def _entry(ros_topic: str, gz_topic: str, ros_type: str, gz_type: str,
           direction: str) -> dict:
    """
    Single parameter_bridge entry
    """
    return {"ros_topic_name": ros_topic, "gz_topic_name": gz_topic,
            "ros_type_name": ros_type, "gz_type_name": gz_type,
            "direction": direction}


def entity_bridge_entries(names: list[str]) -> list[dict]:
    """
    Attach (ROS->GZ), detach (ROS->GZ) and state (GZ->ROS) entries, for both hands,
    of every entity with a DetachableJoint
    """
    out = []
    for n in names:
        t = entity_topics(n)
        out.append(_entry(t["attach"], t["attach"],
                          "std_msgs/msg/Empty", "gz.msgs.Empty", "ROS_TO_GZ"))
        out.append(_entry(t["detach"], t["detach"],
                          "std_msgs/msg/Empty", "gz.msgs.Empty", "ROS_TO_GZ"))
        out.append(_entry(t["state"], t["state"],
                          "std_msgs/msg/String", "gz.msgs.StringMsg", "GZ_TO_ROS"))
        # left hand
        out.append(_entry(t["attach_left"], t["attach_left"],
                          "std_msgs/msg/Empty", "gz.msgs.Empty", "ROS_TO_GZ"))
        out.append(_entry(t["detach_left"], t["detach_left"],
                          "std_msgs/msg/Empty", "gz.msgs.Empty", "ROS_TO_GZ"))
        out.append(_entry(t["state_left"], t["state_left"],
                          "std_msgs/msg/String", "gz.msgs.StringMsg", "GZ_TO_ROS"))
    return out


def write_bridge_config(scene: str, spawn_test_books: bool = True,
                        base_path: str | None = None,
                        out_path: str | None = None) -> str:
    """
    Write /tmp/gz_bridge_<scene>.yaml and return its path
    Raises if the static base is not a valid list: a launch failing with a traceback
    is better than a bridge that dies silently.
    """
    base_path = base_path or os.path.join(
        get_package_share_directory("agibot_x2_pkg"), "config", "gz_bridge.yaml")
    with open(base_path, encoding="utf-8") as f:
        base = yaml.safe_load(f) or []
    if not isinstance(base, list):
        raise ValueError(f"{base_path}: expected a list of bridge entries, "
                         f"found {type(base).__name__}")

    names = [e[0] for e in test_entities(scene)] if spawn_test_books else []
    merged = base + entity_bridge_entries(names)

    out_path = out_path or os.path.join(tempfile.gettempdir(), f"gz_bridge_{scene}.yaml")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(f"# GENERATO da agibot_x2_pkg.bridge_config (scene={scene}, "
                f"{len(names)} entita': {names}).\n"
                f"# NON modificare: la base e' {base_path}\n")
        yaml.safe_dump(merged, f, sort_keys=False, allow_unicode=True)
    return out_path
