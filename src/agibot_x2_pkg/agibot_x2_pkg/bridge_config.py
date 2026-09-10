"""
Genera il file YAML per ros_gz_bridge/parameter_bridge al launch:
base STATICA (config/gz_bridge.yaml: clock, contatti, camera_info) + le voci
attach/detach/state del DetachableJoint per OGNI entita' della scena
(book_placer.test_entities). Nessuno deve piu' aggiungere a mano le voci
/gt_*/... al file statico (2026-09-06): la fonte unica delle entita' e'
book_placer, e i nomi dei topic vengono da book_placer.entity_topics, la
stessa funzione usata dal launch per il blocco <plugin> del libro.

Vive in un modulo a parte (e non dentro il launch) per essere provabile
senza Gazebo:
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
    return {"ros_topic_name": ros_topic, "gz_topic_name": gz_topic,
            "ros_type_name": ros_type, "gz_type_name": gz_type,
            "direction": direction}


def entity_bridge_entries(names: list[str]) -> list[dict]:
    """Voci attach (ROS->GZ), detach (ROS->GZ), state (GZ->ROS) per ogni
    entita' con DetachableJoint."""
    out = []
    for n in names:
        t = entity_topics(n)
        out.append(_entry(t["attach"], t["attach"],
                          "std_msgs/msg/Empty", "gz.msgs.Empty", "ROS_TO_GZ"))
        out.append(_entry(t["detach"], t["detach"],
                          "std_msgs/msg/Empty", "gz.msgs.Empty", "ROS_TO_GZ"))
        out.append(_entry(t["state"], t["state"],
                          "std_msgs/msg/String", "gz.msgs.StringMsg", "GZ_TO_ROS"))
    return out


def write_bridge_config(scene: str, spawn_test_books: bool = True,
                        base_path: str | None = None,
                        out_path: str | None = None) -> str:
    """Scrive /tmp/gz_bridge_<scene>.yaml e ne ritorna il percorso.
    Solleva se la base statica non e' YAML valido: meglio un launch che
    fallisce con traceback di un bridge morto in silenzio."""
    base_path = base_path or os.path.join(
        get_package_share_directory("agibot_x2_pkg"), "config", "gz_bridge.yaml")
    with open(base_path, encoding="utf-8") as f:
        base = yaml.safe_load(f) or []
    if not isinstance(base, list):
        raise ValueError(f"{base_path}: attesa una lista di voci bridge, "
                         f"trovato {type(base).__name__}")

    names = [e[0] for e in test_entities(scene)] if spawn_test_books else []
    merged = base + entity_bridge_entries(names)

    out_path = out_path or os.path.join(tempfile.gettempdir(), f"gz_bridge_{scene}.yaml")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(f"# GENERATO da agibot_x2_pkg.bridge_config (scene={scene}, "
                f"{len(names)} entita': {names}).\n"
                f"# NON modificare: la base e' {base_path}\n")
        yaml.safe_dump(merged, f, sort_keys=False, allow_unicode=True)
    return out_path
