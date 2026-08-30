"""
Converte un SortPlan in una sequenza di azioni ROS2 per il robot X2.

Ogni azione è un messaggio FollowJointTrajectory verso:
  - left_arm_controller     (braccio principale pick-and-place)
  - left_gripper_controller (ganasce parallele mano sinistra, vedi Blender.md)
  - right_arm_controller    (ausiliario per libri pesanti)
  - waist_controller        (rotazione verso carrello)
  - head_controller         (tracking visivo)

La sequenza per ogni libro è:
  LOOK → PRE_GRASP → REACH → GRASP → LIFT → TRANSPORT → PLACE → OPEN → HOME
"""

from __future__ import annotations
import logging
import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from scripts.vision.book_detector import DetectedObject
    from scripts.sorting.sort_planner import SortPlan

log = logging.getLogger(__name__)


@dataclass
class RobotAction:
    action_type: str          # MOVE_ARM | GRASP | PLACE | ROTATE_WAIST | MOVE_HEAD
    target_obj_id: int        # -1 = nessun oggetto specifico
    description: str
    joint_names: list[str] = field(default_factory=list)
    joint_positions: list[float] = field(default_factory=list)
    duration_sec: float = 2.0
    # Per azioni con traiettoria multi-punto
    waypoints: list[dict] = field(default_factory=list)
    # True solo sul PLACE che deposita sul tavolo di staging (non sul PLACE
    # finale sullo scaffale) - library_manager_node._execute_actions() lo usa
    # per scattare/ri-analizzare col table_camera subito dopo, vedi
    # _rephotograph_on_table() e Gazebo.md § "Camere".
    on_staging_table: bool = False


# Posizioni articolari di riferimento (radianti, calibrate sul modello x2_hand)
JOINT_CONFIGS = {
    "home": {
        "left_arm": [0.0, 0.1, 0.0, 0.0, 0.0],
        "head":     [0.0, 0.0],
        "waist":    [0.0, 0.0],
    },
    # Braccio verso scaffale ripiano basso (h ~0.5m)
    "reach_shelf_low": {
        "left_arm": [0.7, 0.5, 0.0, -1.2, 0.3],
    },
    # Braccio verso scaffale ripiano medio (h ~1.0m)
    "reach_shelf_mid": {
        "left_arm": [0.4, 0.4, 0.0, -0.9, 0.2],
    },
    # Braccio verso scaffale ripiano alto (h ~1.5m)
    "reach_shelf_high": {
        "left_arm": [0.1, 0.35, 0.0, -0.6, 0.1],
    },
    # Posizione di trasporto (libro sollevato, sicuro)
    "carry": {
        "left_arm": [0.3, 0.2, 0.0, -0.5, 0.5],
    },
    # Deposita sul carrello (waist ruotato 90° a destra)
    "deposit_cart": {
        "waist":    [1.57, 0.0],
        "left_arm": [0.4,  0.3, 0.0, -0.7, 0.5],
    },
}

LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
]
LEFT_GRIPPER_JOINTS = ["left_gripper_left_finger_joint", "left_gripper_right_finger_joint"]
HEAD_JOINTS  = ["head_yaw_joint", "head_pitch_joint"]
WAIST_JOINTS = ["waist_yaw_joint", "waist_pitch_joint"]

# Posizioni (m) dei giunti prismatici del gripper: 0 = ganasce a battuta
# (FINGER_MIN_GAP), upper = tutta aperta (~GRIPPER_MAX_OPENING). Vedi
# environment/gripper/create_gripper.py.
GRIPPER_OPEN = [0.037, 0.037]
GRIPPER_CLOSED = [0.013, 0.013]  # chiusura su un dorso di libro medio (~30 mm)


def _shelf_row_to_config(shelf_row: int) -> str:
    """Mappa il numero di ripiano alla configurazione di reach."""
    if shelf_row == 0:
        return "reach_shelf_low"
    elif shelf_row == 1:
        return "reach_shelf_mid"
    else:
        return "reach_shelf_high"


def _head_pitch_for_row(shelf_row: int) -> float:
    """Angolo testa per guardare al ripiano corretto."""
    pitches = {0: -0.4, 1: -0.15, 2: 0.1, 3: 0.3}
    return pitches.get(shelf_row, -0.2)


class ActionSequencer:
    """
    Genera la lista completa di RobotAction da eseguire per riordinare la libreria.

    Uso:
        sequencer = ActionSequencer()
        actions = sequencer.generate(sort_plan)
        # Poi passa actions al nodo ROS2
    """

    def generate(self, plan: "SortPlan") -> list[RobotAction]:
        actions: list[RobotAction] = []

        # FASE 1: Sposta ostacoli sul carrello
        for obj in plan.obstacles_to_move:
            actions += self._obstacle_sequence(obj)

        # FASE 2: Rimuovi libri dallo scaffale (nel removal_order)
        staging_slot = 0  # slot temporaneo sul tavolo di staging
        for book in plan.removal_order:
            actions += self._pick_from_shelf(book, staging_slot)
            staging_slot += 1

        # FASE 3: Reinserisci libri nell'ordine target
        for target_slot, book in enumerate(plan.insertion_order):
            actions += self._place_on_shelf(book, target_slot)

        # HOME finale
        actions.append(RobotAction(
            action_type="MOVE_ARM",
            target_obj_id=-1,
            description="Torna alla posizione HOME",
            joint_names=LEFT_ARM_JOINTS,
            joint_positions=JOINT_CONFIGS["home"]["left_arm"],
            duration_sec=2.5,
        ))

        log.info(f"Sequenza generata: {len(actions)} azioni totali")
        return actions

    # ─── Sequenze per ostacolo ────────────────────────────────────────────

    def _obstacle_sequence(self, obj: "DetectedObject") -> list[RobotAction]:
        """Prendi l'ostacolo e portalo sul carrello."""
        row_cfg = _shelf_row_to_config(max(0, obj.shelf_row))
        return [
            RobotAction(
                action_type="MOVE_HEAD",
                target_obj_id=obj.obj_id,
                description=f"Guarda l'ostacolo [{obj.obj_id}] {obj.class_name}",
                joint_names=HEAD_JOINTS,
                joint_positions=[0.0, _head_pitch_for_row(obj.shelf_row)],
                duration_sec=1.0,
            ),
            RobotAction(
                action_type="MOVE_ARM",
                target_obj_id=obj.obj_id,
                description=f"Pre-presa ostacolo [{obj.obj_id}]",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=JOINT_CONFIGS[row_cfg]["left_arm"],
                duration_sec=2.0,
            ),
            _gripper_action(obj.obj_id, f"Presa ostacolo [{obj.obj_id}]", closed=True),
            RobotAction(
                action_type="ROTATE_WAIST",
                target_obj_id=obj.obj_id,
                description="Ruota verso carrello",
                joint_names=WAIST_JOINTS,
                joint_positions=JOINT_CONFIGS["deposit_cart"]["waist"],
                duration_sec=2.0,
            ),
            RobotAction(
                action_type="PLACE",
                target_obj_id=obj.obj_id,
                description=f"Deposita ostacolo sul carrello",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=JOINT_CONFIGS["deposit_cart"]["left_arm"],
                duration_sec=1.5,
            ),
            _gripper_action(obj.obj_id, "Rilascia ostacolo", closed=False, duration_sec=0.8),
            RobotAction(
                action_type="ROTATE_WAIST",
                target_obj_id=-1,
                description="Ritorna fronte scaffale",
                joint_names=WAIST_JOINTS,
                joint_positions=JOINT_CONFIGS["home"]["waist"],
                duration_sec=2.0,
            ),
        ]

    # ─── Pick da scaffale ─────────────────────────────────────────────────

    def _pick_from_shelf(self, book: "DetectedObject",
                         staging_slot: int) -> list[RobotAction]:
        """Rimuovi un libro dallo scaffale e mettilo sul tavolo di staging."""
        row_cfg = _shelf_row_to_config(book.shelf_row)
        rotate_action = self._maybe_rotate_book(book)
        actions = [
            RobotAction(
                action_type="MOVE_HEAD",
                target_obj_id=book.obj_id,
                description=f"Osservo libro [{book.obj_id}] '{book.title or book.color_name}'",
                joint_names=HEAD_JOINTS,
                joint_positions=[0.0, _head_pitch_for_row(book.shelf_row)],
                duration_sec=1.0,
            ),
            RobotAction(
                action_type="MOVE_ARM",
                target_obj_id=book.obj_id,
                description=f"Pre-presa libro [{book.obj_id}]",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=JOINT_CONFIGS[row_cfg]["left_arm"],
                duration_sec=2.5,
            ),
            _gripper_action(book.obj_id, f"Prendo libro [{book.obj_id}]", closed=True),
        ]
        if rotate_action:
            actions.append(rotate_action)
        actions += [
            RobotAction(
                action_type="MOVE_ARM",
                target_obj_id=book.obj_id,
                description=f"Sollevo libro [{book.obj_id}]",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=JOINT_CONFIGS["carry"]["left_arm"],
                duration_sec=2.0,
            ),
            RobotAction(
                action_type="ROTATE_WAIST",
                target_obj_id=book.obj_id,
                description="Ruoto verso tavolo staging",
                joint_names=WAIST_JOINTS,
                joint_positions=[math.pi, 0.0],  # 180° → tavolo dietro
                duration_sec=2.5,
            ),
            RobotAction(
                action_type="PLACE",
                target_obj_id=book.obj_id,
                description=f"Deposito libro sul tavolo (slot {staging_slot})",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=_staging_position(staging_slot),
                duration_sec=1.5,
            ),
            _gripper_action(book.obj_id, "Rilascio libro sul tavolo", closed=False, duration_sec=0.8),
            RobotAction(
                # target_obj_id=book.obj_id (non -1 come le altre ROTATE_WAIST
                # di ritorno): serve a _execute_actions() per sapere QUALE
                # libro ri-fotografare col table_camera una volta che braccio/
                # gripper si sono tolti di mezzo dall'inquadratura tornando
                # verso lo scaffale - vedi on_staging_table sopra.
                action_type="ROTATE_WAIST",
                target_obj_id=book.obj_id,
                description="Ritorno fronte scaffale",
                joint_names=WAIST_JOINTS,
                joint_positions=JOINT_CONFIGS["home"]["waist"],
                duration_sec=2.0,
                on_staging_table=True,
            ),
        ]
        return actions

    # ─── Place sullo scaffale ─────────────────────────────────────────────

    def _place_on_shelf(self, book: "DetectedObject",
                        target_slot: int) -> list[RobotAction]:
        """Prendi il libro dal tavolo staging e posizionalo nel target_slot."""
        target_row = target_slot // 8   # assumiamo max 8 libri per ripiano
        row_cfg = _shelf_row_to_config(target_row)
        src_slot = list.index if hasattr(list, "index") else 0  # slot staging

        return [
            RobotAction(
                action_type="ROTATE_WAIST",
                target_obj_id=book.obj_id,
                description=f"Giro verso staging per libro [{book.obj_id}]",
                joint_names=WAIST_JOINTS,
                joint_positions=[math.pi, 0.0],
                duration_sec=2.0,
            ),
            RobotAction(
                action_type="MOVE_ARM",
                target_obj_id=book.obj_id,
                description=f"Prendo libro [{book.obj_id}] dallo staging",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=_staging_position(target_slot),
                duration_sec=2.0,
            ),
            _gripper_action(book.obj_id, f"Presa libro [{book.obj_id}]", closed=True),
            RobotAction(
                action_type="MOVE_ARM",
                target_obj_id=book.obj_id,
                description="Sollevo per trasporto",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=JOINT_CONFIGS["carry"]["left_arm"],
                duration_sec=1.5,
            ),
            RobotAction(
                action_type="ROTATE_WAIST",
                target_obj_id=-1,
                description="Giro verso scaffale",
                joint_names=WAIST_JOINTS,
                joint_positions=JOINT_CONFIGS["home"]["waist"],
                duration_sec=2.0,
            ),
            RobotAction(
                action_type="MOVE_HEAD",
                target_obj_id=-1,
                description=f"Guardo slot target (slot {target_slot})",
                joint_names=HEAD_JOINTS,
                joint_positions=[0.0, _head_pitch_for_row(target_row)],
                duration_sec=1.0,
            ),
            RobotAction(
                action_type="PLACE",
                target_obj_id=book.obj_id,
                description=f"Inserisco libro [{book.obj_id}] slot {target_slot}",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=JOINT_CONFIGS[row_cfg]["left_arm"],
                duration_sec=2.5,
            ),
            _gripper_action(book.obj_id, "Rilascio libro sullo scaffale", closed=False, duration_sec=0.8),
            RobotAction(
                action_type="MOVE_ARM",
                target_obj_id=-1,
                description="Ritiro braccio dallo scaffale",
                joint_names=LEFT_ARM_JOINTS,
                joint_positions=JOINT_CONFIGS["home"]["left_arm"],
                duration_sec=1.5,
            ),
        ]

    def _maybe_rotate_book(self, book: "DetectedObject") -> "RobotAction | None":
        """
        Se il libro non è upright, genera un'azione di rotazione polso
        (il gripper resta chiuso sul libro: qui si muove il braccio, non
        le ganasce, vedi LEFT_GRIPPER_JOINTS per l'apertura/chiusura).
        """
        if book.orientation in ("upright", "unknown"):
            return None

        wrist_angle = {
            "sideways_right": -1.57,
            "sideways_left":   1.57,
            "inverted":        3.14,
        }.get(book.orientation, 0.0)

        return RobotAction(
            action_type="MOVE_ARM",
            target_obj_id=book.obj_id,
            description=f"Ruoto libro [{book.obj_id}] da '{book.orientation}' a upright",
            joint_names=LEFT_ARM_JOINTS,
            joint_positions=[0.5, 0.4, 0.0, -1.0, wrist_angle],
            duration_sec=1.5,
        )


# ─── Helpers ─────────────────────────────────────────────────────────────────

def _gripper_action(target_obj_id: int, description: str, closed: bool,
                     duration_sec: float = 1.0) -> RobotAction:
    """Apri/chiudi le ganasce del gripper sinistro (LEFT_GRIPPER_JOINTS)."""
    return RobotAction(
        action_type="GRASP",
        target_obj_id=target_obj_id,
        description=description,
        joint_names=LEFT_GRIPPER_JOINTS,
        joint_positions=GRIPPER_CLOSED if closed else GRIPPER_OPEN,
        duration_sec=duration_sec,
    )


def _staging_position(slot: int) -> list[float]:
    """
    Posizione braccio per slot sul tavolo di staging.
    Ogni slot è spostato lateralmente di ~0.1 rad.
    """
    lateral_offset = -0.3 + slot * 0.12
    return [0.3, max(-0.5, min(0.8, lateral_offset)), 0.0, -0.4, 0.5]
