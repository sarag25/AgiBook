"""
Script per Blender 5.1.2 per generare l'end-effector a "morsa sottile"
(gripper a ganasce parallele) della mano sinistra del robot X2.

Esportato come STL (coerente con gli altri link del robot in meshes/, mesh
CAD senza texture — a differenza dei .glb di libreria/libri/decorazioni, che
servono texture incorporate per le foto di scena, vedi create_desk_decorations.py).

Il gripper non fa parte della scena libreria: questo script è standalone,
non viene importato da create_scene.py (che orchestra solo libreria+libri+
decorazioni da scrivania).

Due mesh esportate, tre link URDF: le due ganasce sono geometricamente
identiche, la loro posizione/orientamento come "sinistra"/"destra" è
determinato dal giunto URDF (origine + segno dell'asse), non dalla mesh.
  - gripper_base.STL    link fisso, sostituisce la mesh statica della mano
                         attuale (left_wrist_hand_link.STL) su left_wrist_yaw_link
  - gripper_finger.STL  ganascia mobile, riusata per left_gripper_left_finger_link
                         e left_gripper_right_finger_link

Assi nel frame locale di ciascun link (coerenti con l'origine del giunto
URDF che lo posiziona):
  X = asse di apertura/chiusura (ogni ganascia trasla lungo X, verso il
      centro o verso l'esterno a seconda del segno dell'asse del giunto)
  Y = larghezza di contatto della ganascia
  Z = direzione di affondo: le ganasce si protendono in -Z dalla base,
      cioè si allontanano dal polso verso l'oggetto da afferrare

Lo spessore di ogni ganascia lungo X (FINGER_THICKNESS) è volutamente
sottile per poter scivolare nel gioco tra i dorsi dei libri (spessore
libro + 6 mm di gioco usato da create_scene.py per affiancarli sul
ripiano, vedi [[Blender]]).
"""

import bpy
import os


def script_dir():
    """
    Directory of the script currently open in Blender's text editor.
    """
    return os.path.dirname(os.path.abspath(bpy.context.space_data.text.filepath))


def default_output_dir():
    """
    meshes/gripper/ directory of the ROS2 package, resolved from this
    script's location.
    """
    return os.path.normpath(
        os.path.join(script_dir(), "..", "..", "src", "agibot_x2_pkg", "meshes", "gripper")
    )


GRIPPER_MAX_OPENING = 0.08   # m, stesso valore usato in create_desk_decorations.py

BASE_SIZE = (0.05, 0.05, 0.02)     # X, Y, Z: piastra di attacco al polso
FINGER_THICKNESS = 0.006           # X: spessore ganascia (asse di chiusura)
FINGER_WIDTH = 0.05                # Y: larghezza di contatto
FINGER_LENGTH = 0.10               # Z: lunghezza della ganascia (affondo)
FINGER_MIN_GAP = 0.004             # X: distanza minima tra le ganasce a morsa chiusa

# Corsa per ganascia del giunto prismatico: da FINGER_MIN_GAP (morsa chiusa)
# a GRIPPER_MAX_OPENING (morsa tutta aperta), ripartita simmetricamente sui due lati.
FINGER_TRAVEL = (GRIPPER_MAX_OPENING - FINGER_MIN_GAP) / 2.0

assert FINGER_THICKNESS < 0.008, "Le ganasce devono restare sottili per entrare tra i libri"


def clear_scene():
    """
    Clean the scene in Blender.
    """
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def add_box_part(size, center, name):
    """
    Create a box of the given size (X, Y, Z) centered at the given point,
    bake the transform into the mesh, reset the origin, and return the object.
    """
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = size
    bpy.ops.object.transform_apply(location=True, rotation=False, scale=True)
    return obj


def build_base():
    """
    Mounting plate: replaces the static hand mesh on left_wrist_yaw_link,
    hangs down (-Z) to where the two fingers are mounted.
    """
    return add_box_part(BASE_SIZE, (0.0, 0.0, -BASE_SIZE[2] / 2.0), "gripper_base")


def build_finger():
    """
    Single finger blade, authored in its own joint frame (local origin =
    the prismatic joint's attachment point on gripper_base): thin along
    X, extends downward (-Z) past the base. Reused for both fingers.
    """
    return add_box_part(
        (FINGER_THICKNESS, FINGER_WIDTH, FINGER_LENGTH),
        (0.0, 0.0, -FINGER_LENGTH / 2.0),
        "gripper_finger",
    )


def export_stl(obj, name, export_dir):
    """
    Export the selected object as a binary STL file, matching the format
    used by the rest of the robot's link meshes (unlike the textured .glb
    props, the gripper has no material to preserve).
    """
    out_path = os.path.join(export_dir, name + ".STL")
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.wm.stl_export(
        filepath=out_path,
        export_selected_objects=True,
        ascii_format=False,
    )
    print(f"  -> {out_path}")


def main():
    """
    Clear the scene, build base + finger, and export both as STL into
    src/agibot_x2_pkg/meshes/gripper/.
    """
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    export_dir = default_output_dir()
    os.makedirs(export_dir, exist_ok=True)

    clear_scene()

    base = build_base()
    export_stl(base, "gripper_base", export_dir)

    finger = build_finger()
    export_stl(finger, "gripper_finger", export_dir)

    print(
        f"\nGripper creato: apertura max {GRIPPER_MAX_OPENING:.3f} m "
        f"(corsa per ganascia {FINGER_TRAVEL:.4f} m), "
        f"spessore ganascia {FINGER_THICKNESS * 1000:.1f} mm."
    )


if __name__ == "__main__":
    main()
