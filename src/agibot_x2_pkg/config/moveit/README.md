# MoveIt 2 per l'AgiBot X2 (2026-09-18)

File letti da `launch/moveit.launch.py` (MoveItConfigsBuilder, pacchetto `agibot_x2_pkg`):

| File | Cosa |
|---|---|
| `x2.srdf` | gruppi (`right_arm`/`left_arm` = vita yaw+pitch + 5 giunti braccio + giunto fisso della pinza; `*_arm_only`; `waist`; `*_gripper`; `head`), pose `home`/`carry`, end effector, giunti passivi (base cinematica, gambe, waist_roll) e le coppie di link senza collisione (GENERATE) |
| `kinematics.yaml` | KDL sui soli bracci: richiesto dal builder, non usato (i goal sono a giunti) |
| `joint_limits.yaml` | velocita'/accelerazioni per la parametrizzazione temporale (piu' basse dell'URDF) |
| `moveit_controllers.yaml` | collega move_group ai controller ros2_control gia' attivi (nessun controller nuovo) |

## Rigenerare le coppie di collisione dell'SRDF

Dopo una modifica all'URDF (nuovi link, collisioni cambiate):

```bash
source install/setup.bash
xacro src/agibot_x2_pkg/urdf/x2_hand_gazebo.urdf finger_mu:=1.0 > /tmp/x2.urdf
ros2 run moveit_setup_assistant collisions_updater --urdf /tmp/x2.urdf \
    --srdf src/agibot_x2_pkg/config/moveit/x2.srdf --output /tmp/x2_out.srdf \
    --default --always --trials 20000
# controllare che i gruppi siano rimasti (il tool riscrive tutto il file), poi:
cp /tmp/x2_out.srdf src/agibot_x2_pkg/config/moveit/x2.srdf
```

Nota: il tool toglie i commenti e riordina; i gruppi vanno ricontrollati (in
particolare `<joint name="*_gripper_mount_joint"/>` nei gruppi braccio, che
serve all'end effector).

## Uso

```bash
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py     # simulazione + controller
ros2 launch agibot_x2_pkg moveit.launch.py               # move_group + planning_scene_builder
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p target:=6 ...   # usa MoveIt (moveit:=false per spegnerlo)
```

`planning_scene_builder` (agibot_x2_pkg_py) riempie la scena da
`/tmp/x2_detections.json` (libreria, tavolo, oggetti misurati) e gestisce
l'oggetto in mano (`/scene_builder/attach|detach|remove`).
