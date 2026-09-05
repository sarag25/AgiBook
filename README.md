# SmartRobotics

## Blender

### Create the books with texture (cover, retro and spine)

In Blender (version 5.1.2), select the **Scripting** tab then click on the :open_file_folder: to load the script
 [create_books.py](books/create_books.py). It'll appear on the text editor. Click the :arrow_forward: to run the script. The books in the `.glb` format will be saved in the *src/agibot_x2_pkg/meshes/books/* folder.

> [!TIP]
> :eyes: If you want to see the result, select the **Texture Paint** tab.

> [!NOTE]
> :link: The books' texture images used in the script are downloadable here [`books_textures.zip`](https://drive.google.com/file/d/12ZqwQXHbO_CrZTKkXI9Myd3cvk0yf5Lq/view?usp=sharing). You need to extract the content of the `.zip` folder in the *books* one (the same of the script [create_books.py](books/create_books.py)).

## Docker

###### image : ros2_gui [ TAG: v0.1 ] 
###### container : ros2_project

#### CREATE CONTAINER
Inside folder ROS2_ContainerGUI containing bash scripts, open in terminal:
```
wsl
./create_container.sh ros2_gui:v0.1 <optional_work_space_name> ros2_project
```

## Workspace
#### CREATE NEW WORKSPACE
Inside container terminal *> robot@docker-desktop:~$*:
```
mkdir ws_name  # if not created already
cd <ws_name>
```
In terminal *> robot@docker-desktop:~/**<ws_name>**$*
```
mkdir src
```
Modify CMake if needed

Creation of robot ```.urdf``` ref file, placed in ```urdf/``` dir

#### ENABLE ROS2 COMMANDS IN WS DIRECTORY:
In terminal *> robot@docker-desktop:~/**<ws_name>**$*
```
source /opt/ros/jazzy/setup.bash
```

#### MAKE FILES RECOGNIZABLE IN WS/SRC FOLDER:
In terminal *> robot@docker-desktop:~/**<ws_name>**$*
```
source install/setup.bash
```

#### COMPILE WS FOLDER STRUCTURE:
In terminal *> robot@docker-desktop:~/**<ws_name>**$*

```
colcon build
```
Redo at any change in workspace files


#### IF COPY and PASTE WS FOLDER (colcon build error):
In terminal *> robot@docker-desktop:~/**<ws_name>**$*

```
rm -rf build/ install/ log/
```

#### INSTALL/UPGRADE ROS2 PACKAGES
```
sudo apt update
sudo apt install ros-jazzy-controller-manager ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-rclcpp
sudo apt update && sudo apt upgrade -y
```

#### DUE PC SULLA STESSA RETE: ISOLA IL GRAFO ROS 2
I container usano `--network=host`: due PC in LAN che simulano insieme si **vedono via DDS** (spawner con `Controller already loaded`, RViz che mescola i `/tf` di due robot e mostra il modello "piegato", comandi attach/detach che finiscono sull'altra macchina). I file di `.devcontainer` impostano già `ROS_LOCALHOST_ONLY=1`; in un container **creato prima del 2026-08-30** aggiungilo a mano:
```
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc && source ~/.bashrc
```
(va fatto in ogni terminale usato per lanciare nodi — o riapri i terminali).

#### SE MANCA `topic_tools` (o `ros_gz_image`) AL LANCIO
Errore tipico: `package 'topic_tools' not found` lanciando `rviz_gaz_control.launch.py` (servono per il bridge delle camere: relay dei `camera_info` e bridge immagini).
```
sudo apt update
sudo apt install ros-jazzy-topic-tools ros-jazzy-ros-gz-image
```
Sono anche nel `.devcontainer/DockerFile`, quindi un container ricostruito dall'immagine li ha già; un container **vivo** creato prima del 2026-08-29 va aggiornato a mano con il comando sopra.

#### LAUNCH RVIZ + GAZEBO:
In terminal > robot@docker-desktop:~/<ws_name>$
```
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py
```
Senza argomenti parte l'**ambiente della pipeline definitiva** (`scene:=grasp_test`, vedi sezione sotto): libreria + tavolo + 4 libri e 2 oggetti fisici sul primo scaffale.

#### AMBIENTE (scene): COME SCEGLIERE L'ENVIRONMENT
L'ambiente si sceglie con gli argomenti del launch (`ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py --show-args` li elenca tutti). I file coinvolti sono in `src/agibot_x2_pkg/`:

| Argomento | Default | Valori / file presenti | Cosa cambia |
|---|---|---|---|
| `scene` | `grasp_test` | `grasp_test` → `urdf/bookshelf.urdf` + `urdf/table.urdf` + 6 entità fisiche (`book_placer.GRASP_TEST_ENTITIES`) · `full` → `urdf/full_scene.urdf` (mesh unica `meshes/full_scene.glb`: libreria piena di libri dipinti + decorazioni + tavolo) + 2 libri fisici `picktest_it`/`picktest_emma` (`book_placer.TEST_BOOKS`) | Quale scena viene spawnata davanti al robot |
| `spawn_test_books` | `true` | `true` / `false` | `false` = nessuna entità fisica: con `grasp_test` restano solo libreria e tavolo vuoti, con `full` solo la scena dipinta |
| `world` | `empty.world` | `worlds/empty.world` (vuoto, fisica+sensori) · `worlds/bookshelf.world` (vecchio placeholder di libreria/tavolo dentro l'SDF: **non** usarlo insieme alle scene sopra, si duplicano) | Il mondo Gazebo di base |
| `model` | `x2_hand_gazebo.urdf` | file in `urdf/` (`x2_hand_gazebo.urdf` = robot con gripper e camere; `OLD_WORKING_x2_hand_gazebo.urdf` = versione precedente) | Il robot |
| `shelf_x` / `shelf_y` / `shelf_yaw_deg` | `0.40` / `0.0` / `90.0` | metri / gradi | Posa della libreria (il tavolo la segue) |
| `x` / `y` / `z` / `yaw` | `-0.10` / `0` / `0.662` / `0` | metri / rad | Posa di spawn del robot |

Esempi:
```
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py                       # pipeline: 4 libri + 2 oggetti (default)
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py scene:=full           # scena completa full_scene.urdf (il file è rimasto nel repo)
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py spawn_test_books:=false   # solo libreria e tavolo vuoti
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py scene:=full world:=bookshelf.world spawn_test_books:=false   # vecchio comportamento
```
Gli script che devono conoscere le entità (`pick_place_teleop`, `pick_test_book`) hanno lo stesso parametro con lo stesso default: passa `-p scene:=full` solo se hai lanciato la scena completa.

> [!NOTE]
> Se il launch risponde `launch configuration 'scene' does not exist` o non trova i libri nuovi, l'install è vecchio: `colcon build` + `source install/setup.bash` nello stesso terminale (o `smartbuild`).

#### Lista controller attivi durante la simulazione per vedere se va tutto, in altro terminale::
In new terminal *> robot@docker-desktop:~/**<ws_name>**$*
```
ros2 control list_controllers
```
###### solved joint_state_broadcaster 5s timeout error: https://github.com/ros-controls/gz_ros2_control/issues/421

#### Activate Gazebo GUI controllers
In new terminal *> robot@docker-desktop:~/**<ws_name>**$*
```
source /opt/ros/jazzy/setup.bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
If pkg not found:
```
sudo apt update
sudo apt install ros-<ros-distro>-rqt-joint-trajectory-controller
```
###### ros-distro = jazzy

#### See graph
In new terminal *> robot@docker-desktop:~/**<ws_name>**$*
```
rqt_graph
```

#### See transformation matrices
In new terminal *> robot@docker-desktop:~/**<ws_name>**$*
```
ros2 run tf2_ros tf2_echo pelvis right_shoulder_pitch_link
```

## SCRIPTING / MOVEIT

#### Create new python scripting package
In terminal > robot@docker-desktop:~/SmartRobotics/src$:
```
ros2 pkg create --build-type ament_python agibot_x2_pkg_py
```
--> Script in agibot_x2_pkg_py/agibot_x2_pkg_py/
Update in setup.py: 
```
entry_points={
    'console_scripts': [
        'move_arm = agibot_x2_pkg_py.move_arm:main'
    ],
},
```

#### START THE SCRIPT
Terminal parallel to running Gazebo simulation > robot@docker-desktop:~/SmartRobotics/src$:
```
ros2 run agibot_x2_pkg_py move_arm
```

#### VERIFY IF DATA IS BEING SENT BY LISTENING TO TOPIC
Terminal parallel to running Gazebo simulation > robot@docker-desktop:~/SmartRobotics/src$:
```
ros2 topic echo /left_arm_controller/joint_trajectory
```

## CAMERE (Gazebo)

Il robot e la scena hanno **4 camere simulate** (definite in `urdf/control_file.gazebo` e `urdf/full_scene.urdf`, bridged verso ROS2 da `rviz_gaz_control.launch.py`):

| Camera | Topic immagine | Risoluzione | Cosa vede |
|---|---|---|---|
| Testa (RGBD) | `/rgbd_head_front/image` + `/rgbd_head_front/depth_image` | 320×240 | La scena davanti al robot (libreria). La depth è in metri (float32). |
| TCP mano sinistra | `/tcp_camera_left/image` | 320×240 | Fra le dita del gripper sinistro: l'oggetto mentre viene afferrato |
| TCP mano destra | `/tcp_camera_right/image` | 320×240 | Fra le dita del gripper destro (il braccio usato per il pick&place) |
| Tavolo | `/table_camera/image` | 640×480 | Il tavolo di staging dall'alto (piena risoluzione: serve per l'OCR) |

Ogni camera pubblica anche `<nome>/camera_info` (calibrazione) e le varianti compresse automatiche (`/compressed`, `/theora`, ecc. — stessi frame, altri formati).

#### VEDERE LE CAMERE — DA TERMINALE
Con la simulazione attiva (`rviz_gaz_control.launch.py`), in un altro terminale:
```
# elenca i topic camera attivi
ros2 topic list | grep -E "rgbd_head_front|tcp_camera|table_camera"

# verifica che una camera pubblichi e a che frequenza (atteso ~5 Hz)
ros2 topic hz /tcp_camera_right/image

# visualizzatore immagini con menu a tendina per cambiare camera
ros2 run rqt_image_view rqt_image_view

# oppure aperto direttamente su una camera
ros2 run rqt_image_view rqt_image_view /tcp_camera_right/image
```
> [!TIP]
> Per la **depth** della testa in `rqt_image_view`: la scena sta a 0.2–0.5 m ma la scala di default è 10 m, quindi appare quasi nera — abbassa il valore `10.00m` in alto a ~`2.00m` per vedere i contrasti.

#### VEDERE LE CAMERE — IN RVIZ
RViz è già aperto dal launch. In basso a sinistra nel pannello Displays: **Add → By topic →** scegli `/tcp_camera_right/image → Image` (o un'altra camera). Si apre un pannello immagine agganciabile. In alternativa: **Panels → Add New Panel → Image** e poi imposta il campo *Topic*.

#### VEDERE LE CAMERE — IN GAZEBO
Tutti i sensori hanno `<visualize>true</visualize>`: nella GUI di Gazebo clicca i tre puntini in alto a destra (⋮) → cerca e aggiungi il plugin **Image Display**, poi scegli il topic della camera dal menu del pannello. Utile per distinguere un problema di sensore (non si vede neanche qui) da un problema di bridge ROS (si vede qui ma non nei topic ROS).

#### VEDERE LA PRESA DI UN OGGETTO
Le camere TCP inquadrano lo spazio fra le dita: apri `/tcp_camera_right/image` in `rqt_image_view`, poi guida il braccio col teleop (sezione sotto) — quando le dita si avvicinano a un libro lo vedi entrare nell'inquadratura, e durante `c` (chiusura) resta in vista fra le ganasce.

## LIBRI FISICI DI TEST della scena `full` (pick & place vero)

> Vale per `scene:=full`. Nella scena di default (`grasp_test`, sezione sotto) le entità si chiamano `gt_*` e sono già tutte fisiche.

I libri della scena unica sono **solo visual** (una mesh unica, niente fisica individuale): il gripper non può afferrarli. Dal 2026-08-29 il launch spawna anche **2 libri fisici** afferrabili (`picktest_it` e `picktest_emma`, stesse mesh `.glb` con le texture reali di copertina/dorso/retro) sul ripiano alto, lato destro — disattivabili con `spawn_test_books:=false`.

Ogni libro ha un **DetachableJoint** verso il dito del gripper destro (pattern MOGI-ROS): la presa non si affida all'attrito, si "incolla" su comando.

```
# incolla il libro al dito destro (da fare A CONTATTO, dopo aver chiuso il gripper col teleop)
ros2 topic pub --once /picktest_it/attach std_msgs/msg/Empty {}

# rilascia
ros2 topic pub --once /picktest_it/detach std_msgs/msg/Empty {}

# stato attached/detached
ros2 topic echo /picktest_it/state
```

> [!IMPORTANT]
> Il plugin nasce **attaccato** (comportamento di Gazebo): il launch pubblica da solo il detach iniziale a +30s. Non comandare il braccio nei primi ~30 secondi.

Sequenza di prova consigliata (con `/tcp_camera_right/image` aperto in `rqt_image_view` per vedere la presa):
1. Teleop (sezione sotto), braccio destro: avvicina le dita a un libro di test.
2. `c` per chiudere il gripper sul dorso.
3. `ros2 topic pub --once /picktest_it/attach std_msgs/msg/Empty {}` → il libro è incollato.
4. Muovi il braccio: il libro segue. Portalo sul tavolo.
5. `.../detach` + `o` per rilasciarlo sul tavolo (dove lo vede `/table_camera/image`).

## SCENA DI DEFAULT "grasp_test" (primo scaffale: 4 libri + 2 oggetti, tutti afferrabili)

È l'ambiente della pipeline definitiva (vedi `src/TODO`) e parte **senza argomenti**: **libreria + tavolo** (modelli separati `bookshelf.urdf`/`table.urdf`, niente libri dipinti) e sul **primo scaffale dall'alto** (l'unico raggiungibile dal braccio e inquadrato dalla camera di testa), lato destro, **4 libri + 2 oggetti tutti fisici, con collision e afferrabili** (`book_placer.GRASP_TEST_ENTITIES`):

| # (tasto teleop) | Entità | Oggetto | Spessore mesh | Collision (spessore) | world y |
|---|---|---|---|---|---|
| 1 | `gt_hunger` | Hunger Games – trilogia | 7.0 cm | 6.4 cm | −0.335 |
| 2 | `gt_it` | IT | 5.5 cm | 4.9 cm | −0.253 |
| 3 | `gt_ballata` | Hunger Games – Ballata dell'usignolo | 4.5 cm | 3.9 cm | −0.183 |
| 4 | `gt_alba` | Hunger Games – Alba sulla mietitura | 3.8 cm | 3.2 cm | −0.122 |
| 5 | `gt_pen` | portapenne | 5.6 cm | 5.6 cm | −0.055 |
| 6 | `gt_mug` | tazza | 7.5 cm | 7.5 cm | +0.031 |

Libri a `x=0.34` con il dorso verso il robot, 2 cm fra una mesh e l'altra; oggetti a `x=0.40`. Ogni entità è un corpo **dinamico** con inerzia e box di collision: si può spingere, prendere, far cadere.

**Collision dei libri più stretta della mesh** (voce del TODO): larghezza copertina e altezza sono quelle della mesh, lo spessore della collision è ridotto di 3 mm per lato (`book_placer.BOOK_COLLISION_SIDE_MARGIN`). Così le dita (1 cm) entrano fra due libri anche vicini e, chiudendo, affondano leggermente nella mesh invece di fermarsi a filo. Tutti gli spessori di collision sono ≥ 3 cm = chiusura minima del gripper (`arm_kinematics.GRIPPER_MIN_GAP`, dita a ±2 cm): ogni libro si può **stringere** davvero, non solo agganciare.

Presa con gli slider / controller (`rqt_joint_trajectory_controller`, sezione "Activate Gazebo GUI controllers"): scegli `right_arm_controller` (e `waist_controller`), muovi i giunti finché le dita sono ai lati del dorso, poi `right_gripper_controller` per chiudere le dita (posizione per dito ≈ (spessore collision − 0.03)/2: IT ≈ 0.009, Hunger ≈ 0.017, Ballata ≈ 0.004, Mietitura 0). Per non affidarsi solo all'attrito c'è il DetachableJoint (`/gt_<nome>/attach|detach|state`, bridge in `gz_bridge.yaml`; il detach iniziale automatico parte a +25 s dal lancio):
```
ros2 topic pub --once /gt_it/attach std_msgs/msg/Empty {}      # incolla IT al dito destro (a contatto)
ros2 topic pub --once /gt_it/detach std_msgs/msg/Empty {}      # rilascia
ros2 run agibot_x2_pkg_py pick_place_teleop                    # tasti 1..6 = bersagli, b = chiudi allo spessore + attach, n = release
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=hunger   # hunger | it | ballata | alba | pen | mug
```
La camera tavolo (`/table_camera/image`) c'è anche qui (è dentro `table.urdf`); le camere del robot sono le stesse. `library_manager_node` funziona identico (trigger `head`, `rephotograph`).

Per tornare alla scena completa: `scene:=full` (vedi "AMBIENTE (scene)").

## PRESA AUTOMATICA DI UN LIBRO DI TEST (`pick_test_book`)

Sequenza completa senza tastiera, stile demo MOGI-ROS: le pose del braccio sono calcolate dalla **cinematica inversa** ricavata dall'URDF (`agibot_x2_pkg_py/arm_kinematics.py`), a partire dalla posizione nota del libro (`book_placer.test_entities(scene)`).

```
# con la simulazione su e i controller attivi (aspetta anche il detach automatico a +25s)
ros2 run agibot_x2_pkg_py pick_test_book                       # IT (scena di default grasp_test)
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=ballata    # hunger | it | ballata | alba | pen | mug
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p scene:=full -p book:=emma   # scena completa: it | emma
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p dry_run:=true   # solo il calcolo IK, il robot non si muove
```

Cosa fa: apre il gripper → busto e braccio in pre-grasp (8 cm davanti al dorso) → avvicinamento **rettilineo** al punto di presa (2.5 cm oltre il dorso) → chiude le dita allo spessore del libro → `attach` → sfila il libro all'indietro in linea retta → posa di trasporto (braccio raccolto, non spazza la libreria) → vita a −90° verso il tavolo → busto in avanti, braccio teso oltre il bordo → `detach` + apre (il libro cade sul tavolo) → torna a casa. Con `/tcp_camera_right/image` aperto vedi tutto in soggettiva.

> [!NOTE]
> Il calcolo IK dura ~1 min su questa CPU (una volta sola, all'avvio). Se stampa `IK con errore > 2 cm` il libro non è raggiungibile da dove sta il robot: controlla `robot_x` (default −0.10, come lo spawn del launch).

Perché il robot ora parte a `x=-0.10` invece di `0.0`: a 25 cm dallo scaffale la spalla destra era troppo vicina e alta per un approccio orizzontale (il gripper arrivava solo da sopra); 10 cm più indietro l'IK trova una presa laterale pulita (dita lungo Y a 1°, avvicinamento a 7°). I libri di test stanno a y=−0.20/−0.30 — la zona davanti alla spalla destra: il roll della spalla è limitato a +0.06 rad e il braccio destro non può portarsi verso il centro del corpo.

**Limite noto**: il tavolo è al limite della portata — il libro viene **lasciato cadere** sul bordo, non appoggiato.

## PROVA COMPLETA DELLA PIPELINE (occhi → presa → tavolo → identificazione)

Flusso **ibrido**: percezione e identificazione automatiche (`library_manager_node`), presa guidata col teleop, completamento dell'identificazione automatico sul tavolo. Servono 4 terminali.

> Flusso scritto per la scena completa (`scene:=full`, libri `picktest_it`/`picktest_emma`). Nella scena di default (`grasp_test`) è identico con i nomi `gt_*` (`/gt_it/attach`, ecc.) e senza `scene:=full` nei comandi.

**T1 — simulazione**
```
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py scene:=full
```
Aspetta che `ros2 control list_controllers` mostri tutto `active` e che nei log passino i `topic pub .../detach` (+25s): prima non toccare i bracci.

**T2 — nodo pipeline** (venv obbligatorio per YOLO/EasyOCR)
```
source .venv/bin/activate
source install/setup.bash
ros2 run agibot_x2_pkg library_manager_node --ros-args -p "default_sort:=''"
```
`default_sort:=''` (con quelle virgolette esatte: la shell altrimenti manda un valore vuoto che rcl non accetta) evita che dopo l'analisi parta da sola la coreografia automatica (braccio sinistro, angoli non calibrati) che intralcerebbe il teleop. Attendi `LibraryManagerNode pronto.`

**T3 — comandi**

1. Punta la testa sul ripiano alto (a riposo la camera fissa la tavola):
```
ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [head_yaw_joint, head_pitch_joint], points: [{positions: [0.0, -0.35], time_from_start: {sec: 2}}]}}"
```
2. Controlla in `rqt_image_view` (`/rgbd_head_front/image`) che IT ed Emma siano inquadrati, poi fai analizzare **ciò che il robot vede**:
```
ros2 topic pub -1 /library_manager/trigger std_msgs/String "data: 'head'"
```
In T2: detection → colori → OCR. A 320×240 i titoli saranno spesso parziali: **è atteso**, si completano al passo 5. Segnati gli `obj_id` dei libri (`ros2 topic echo /library_manager/detections --once`, oppure guarda `/tmp/x2_detections.jpg`).

**T4 — presa col teleop** (braccio destro, default)
```
ros2 run agibot_x2_pkg_py pick_place_teleop --ros-args -p scene:=full
```
3. Con `/tcp_camera_right/image` aperto in rqt: porta le dita ai lati di IT, chiudi con `c`, poi da T3:
```
ros2 topic pub --once /picktest_it/attach std_msgs/msg/Empty {}
```
4. Muovi il braccio: il libro segue. `h` ripetuto per girare la vita verso il tavolo, abbassa, poi:
```
ros2 topic pub --once /picktest_it/detach std_msgs/msg/Empty {}
```
e `o` per aprire. Il libro è sul tavolo (`/table_camera/image`).

5. **Completa l'identificazione dal tavolo** (camera 640×480, da vicino):
```
ros2 topic pub -1 /library_manager/rephotograph std_msgs/String "data: ''"
```
(`data: '<obj_id>'` per un libro preciso; vuoto = primo libro con titolo/autore mancanti). In T2 cerca `Ri-identificazione libro [N] dal tavolo: title='...' author='...'`.

6. Ripeti 3-5 con Emma (`/picktest_emma/attach|detach`).

Non ancora automatico (prossimi passi): attach/detach dentro l'azione GRASP della sequenza, ritarget della sequenza sul braccio destro con pose dalle detection, rotazione fisica copertina/retro→ISBN.

## PICK & PLACE TELEOP

#### START THE TELEOP SCRIPT
Terminal parallel to running Gazebo simulation (`rviz_gaz_control.launch.py`) > robot@docker-desktop:~/SmartRobotics$:
```
ros2 run agibot_x2_pkg_py pick_place_teleop
```
Click once on this terminal window and leave the keyboard focus there (not on the Gazebo window) — keys only reach the script from there.

Braccio destro attivo di default (i libri raggiungibili stanno sul suo lato, vedi Gazebo.md "Raggiungibilita del braccio" nella documentazione del progetto).

| Tasti | Effetto |
|---|---|
| `q` / `a` | shoulder_pitch +/- |
| `w` / `s` | shoulder_roll +/- |
| `e` / `d` | shoulder_yaw +/- |
| `r` / `f` | elbow +/- |
| `t` / `g` | wrist_yaw +/- |
| `y` / `h` | waist_yaw +/- |
| `u` / `j` | waist_pitch +/- |
| `o` | apri gripper (braccio attivo) — **il gripper parte chiuso**: premilo prima di avvicinarti a un libro |
| `c` | chiudi gripper a fondo (per i libri usa `b`, che non compenetra) |
| `1` … `6` | bersaglio: nella scena di default `1` Hunger Games, `2` IT, `3` Ballata, `4` Mietitura, `5` portapenne, `6` tazza (con `-p scene:=full`: `1` IT, `2` Emma) — l'elenco è stampato all'avvio |
| `b` | **grasp automatico**: chiude le dita allo spessore del libro bersaglio (da `BOOK_CATALOG`) e dopo 3 s pubblica l'`attach` del DetachableJoint — il libro segue la mano. Solo braccio destro. |
| `n` | **release**: `detach` del libro bersaglio + apre il gripper |
| `z` | cambia braccio attivo (destro <-> sinistro) |
| `x` | home (braccio/vita attivi a 0) |
| `p` | stampa le posizioni correnti |
| `CTRL-C` | esci |

#### QUICK PICK & PLACE SEQUENCE (libreria -> tavolo)
1. `q` / `f` a piccoli passi finché le dita non sono ai lati di un libro del ripiano alto (z ≈ 0.99-1.10 m).
2. `c` per chiudere il gripper sul libro.
3. `a` per allontanare il libro dallo scaffale.
4. `h` ripetuto per girare la vita verso il tavolo (~-1.935 rad, controlla con `p`).
5. `q`/`a`/`f` per abbassare verso il tavolo, poi `o` per rilasciare.
6. `x` per tornare a casa.
