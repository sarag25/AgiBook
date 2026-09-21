> **Guida completa storica** (era il README fino al 2026-09-21). Il README attuale è più breve: vedi [../README.md](../README.md).

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

#### SE LO SPAWNER MUORE PER TIMEOUT ALL'AVVIO
Negli avvii lenti (`Failed getting a result from calling /controller_manager/load_controller in 180.0`) lo spawner può esaurire i tentativi mentre la simulazione è ancora in stallo. Quando l'RTF si è ripreso, rilancialo a mano (secondi, a quel punto):
```
ros2 run controller_manager spawner left_arm_controller right_arm_controller head_controller waist_controller waist_roll_controller --param-file install/agibot_x2_pkg/share/agibot_x2_pkg/config/x2_controllers.yaml --ros-args -p use_sim_time:=true
```
(niente `*_gripper_controller`: dal 2026-09-06 le dita sono dentro i controller braccio, 7 giunti; `waist_roll_controller` tiene fermo il roll della vita, che prima era un giunto libero e cedeva sotto il braccio teso). Verifica con `ros2 control list_controllers`: 5 controller + broadcaster `active`.

#### SE GAZEBO RESTA VUOTO E `create` RIPETE "Waiting for service /world/bookshelf_world/create"
Il server Gazebo non ha caricato il mondo (dal 2026-09-06 il launch lo avvia con `-s`, che carica il mondo direttamente, e apre la GUI a parte: `gz_gui:=false` per non aprirla e risparmiare un core). Se ricapita: Ctrl+C e rilancia; con pochi core chiudi prima RViz/rqt.

#### CAMERE A SCATTO (shelf_camera e table_camera)
Dal 2026-09-08 le due camere "fotografiche" sono **triggered** in Gazebo: non renderizzano in continuo (niente CPU/RAM sprecate), producono un frame solo quando arriva `true` sul loro topic di scatto. `library_manager_node` scatta da solo (trigger `'shelf'` e ri-foto dal tavolo) e aspetta il frame. Per vederle in `rqt_image_view` scatta a mano:
```
ros2 topic pub -1 /shelf_camera/trigger std_msgs/msg/Bool "data: true"
ros2 topic pub -1 /table_camera/trigger std_msgs/msg/Bool "data: true"
```
(l'immagine in rqt resta l'ultima scattata). Le camere della testa e del TCP restano continue: servono in tempo reale.

#### SE `library_manager_node` MUORE CON "Cannot allocate memory" (import di torch)
La VM WSL2 ha 8 GB: server Gazebo 2,3 GB + **GUI Gazebo 2,3 GB** + RViz 0,7 GB lasciano ~600 MB, e torch da solo ne vuole >1 GB (SAM3 altri 3–4). Rimedi: chiudi la finestra Gazebo Sim (il server continua; o `gz_gui:=false` al lancio); per le prove che non passano dalla foto dello scaffale (ri-foto dal tavolo, ISBN, piano su detection note) usa `-p detector:=none` (niente SAM3/YOLO, parte in secondi); per SAM3 chiudi anche RViz durante l'analisi. Cura definitiva: `.wslconfig` su Windows con più `memory=`.

#### SE RVIZ MOSTRA "ROS Time 0.00" E RobotModel "No transform" SU TUTTI I LINK
Con i controller `active` e Gazebo in moto, il colpevole è il `parameter_bridge`: partito prima che il mondo esistesse, sotto carico può non agganciare mai `/clock` (processo vivo, zero messaggi). Diagnosi in 10 s: `ros2 topic echo /clock --once` non stampa niente mentre `gz topic -e -t /clock -n 1` sì. Cura senza rilanciare tutto — riavvia solo il bridge:
```
pkill -f parameter_bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=install/agibot_x2_pkg/share/agibot_x2_pkg/config/gz_bridge.yaml -p use_sim_time:=true
```
(lascialo in quel terminale; chiudilo con Ctrl+C insieme alla simulazione). Dal 2026-09-06 il launch avvia i bridge **dopo** lo spawn del robot proprio per evitarlo.

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
| Testa (rgbd **a scatto**, 2026-09-16: l'unica camera del robot) | `/head_camera/image` + `/head_camera/depth_image`, trigger `/head_camera/trigger` | 1920×1440, hfov 1.0 | La scena davanti al robot (libreria). La depth è in metri (float32). |
| TCP mano sinistra | `/tcp_camera_left/image` | — | **Spenta** (commentata in `control_file.gazebo` dal commit "Commented cameras") |
| TCP mano destra | `/tcp_camera_right/image` | 640×480 @ 15 Hz sim | Fra le dita del gripper destro (il braccio usato per il pick&place); riattivata il 2026-09-13 per il video |
| Tavolo | `/table_camera/image` | 2560×1920, **a scatto** | Il tavolo di staging dall'alto, sopra il punto di rilascio (ISBN dal retro del libro) |
| Scaffale | `/shelf_camera/image` | 960×720, **a scatto** | Fototessera dei 4 libri per SAM3/OCR |
| Regista (`video:=true`) | `/video_camera/image` | 1280×720 @ 20 Hz sim | Vista obliqua dall'alto di tutta la scena, per il video (vedi sezione VIDEO) |

Ogni camera pubblica anche `<nome>/camera_info` (calibrazione) e le varianti compresse automatiche (`/compressed`, `/theora`, ecc. — stessi frame, altri formati).

#### VEDERE LE CAMERE — DA TERMINALE
Con la simulazione attiva (`rviz_gaz_control.launch.py`), in un altro terminale:
```
# elenca i topic camera attivi
ros2 topic list | grep -E "head_camera|tcp_camera|table_camera"

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

## VIDEO PER LA PRESENTAZIONE (vista dall'alto + vista dalla mano, senza GPU)

Senza GPU la simulazione va a RTF ~0,1 e ogni feed a schermo va a 1-2 fps: registrare lo schermo viene a scatti. La soluzione (2026-09-13) è registrare **in tempo simulato**: una camera "regista" fissa (`urdf/video_camera.urdf`, vista obliqua dall'alto su robot, percorso, libreria e tavolo, 1280×720 @ 20 Hz simulati, `video:=true`) più la camera nel gripper destro (`/tcp_camera_right/image`, 640×480 @ 15 Hz simulati). Il nodo `record_video` rimette ogni frame al suo istante simulato e scrive mp4 fluidi a 25 fps, a prescindere da quanto è lento il PC. Niente GUI Gazebo né RViz durante la registrazione.

**T1 — simulazione headless con la camera regista**
```
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py video:=true gz_gui:=false rviz:=false
```
Aspetta i 7 controller + broadcaster `active` e il detach delle 6 entità. Se lo spawner resta su `switch_controller in 180.0` o `list_controllers`: quasi sempre c'è un **server Gazebo orfano** di un launch precedente (`pgrep -af "gz sim -s"` → due righe; `pkill -f "gz sim -s"` e rilancia; dal 2026-09-13 il launch lo rileva e si rifiuta di partire), oppure la sim è in pausa (vedi Bugs.md).

**OCR e ricerca titoli (2026-09-17).** Il lettore unisce tutti i frammenti letti sul dorso (`HUNGER GAMES`, `STEPHEN KING`), sceglie la rotazione con più lettere e riprova con contrasto aumentato sui dorsi scuri. La ricerca per titolo prova Google Books e, se non risponde (senza chiave API la quota giornaliera condivisa finisce presto: HTTP 429), OpenLibrary. Metti `GOOGLE_BOOKS_API_KEY=...` nel `.env` per avere una quota tua. Nel log del manager: `obj N: Google Books da OCR '...' -> ...` oppure `... non trovato`.

**Scena `ocr_test` (2026-09-17)**: `ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py scene:=ocr_test gz_gui:=false rviz:=false` mette i 15 libri del catalogo in fila sul ripiano alto (19 mm fra l'uno e l'altro: serve alla prova dell'OCR, non alle prese). Risultato della prova (Bugs.md): dal dorso si identificano i libri con titolo grande (Never Flinch, Black Widow, Fantastici 4, Hunger Games); Alba/Ballata/IT/Emma/Cat's Cradle vanno all'ISBN; le enciclopedie non esistono su Google Books e ora non producono più falsi positivi.

**Foto zoomata per libro (2026-09-17).** Dalla foto a 1 m il titolo piccolo dei libri di una serie non si legge (5 px) e la ricerca trova il libro sbagliato ("HUNGER GAMES" → primo della serie invece di "L'alba sulla mietitura"). Con il robot alla posa di lavoro (`walk_to_shelf -p distance:=1.5`) e le detection già fatte:
```
ros2 topic pub -1 /library_manager/trigger std_msgs/String "data: 'zoom'"
```
Per ogni libro il manager punta busto e testa sul dorso misurato, scatta (33–41 px/cm), ritaglia il dorso proiettato (`/tmp/x2_zoom_N_objK.jpg`), rifà OCR unendo titolo verticale e sottotitolo orizzontale, cerca il titolo e ripubblica le detection con ISBN/titolo/autore aggiornati. Log: `zoom obj 1: OCR '... VaLBA Gulla | Mietitura | HUNGER | GAMES' -> identificato 'L'alba sulla mietitura' Suzanne Collins ...`. Un dorso con il solo nome dell'autore ("STEPHEN KING") non viene identificato di proposito: va all'ISBN.

**Camere (2026-09-17)**: shelf_camera e table_camera sono state rimosse. Restano `head_camera` (rgbd 1920×1440 a scatto sulla testa: foto della libreria, zoom per l'OCR, ISBN con il libro in mano), `tcp_camera_right` (solo per il filmato) e la `video_camera` regista (solo con `video:=true`). I comandi `data: 'shelf'`/`'zoom'` del manager e `pick_test_book -p head_isbn:=true` usano tutti la head_camera; la ri-foto dal tavolo (T5) non esiste più.

**Camera della libreria = testa (2026-09-16).** `library_manager_node` fotografa la libreria con la `head_camera` (default `camera:=head`; `camera:=shelf` per la vecchia shelf_camera): prima dello scatto mette la testa su (`look_head_pitch` −0.38) e il busto indietro (`look_waist_pitch` −0.31), legge `/joint_states` e calcola la posa della camera nel mondo (`HeadCameraPose`: base mobile + vita + testa + posa di spawn). Il robot deve stare a 0,8–1,3 m dai dorsi (es. `walk_to_shelf -p distance:=0.9`); dalla posa di lavoro (0,37 m) i libri ai lati escono dall'inquadratura. Se il launch non usa il `walk_distance` di default (spawn a x −1.6), passa `-p robot_spawn_x:=<x di spawn>` (con `walk_distance:=0`: −0.1). All'avvio il launch scatta una foto a vuoto (`head_camera_warmup`): il primo render di un sensore rgbd in Gazebo esce nero (vedi Bugs.md), i successivi no. Log atteso: `sguardo: testa ... fatto`, `head_camera a (-0.74, -0.01, 1.20), asse ottico (1.00, 0.00, -0.01)`, poi `Scatto #N`.

**T2 — nodo pipeline** (venv, radice del repo; niente SAM3 nella scena filmata)
```
source .venv/bin/activate && source install/setup.bash
ros2 run agibot_x2_pkg library_manager_node --ros-args -p detector:=none -p "default_sort:=''" -p plan_only:=true
```

**T3 — registratore** (parte quando arrivano i primi frame; Ctrl+C alla fine chiude i file)
```
ros2 run agibot_x2_pkg_py record_video --ros-args -p prefix:=presa_it
```
Log ogni 5 s: `frame ricevuti: regista N, tcp M; video: X s simulati`. Se `regista 0`: launch senza `video:=true`.

**T4 — la scena** (camminata → presa di IT → foto dal tavolo con ISBN)
```
ros2 run agibot_x2_pkg_py walk_to_shelf && \
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=it && \
ros2 topic pub -1 /library_manager/rephotograph std_msgs/String "data: '3'"
```
Quando in T2 compare `ISBN dal barcode: [...]` e i metadati, Ctrl+C in T3. Output in `videos/`:
`presa_it_main.mp4` (regista), `presa_it_pip.mp4` (mano), `presa_it_combo.mp4` (regista con la mano in picture-in-picture in basso a destra, già sincronizzate: stesso clock simulato). Durata = tempo simulato (~60 s per tutta la sequenza); il tempo reale di registrazione sarà ~10× tanto.

Per il montaggio: davanti al combo metti `/tmp/x2_detections.jpg` (foto dello scaffale con le detection, fatta a parte con SAM3) e in coda `/tmp/x2_table_photo.jpg` + le righe di log con ISBN/titolo/autore/anno. Posa/inquadratura della regista: `video_camera_joint` in `video_camera.urdf` (i commenti spiegano come è stata calcolata).

## LIBRI FISICI DI TEST della scena `full` (pick & place vero)

> Vale per `scene:=full`. Nella scena di default (`grasp_test`, sezione sotto) le entità si chiamano `gt_*` e sono già tutte fisiche.

I libri della scena unica sono **solo visual** (una mesh unica, niente fisica individuale): il gripper non può afferrarli. Dal 2026-08-29 il launch spawna anche **2 libri fisici** afferrabili (`picktest_it` e `picktest_emma`, stesse mesh `.glb` con le texture reali di copertina/dorso/retro) sul ripiano alto, lato destro — disattivabili con `spawn_test_books:=false`.

Ogni libro ha un **DetachableJoint** verso il dito del gripper destro (pattern MOGI-ROS): la presa non si affida all'attrito, si "incolla" su comando.

```
# incolla il libro al dito destro (da fare A CONTATTO, dopo aver chiuso il gripper col teleop)
# (dal 2026-09-06 preferisci l'interfaccia unica: ros2 topic pub -1 /gripper/right/attach std_msgs/msg/String "data: 'picktest_it'")
ros2 topic pub --once /picktest_it/attach std_msgs/msg/Empty {}

# rilascia
ros2 topic pub --once /picktest_it/detach std_msgs/msg/Empty {}

# stato attached/detached
ros2 topic echo /picktest_it/state
```

> [!IMPORTANT]
> Il plugin nasce **attaccato** (comportamento di Gazebo, non disattivabile): il launch avvia i publisher di detach **prima** dello spawn delle entità (5 Hz per 30 s), così il vincolo si scioglie entro una frazione di secondo dalla nascita. Lascia comunque fermo il braccio nei primi secondi di assestamento.

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
| 1 | `gt_hunger` | Hunger Games – trilogia | 7.0 cm | 6.4 cm | −0.320 |
| 2 | `gt_it` | IT | 5.5 cm | 4.9 cm | −0.238 |
| 3 | `gt_ballata` | Hunger Games – Ballata dell'usignolo | 4.5 cm | 3.9 cm | −0.168 |
| 4 | `gt_alba` | Hunger Games – Alba sulla mietitura | 3.8 cm | 3.2 cm | −0.107 |
| 5 | `gt_pen` | portapenne | 5.6 cm | 5.6 cm | −0.040 |
| 6 | `gt_globe` | mappamondo da scrivania | 8.0 cm | 8.0 cm | +0.050 |

Libri con i **dorsi allineati sul piano `x=0.27`** (centro = 0.27 + larghezza/2: le larghezze sono diverse e con i centri allineati Hunger Games sporgeva); portapenne a `x=0.40`, mappamondo al fronte (`x=0.31`). **Gap ≥ 2 cm fra ogni oggetto e il vicino/la parete** (2026-09-06): le dita sono spesse 1 cm e devono entrare ai lati senza urtare — con i valori precedenti Hunger Games stava a 8 mm dalla parete e portapenne/mappamondo a 7 mm l'uno dall'altro. Il mappamondo ha sostituito la tazza (forma non cilindrica come richiesto, e con 8 cm entra ancora nell'apertura massima della pinza). Ogni entità è un corpo **dinamico** con inerzia e box di collision: si può spingere, prendere, far cadere.

**Collision dei libri più stretta della mesh** (voce del TODO): larghezza copertina e altezza sono quelle della mesh, lo spessore della collision è ridotto di 3 mm per lato (`book_placer.BOOK_COLLISION_SIDE_MARGIN`). Così le dita (1 cm) entrano fra due libri anche vicini e, chiudendo, affondano leggermente nella mesh invece di fermarsi a filo. Tutti gli spessori di collision sono ≥ 3 cm = chiusura minima del gripper (`arm_kinematics.GRIPPER_MIN_GAP`, dita a ±2 cm): ogni libro si può **stringere** davvero, non solo agganciare.

Presa con gli slider (`rqt_joint_trajectory_controller`, sezione "Activate Gazebo GUI controllers"): scegli `right_arm_controller` (e `waist_controller`) — le due dita sono **dentro** `right_arm_controller` (7 giunti; `right_gripper_controller` non esiste più). Attenzione: con le dita a tutta apertura (0.037) le facce esterne stanno a ±6,2 cm dal centro e il libro accanto a ±4,75 cm: **apri solo quanto serve** (IT ≈ 0.017–0.019 per dito, Hunger ≈ 0.024–0.026, Ballata ≈ 0.012–0.014, Mietitura ≈ 0.008–0.010) o le dita urtano i vicini prima di entrare. Chiusura ≈ (spessore collision − 0.03)/2: IT ≈ 0.009, Hunger ≈ 0.017, Ballata ≈ 0.004, Mietitura 0.

**Attach/detach unico** (2026-09-06): non serve più conoscere il topic dell'oggetto. Il `GraspManagerNode` (avviato dal launch, `grasp_manager:=false` per spegnerlo) legge i **sensori di contatto delle dita** e stampa `Contatto right: gt_it` quando le tocchi; l'attach/detach passa da tre topic globali:
```
ros2 topic pub -1 /gripper/right/attach std_msgs/msg/String "data: 'gt_it'"   # incolla IT (avvisa se il dito tocca altro)
ros2 topic pub -1 /gripper/right/attach std_msgs/msg/String "data: ''"        # incolla CIO' CHE IL DITO TOCCA
ros2 topic pub -1 /gripper/right/detach std_msgs/msg/Empty {}                 # stacca cio' che e' agganciato
ros2 topic echo /gripper/right/attached --once                                # entita' agganciata ('' = niente)
ros2 run agibot_x2_pkg_py pick_place_teleop                    # 1..6 = bersagli, b = chiudi allo spessore + attach, v = attach di cio' che tocco, n = release
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=hunger   # hunger | it | ballata | alba | pen | globe (tutto automatico)
```
Sotto restano i topic per entità del DetachableJoint (`/gt_<nome>/attach|detach|state`, plugin nel modello dell'oggetto) — le loro voci del bridge sono **generate al launch** da `agibot_x2_pkg/bridge_config.py` (`/tmp/gz_bridge_<scene>.yaml`): `config/gz_bridge.yaml` contiene solo la base statica (clock, contatti, camera_info) e deve restare YAML valido (`python3 -c "import yaml,sys; yaml.safe_load(open(sys.argv[1]))" src/agibot_x2_pkg/config/gz_bridge.yaml`; niente `'''` come commento). Il detach iniziale automatico parte **prima** dello spawn delle entità e martella per 30 s, quindi nasce tutto già staccato. Tutto il dettaglio (geometria delle dita, sensori, GraspManager) è nella nota Obsidian `PickAndPlace.md`.

> [!IMPORTANT]
> Dopo aver toccato `agibot_x2_pkg_py` (teleop, pick_test_book, gripper_controller) va ricompilato **anche quel pacchetto**: `colcon build --packages-select agibot_x2_pkg agibot_x2_pkg_py` — l'install stantio era il motivo per cui il nodo dei contatti "non stampava".
La camera tavolo (`/table_camera/image`) c'è anche qui (è dentro `table.urdf`); le camere del robot sono le stesse. `library_manager_node` funziona identico (trigger `head`, `rephotograph`).

Per tornare alla scena completa: `scene:=full` (vedi "AMBIENTE (scene)").

## PRESA AUTOMATICA DI UN LIBRO DI TEST (`pick_test_book`)

> **MoveIt 2 (2026-09-18).** Con `ros2 launch agibot_x2_pkg moveit.launch.py` acceso (dopo la simulazione), `pick_test_book` pianifica i tratti liberi con `move_group` (collisioni con libreria, tavolo, oggetti misurati e oggetto in mano) e verifica i tratti rettilinei contro la planning scene; se un tratto tocca, si ferma e stampa le coppie in collisione. `-p moveit:=false` per il comportamento di prima. Config in `src/agibot_x2_pkg/config/moveit/README.md`.

> **2026-09-17 (notte).** `-p arm:=right|left|auto` (default `auto`: braccio sinistro solo per gli oggetti chiaramente a sinistra, y > 0,10 m, destro altrimenti — dal vivo il sinistro non ha un avvicinamento rettilineo al mappamondo a y=+0,05; a sinistra cambiano controller, sensori, topic `/<entità>/attach_left|detach_left`, `/gripper/left/*`). **Mai comandare le dita a 0.0**: chiuse a fine corsa le dita esterne restano bloccate per sempre (`FINGER_MIN` 2 mm nel nodo; vedi Bugs.md). `walk_to_shelf -p shelf_distance:=0.666` porta il robot alla posa di lavoro misurando la distanza dalla libreria con la depth della testa (`-p measure_only:=true` solo misura).

Sequenza completa senza tastiera, stile demo MOGI-ROS: le pose del braccio sono calcolate dalla **cinematica inversa** ricavata dall'URDF (`agibot_x2_pkg_py/arm_kinematics.py`), a partire dalla posizione nota del libro (`book_placer.test_entities(scene)`).

Dal 2026-09-06 (vedi `PickAndPlace.md` in Obsidian): (1) le dita si aprono **quanto basta** per entrare ai lati dell'oggetto senza urtare i vicini (`approach_opening`, calcolata dallo spessore e dallo spazio libero; se non c'è spazio si ferma con un errore chiaro); (2) se l'oggetto è fuori portata a busto dritto l'IK viene ritentata con la **vita libera in yaw** (serve per portapenne e mappamondo); (3) il rilascio è un punto **dentro il tavolo** calcolato dall'IK (bordo del tavolo avvicinato a y=−0.33), non più il braccio teso sul bordo. Prova a secco prima: `-p dry_run:=true` stampa aperture ed errori IK senza muovere niente. Il GraspManager, se attivo, logga il contatto e lo stato `attached/detached` in parallelo.

Dalla sera del 2026-09-06: (4) l'uscita dallo scaffale è lunga quanto serve perché l'oggetto sia **tutto fuori** prima di ruotare la vita (prima IT restava dentro di 2,5 cm e travolgeva i vicini); (5) i **libri vengono posati di piatto, copertina in giù** (polso ruotato di 90°), così il retro con il codice a barre guarda la `table_camera` (riattivata sopra il punto di rilascio, 1920×1440). Per leggere l'ISBN e i metadati dal tavolo, con `library_manager_node` acceso **dalla radice del repo** (`cd ~/SmartRobotics`, venv attivo — usa `sorting/extract_isbn.py`, pyzbar + OpenLibrary/Google Books, serve rete):
```
ros2 topic pub -1 /library_manager/rephotograph std_msgs/String "data: ''"      # '' = primo libro senza titolo/autore, oppure "data: '3'" = obj_id
```
Nel log del nodo: `Foto tavolo salvata: /tmp/x2_table_photo.jpg`, `ISBN dal barcode: ['9788868365622']`, `ISBN 9788868365622: title='It' author='Stephen King' year='1987'`; titolo/autore/anno dai metadati sovrascrivono l'OCR del dorso e finiscono in `/tmp/x2_detections.json` (`isbn`, `year`). Senza barcode leggibile resta l'OCR. La stessa foto si può analizzare a mano con la pipeline standalone: `python3 sorting/identify_book.py` (vedi `sorting/`).

```
# con la simulazione su e i controller attivi (aspetta anche il detach automatico a +25s)
ros2 run agibot_x2_pkg_py pick_test_book                       # IT (scena di default grasp_test)
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=ballata    # hunger | it | ballata | alba | pen | globe
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p scene:=full -p book:=emma   # scena completa: it | emma
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p dry_run:=true   # solo il calcolo IK, il robot non si muove
```

Cosa fa: apre il gripper → busto e braccio in pre-grasp (8 cm davanti al dorso) → avvicinamento **rettilineo** al punto di presa (2.5 cm oltre il dorso) → chiude le dita allo spessore del libro → `attach` → sfila il libro all'indietro in linea retta → posa di trasporto (braccio raccolto, non spazza la libreria) → vita a −90° verso il tavolo → busto in avanti, braccio teso oltre il bordo → `detach` + apre (il libro cade sul tavolo) → torna a casa. Con `/tcp_camera_right/image` aperto vedi tutto in soggettiva.

> [!NOTE]
> Il calcolo IK dura ~1 min su questa CPU (una volta sola, all'avvio). Se stampa `IK con errore > 2 cm` il libro non è raggiungibile da dove sta il robot: controlla `robot_x` (default −0.10, come lo spawn del launch).

Perché il robot ora parte a `x=-0.10` invece di `0.0`: a 25 cm dallo scaffale la spalla destra era troppo vicina e alta per un approccio orizzontale (il gripper arrivava solo da sopra); 10 cm più indietro l'IK trova una presa laterale pulita (dita lungo Y a 1°, avvicinamento a 7°). I libri di test stanno a y=−0.20/−0.30 — la zona davanti alla spalla destra: il roll della spalla è limitato a +0.06 rad e il braccio destro non può portarsi verso il centro del corpo.

**Limite noto**: il tavolo è al limite della portata — il libro viene **lasciato cadere** sul bordo, non appoggiato.

## PRESA AUTOMATICA DA PERCEZIONE (il robot misura il libro e decide da solo)

Dal 2026-09-13 la `shelf_camera` è rgbd: dalla depth della foto dello scaffale `library_manager_node` calcola per ogni oggetto rilevato posizione del dorso (`world_x`, `world_y`), spessore, altezza e spazio libero ai lati (`vision/shelf_geometry.py`), e `pick_test_book` può prendere un oggetto **dal JSON** invece che dalle pose note (`book_placer`). Funziona con SAM3 (`detector:=sam3`, identifica anche i titoli) o con il detector geometrico `detector:=depth` (nessuna rete neurale, parte in un secondo: per il PC senza GPU).

```
# T1  ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py gz_gui:=false   (poi walk_to_shelf)
# T2  (venv, radice repo)
ros2 run agibot_x2_pkg library_manager_node --ros-args -p detector:=depth -p "default_sort:=''" -p plan_only:=true
# T3  foto + misure
ros2 topic pub -1 /library_manager/trigger std_msgs/String "data: 'shelf'"
#     in T2: "obj 3 book: dorso x=0.270 y=-0.238 z=0.993..1.228 spessore 55 mm altezza 235 mm, liberi +y 19 / -y 17 mm"
cat /tmp/x2_detections.json          # scegli l'id (o guarda /tmp/x2_detections.jpg)
#     ogni scatto resta anche con suffisso: /tmp/x2_shelf_photo_N.jpg, x2_shelf_depth_N.npy,
#     x2_detections_N.jpg/.json (N = numero dello scatto da quando T2 e' acceso)
# T3  presa: id, oppure 'auto' (primo libro), oppure una parola del titolo (con SAM3+OCR)
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p target:=3
ros2 topic pub -1 /library_manager/rephotograph std_msgs/String "data: '3'"    # ISBN dal tavolo
```
Con `target` la chiusura è **a contatto** (`close_mode:=contact`: le dita si chiudono a 5 mm/s finché il sensore del dito non tocca l'oggetto; nel launch `Contatto right: gt_it`) e l'attach passa dal GraspManager (`ATTACH right -> gt_it`). `-p dry_run:=true` per il solo piano IK. `-p close_mode:=fixed` chiude sullo spessore misurato. Se il JSON non ha `thickness_m`: la depth non è arrivata (rilancia con il `bookshelf.urdf` rgbd, ricompilato).

**Oggetti bassi** (2026-09-13, dalla prima esecuzione reale): mappamondo e portapenne si prendono a 2,5 cm dalla cima, non a metà altezza, e prima di muoversi il nodo controlla che avambraccio e attacco della pinza restino ≥1 cm sopra il ripiano oltre il bordo anteriore (x=0.25), sia nella presa sia lungo l'avvicinamento (log `avambraccio sopra il ripiano: +N mm`); se non basta alza la presa di 1 cm alla volta. L'avvicinamento è pianificato all'indietro dalla posa di presa, così finisce esattamente lì. Per gli oggetti l'apertura di avvicinamento è `p_min+10 mm` quando c'è spazio (lo spessore misurato dalla depth di un oggetto tondo è la faccia frontale, non la larghezza massima). Se leggi `dita chiuse senza contatto` con l'oggetto fermo, la mano non è arrivata: confronta i giunti reali con il comando (vedi Bugs.md).

## PIPELINE AUTOMATICA COMPLETA (`library_pipeline`, 2026-09-13)

Un solo comando fa: foto della libreria → **oggetti** (non libri) presi e parcheggiati sul tavolo → seconda foto dei soli libri → OCR dei dorsi + **Google Books per titolo** (metadati) → i libri **senza** metadati portati uno alla volta a faccia in giù sotto la `table_camera`, ISBN dal codice a barre → metadati. Risultato in `/tmp/x2_library.json`. Ogni presa è un processo `pick_test_book -p target:=<id> -p release_x/y` (gli stessi comandi che daresti a mano), le foto passano da `library_manager_node`. Solo mano destra (vedi Obsidian PickAndPlace.md: la sinistra non ha DetachableJoint né IK).

```
# T1  launch (gz_gui:=false)  →  ros2 run agibot_x2_pkg_py walk_to_shelf
# T2  (venv, RADICE del repo: serve sorting/extract_isbn.py per Google Books)
ros2 run agibot_x2_pkg library_manager_node --ros-args -p detector:=depth -p "default_sort:=''" -p plan_only:=true
# T3
ros2 run agibot_x2_pkg_py library_pipeline                              # fase 'ocr' (2026-09-17): distanza via depth -> foto da lontano -> oggetti sul tavolo (braccio automatico) -> foto vicina -> zoom OCR -> /tmp/x2_library.json
ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p phase:=full     # anche ISBN in mano per i libri non identificati
ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p use_depth:=false  # posizioni fisse (photo_x / distance) invece della distanza misurata
ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p dry_run:=true  # foto vere, prese solo pianificate (IK), niente movimento
ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p force_isbn:=true   # ignora i titoli: tutti i libri via ISBN dal tavolo
ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p skip_objects:=true  # solo la parte libri
```
Slot sul tavolo (verificati con l'IK, fascia raggiungibile y −0,37..−0,48): oggetti a `(0.03,-0.40)` e `(0.03,-0.50)` (fuori dall'inquadratura della camera), libri da fotografare a `(-0.43,-0.37)` e `(-0.17,-0.37)` (2 affiancati sotto la camera, hfov 0.9). Con più di 2 libri non identificati i restanti vengono segnalati come irrisolti (non c'è modo di ri-afferrare un libro posato di piatto). Parametri `object_slots_xy`/`book_slots_xy` (liste x,y,x,y). Se una presa fallisce la pipeline si ferma (l'oggetto potrebbe essere in mano).

Per provare il percorso "libro non identificato" senza toccare nulla: `force_isbn:=true`. Un JSON finto non serve: con `detector:=depth` il JSON lo produce il robot e i titoli sono vuoti finché non c'è SAM3/OCR, quindi i libri vanno al tavolo da soli.

## PROVA COMPLETA DELLA PIPELINE (occhi → presa → tavolo → identificazione)

Flusso sulla **scena di default** (`grasp_test`): identificazione automatica sulla foto ad alta risoluzione della `shelf_camera` (SAM3), presa automatica con `pick_test_book` (o guidata col teleop), completamento dell'identificazione sul tavolo. Servono 4 terminali.

> La vecchia variante sulla scena completa (`scene:=full`, entità `picktest_it`/`picktest_emma`, trigger `'head'` sulla camera in testa) resta identica nei comandi: aggiungi `scene:=full` al launch e `-p scene:=full` agli script, e usa i nomi `picktest_*`.

**T1 — simulazione**
```
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py gz_gui:=false
```
Dal 2026-09-13 il robot **nasce 1,5 m più indietro** (`walk_distance`, default 1.5; `walk_distance:=0` = nasce già davanti allo scaffale). `finger_mu:=15` ripristina il vecchio attrito delle dita (test A/B). Le entità vengono spawnate ~2 s dopo i controller, con i publisher di detach già attivi (5 Hz per 30 s). Aspetta che `ros2 control list_controllers` mostri i 7 controller + broadcaster `active`, poi verifica a schermo: 4 libri con i dorsi a filo, portapenne e **mappamondo** accanto, tutti sul ripiano alto. Se uno spawner muore per timeout, vedi la sezione "SE LO SPAWNER MUORE".

**T1b — camminata fino allo scaffale** (base mobile cinematica + gambe, vedi Obsidian Gazebo.md "Camminata")
```
ros2 run agibot_x2_pkg_py walk_to_shelf                 # ~4 s simulati; le braccia oscillano
ros2 run agibot_x2_pkg_py walk_to_shelf --ros-args -p arm_swing:=false   # braccia ferme
```
Log atteso: `Camminata: base_x 0.00 -> 1.50 m … (~6 passi)` e alla fine `Arrivato: base_x = 1.500 m`. Se è già arrivato non fa nulla. **Va fatto prima della presa**: `pick_test_book` presuppone il robot nella posa di lavoro. Il nodo esce con errore se `base_controller`/`legs_controller` non sono attivi (URDF/controller vecchi → ricompila `agibot_x2_pkg` e rilancia).

**T2 — nodo pipeline** (venv obbligatorio; SAM3)
```
source .venv/bin/activate
source install/setup.bash
ros2 run agibot_x2_pkg library_manager_node --ros-args -p detector:=sam3 -p "default_sort:=''" -p plan_only:=true
```
`default_sort:=''` (con quelle virgolette esatte: la shell altrimenti manda un valore vuoto che rcl non accetta) evita che dopo l'analisi parta da sola la coreografia automatica; `plan_only:=true` fa sì che un comando di ordinamento produca **solo il JSON del piano** (`/tmp/x2_sort_plan.json`, stato `planned`) senza muovere il robot — togli il parametro quando vuoi l'esecuzione. Attendi `LibraryManagerNode pronto.` — il caricamento di SAM3 richiede minuti la **prima** volta: lascia il nodo acceso fra una prova e l'altra invece di rilanciarlo.

**T3 — identificazione dalla foto dello scaffale** (shelf_camera 960×720)

1. Controlla in `rqt_image_view` che `/shelf_camera/image` inquadri i 4 libri, poi:
```
ros2 topic pub -1 /library_manager/trigger std_msgs/String "data: 'shelf'"
```
2. In T2 passa detection → colori → OCR. Risultati:
```
ros2 topic echo --full-length /library_manager/detections --once
```
(JSON latched: un elemento per oggetto con `id`, `bbox`, `color`, `title`, `author`, `shelf_row`, `shelf_slot`; senza `--full-length` la stringa viene troncata. Finché non rifai il trigger, resta l'**ultimo** risultato, anche di run vecchi). Stesso contenuto in `/tmp/x2_detections.json`, immagine annotata in `/tmp/x2_detections.jpg`. Segnati gli `obj_id`.
3. **Piano di ordinamento** (dallo stesso terminale; con `plan_only:=true` solo il JSON, niente movimenti):
```
ros2 topic pub -1 /library_manager/command std_msgs/String "data: 'ordina per autore'"
cat /tmp/x2_sort_plan.json
```
Criteri: *colore*, *titolo*, *autore*, *dimensione*; aggiungi *decrescente* per invertire. `insertion_order` è l'ordine finale dei libri (i mancanti/`N/A` sempre in coda); `objects_to_table` le decorazioni da parcheggiare. Si può ripetere con criteri diversi senza rifare la foto.

**T4 — presa automatica** (braccio destro, IK)
```
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=it    # hunger | it | ballata | alba | pen | globe
```
Sequenza da sola: apre, si avvicina, chiude sullo spessore del libro, attach, sfila, ruota la vita verso il tavolo, posa, detach, torna a casa. Con `-p dry_run:=true` solo la verifica IK senza movimento. In alternativa presa guidata: `ros2 run agibot_x2_pkg_py pick_place_teleop` (tasti `1..6` per il bersaglio, `b` afferra, `n` rilascia — vedi sezione teleop).

**T5 — completa l'identificazione dal tavolo** (table_camera 640×480, da vicino):
```
ros2 topic pub -1 /library_manager/rephotograph std_msgs/String "data: ''"
```
(`data: '<obj_id>'` per un libro preciso; vuoto = primo libro con titolo/autore mancanti). In T2 cerca `Ri-identificazione libro [N] dal tavolo: title='...' author='...'`.

**Attenzione (2026-09-17)**: lancia `pick_test_book` con `head_isbn` dal venv (`source .venv/bin/activate`), altrimenti pyzbar non c'è e le foto vengono salvate ma non decodificate (il nodo prova comunque ad aggiungere il `.venv` al path). L'esito finisce in `/tmp/x2_head_isbn_<entità>.json` e, se il libro è fra le detection, in `/tmp/x2_detections.json`. Se nelle foto vedi due libri in mano, l'auto-attach ha agganciato il vicino durante l'avvicinamento: vedi Bugs.md (risolto: scatta solo a dita in chiusura).

**T5-ter — ISBN dalla testa e libro RIMESSO A POSTO** (2026-09-17, `put_back:=true`; verificato dal vivo su IT: ISBN 9788868365622 letto dal secondo scatto, libro rimesso al suo posto, vicini fermi): come T5-bis ma dopo la lettura il libro torna nel suo slot sullo scaffale invece di andare sul tavolo: percorso di uscita e di avvicinamento percorsi all'indietro, stacco e apertura delle dita nella posa di presa, poi braccio e vita a casa (passi `R1..R10`). Si può ripetere su tutti i libri senza rilanciare.
```
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=it -p head_isbn:=true -p put_back:=true
```

**T5-bis — ISBN dalla camera della TESTA** (2026-09-14, alternativa a T5, da provare): il robot, con il libro in mano e il busto verso il tavolo, porta la copertina davanti alla testa, scatta con la `head_camera` (1920×1440, a scatto): prima una foto al centro della copertina, poi solo se non legge parte bassa e cima, poi l'altra copertina e decodifica il barcode; se non legge gira il polso di 180° e prova l'altra copertina, poi posa il libro come sempre.
```
ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=it -p head_isbn:=true      # anche con -p target:=<id>
```
Log `H0..H7`: `mostra alla testa d=0.300 ... -> OK`, `H4. scatto copertina A: foto 1920x1440 salvata in /tmp/x2_head_isbn_gt_it_1.jpg`, `ISBN dalla testa: 978...`. Esito in `/tmp/x2_head_isbn_gt_it.json`. Parametri: `head_isbn_dist` (0.25), `head_yaw` (−0.35), `head_pitch` (0.20), `head_isbn_wait_s` (120). Nella pipeline: `library_pipeline -p head_isbn:=true` (ri-foto dal tavolo solo se dalla testa non legge). Scatto a mano: `ros2 topic pub -1 /head_camera/trigger std_msgs/msg/Bool "data: true"`.

**T6** — ripeti T4–T5 con gli altri libri (`-p book:=hunger|ballata|alba`).

Non ancora automatico (prossimi passi): presa direttamente dalle bbox delle detection (ora `pick_test_book` usa le pose note di `book_placer`), stringa di ordinamento da input utente, rotazione fisica copertina/retro→ISBN.

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
| `1` … `6` | bersaglio: nella scena di default `1` Hunger Games, `2` IT, `3` Ballata, `4` Mietitura, `5` portapenne, `6` mappamondo (con `-p scene:=full`: `1` IT, `2` Emma) — l'elenco è stampato all'avvio |
| `b` | **grasp automatico**: chiude le dita allo spessore del libro bersaglio (da `BOOK_CATALOG`) e dopo 3 s manda `/gripper/right/attach` con il nome del bersaglio al GraspManager — il libro segue la mano. Solo braccio destro. |
| `v` | **attach di ciò che tocco**: `/gripper/right/attach` con nome vuoto — il GraspManager incolla l'ultima entità toccata dal dito (dopo `c`). Prova del percorso a contatto puro. |
| `n` | **release**: `/gripper/right/detach` (stacca ciò che è agganciato) + apre il gripper |
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
