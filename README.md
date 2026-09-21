<div align="center">

# 🤖 Agibook: the Book-Sorting Librarian 📚

**A humanoid robot that analize a bookshelf, asks the user how to reorder it, and automtically moves the books in the correct spot.**

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![MoveIt](https://img.shields.io/badge/MoveIt-2-2F9E7A)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)
![Streamlit](https://img.shields.io/badge/UI-Streamlit-FF4B4B?logo=streamlit&logoColor=white)
![Status](https://img.shields.io/badge/status-simulation-2F9E7A)

<table>
<tr>
<td align="center"><img src="images/begin.png" width="380"><br><sub><b>Before</b>: 2 objects and unordered 4 books on the first shelf</sub></td>
<td align="center"><img src="images/end.png" width="380"><br><sub><b>After</b>: books ordered and objects on the right of the last book</sub></td>
</tr>
</table>

</div>

## What it does

The **AgiBot X2** robot (simulated in Gazebo) stands in front of a bookcase holding 4 books and 2 objects.

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="images/pipeline_dark.png">
    <img src="images/pipeline.png" width="900">
  </picture>
</p>

1. **Looks**: photographs the shelf with the head camera.
2. **Detects and classifies**: finds everything on the shelf, tells the books from the objects, and measures each book's position, thickness and height from the depth image.
3. **Clears the objects**: takes the objects (globe and pen holder) off the shelf and puts them on the table, so they don't get in the way.
4. **Reads the books**: takes a closer photo of the spines and reads the titles (OCR). A book that can't be identified is picked up, rotated in front of the head camera and identified from its ISBN (barcode + Google Books).
5. **Asks**: a phone-sized web page asks how to sort the books (title, author, year, thickness) and shows the shelf as it is and as it will be.
6. **Plans**: computes the **minimum number of grasps** and discards moves the robot cannot perform.
7. **Reorders**: grasps each book with the most suitable hand, slides it out, carries it in front of the shelf and puts it back in its slot. **Books never go on the table**: they stay in the hand or on the shelf.
8. **Puts the objects back**: takes them from the table to the shelf, to the right of the last book.

## Requirements

- Windows 11 + WSL2 + Docker Desktop (or Ubuntu 24.04), **no GPU required**
- ROS 2 **Jazzy**, Gazebo **Harmonic**, MoveIt 2 (`ros-jazzy-moveit`)
- Python 3.12; for perception, a virtual environment with `pip install -r requirements.txt`
- A `.env` file in the repository root with `GOOGLE_BOOKS_API_KEY` (and `HF_TOKEN` for SAM3 and QwenV3-VL).

## Setup

### Docker

###### image : ros2_gui [ TAG: v0.1 ] 
###### container : ros2_project

#### CREATE CONTAINER
Inside folder ROS2_ContainerGUI containing bash scripts, open in terminal:
```
wsl
./create_container.sh ros2_gui:v0.1 <optional_work_space_name> ros2_project
```

### Workspace
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

## Run everything

Every command below goes in **its own terminal**, and in each of them start with:

```bash
cd ~/SmartRobotics && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

**Before the first run** compile the workspace (`colcon build`). **Before every run** check that nothing is left from the previous one (old bridges and nodes give wrong results) and move away its files:

```bash
ps aux | grep -E "gz sim|move_group|planning_scene|relay_|parameter_bridge|gripper_controller"   # close what you find, by PID
mkdir -p /tmp/x2_prev && mv /tmp/x2_detections*.json /tmp/x2_placed_obj*.json /tmp/x2_reorder_*.json /tmp/x2_library_riordinata.json /tmp/x2_prev/ 2>/dev/null
```

### 1. Perception and objects

**Terminal 1 - Simulation.** Wait about 2 minutes, until the log shows `Configured and activated legs_controller`.

```bash
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py gz_gui:=false rviz:=false video:=true
```

> [!WARNING]
> :no_entry: While the simulation is starting don't use any `ros2` command (`ros2 topic`, `ros2 control`...) in the other terminals: they compete with the controllers and the launch fails. Only watch the log.

**Terminal 2 - MoveIt.** Wait for `You can start planning now!`.

```bash
ros2 launch agibot_x2_pkg moveit.launch.py
```

**Terminal 3 - Perception.** It needs the virtual environment. Wait for `LibraryManagerNode pronto` (about 4 minutes).

```bash
source .venv/bin/activate
ros2 run agibot_x2_pkg library_manager_node --ros-args -p detector:=depth -p "default_sort:=''" -p plan_only:=true
```

**Terminal 4 - Video.** Start it before the pipeline. Stop it with `Ctrl+C` (never kill it) about 30 seconds after the pipeline ends: the `.mp4` files are saved in *videos/*.

```bash
ros2 run agibot_x2_pkg_py record_video --ros-args -p prefix:=demo_perception
```

**Terminal 5 - Pipeline.** The robot photographs the shelf and puts the objects on the table.

```bash
ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p phase:=ocr -p skip_books:=true
```

> [!NOTE]
> :books: The file with the ISBNs of the books (`/tmp/x2_library.json`) is produced with `-p phase:=full`. To try the next steps right away, copy the example: `cp docs/esempi/x2_library_esempio.json /tmp/x2_library.json`.

### 2. Reordering

Keep terminals 1, 2 and 3 running. When the pipeline has ended and the video is stopped:

**Terminal 4 - User page.** Open http://localhost:8501 in the browser (the text of the page is in Italian). Use the Python of the virtual environment: `ros2 run agibot_x2_pkg_py sort_ui_utente` would use the system Python, which doesn't have Streamlit. The first start takes about 30 seconds.

```bash
source .venv/bin/activate
python -m streamlit run src/agibot_x2_pkg_py/agibot_x2_pkg_py/sort_ui_utente.py \
    --server.port 8501 --server.address 0.0.0.0 --server.headless true \
    --browser.gatherUsageStats false --theme.primaryColor "#2f9e7a"
```

**Terminal 5 - Executor.** It waits for the plan from the page and records its own video. On the page choose how to sort the books, click *Avvia il riordino* and confirm.

```bash
ros2 run agibot_x2_pkg_py reorder_executor --wait --video-prefix demo_reordering
```

### 3. Objects back on the shelf

When the books are in place, in terminal 5 (after the executor has ended):

```bash
ros2 run agibot_x2_pkg_py reorder_executor --objects-only --video-prefix demo_objects
```

To stop everything close each terminal with `Ctrl+C`, terminal 1 last.

## The user page

Designed for the phone: one column, three steps, no time shown. Choices and plan come from the planner; if a book cannot be moved, the page says so. The interface text is in Italian.

<table>
<tr>
<td align="center"><img src="images/user_page/ui_before.png" width="260"><br><sub>Choice and preview</sub></td>
<td align="center"><img src="images/user_page/ui_confirmation.png" width="260"><br><sub>Confirmation</sub></td>
<td align="center"><img src="images/user_page/ui_progress.png" width="260"><br><sub>Robot progress</sub></td>
</tr>
</table>

## Robot

The robot is the humanoid **AgiBot X2**, simulated in Gazebo. It stands in front of the bookshelf, looks at the books with the head camera and moves them with its two hands.

### Degrees of freedom

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="images/dof_dark.png">
    <img src="images/dof.png" width="520">
  </picture>
</p>

| Part | DoF | Joints |
|---|---|---|
| Neck | 2 | yaw, pitch |
| Shoulder (each arm) | 3 | pitch, roll, yaw |
| Elbow (each arm) | 1 | elbow |
| Wrist (each arm) | 1 | yaw |
| Finger (each of the 2 fingers of a gripper) | 1 | linear opening |
| Waist | 3 | yaw, pitch, roll |
| Leg (each leg) | 6 | hip pitch/roll/yaw, knee, ankle pitch/roll |

Each arm is controlled with 7 joints: the 5 of the arm and the 2 fingers of the gripper.

> [!NOTE]
> :walking: In the simulation the pelvis is fixed to the world through 4 virtual joints (x, y, z and yaw): the robot doesn't have to keep its balance, and it is moved in front of the bookshelf while the legs are animated.

### Cameras and sensors

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="images/cameras_and_sensors_dark.png">
    <img src="images/cameras_and_sensors.png" width="380">
  </picture>
</p>

- **RGB-D camera** (head): takes the photo of the shelf when it is asked (color and depth). The color image is used to read titles and ISBN, the depth image to measure position, thickness and height of every book.
- **TCP camera** (hands): a small camera on each hand, used to film the grasp.
- **Contact sensor** (fingers): tells which object the fingers are touching. The robot uses it to know when a book is really between the fingers.

## Gazebo

### Environment

The robot doesn't work on the whole library of Blender, but on a simpler scene: a bookshelf, a table and only the **top shelf** filled, the only one that the arm and the head camera can reach.

<table align="center">
<tr>
<td align="center"><img src="images/library/env_library_front.png" width="240"><br><sub>Render of the top shelf with its books and objects</sub></td>
<td align="center"><img src="images/table/table_3d.png" width="340"><br><sub>The empty table</sub></td>
</tr>
</table>

- **Bookshelf and table**: static models ([bookshelf.urdf](src/agibot_x2_pkg/urdf/bookshelf.urdf) and [table.urdf](src/agibot_x2_pkg/urdf/table.urdf)) that use the `.glb` meshes made in Blender for the look and simple boxes for the collisions. The table is empty at the beginning.
- **Books and objects**: 4 books and 2 objects (a pen holder and a desk globe), defined in `GRASP_TEST_ENTITIES` of [book_placer.py](src/agibot_x2_pkg/agibot_x2_pkg/book_placer.py). Unlike the shelf, they are free bodies with mass and collisions, so the robot can grasp and move them.
- **Positions**: the bookshelf is 40 cm in front of the robot and the table is beside it. The poses are set in [scene_config.py](src/agibot_x2_pkg/agibot_x2_pkg/scene_config.py).

The environment starts with the simulation launch, without arguments:

```
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py
```

> [!NOTE]
> :twisted_rightwards_arrows: The other scenes are chosen with the `scene` argument: `scene:=ocr_test` puts all the 15 books on the top shelf, `scene:=full` loads the whole library of Blender as a single painted model, with books that can't be grasped.

## Blender (version 5.1.2)

The 3D assets (books, decorations, bookshelf and table) are already in *src/agibot_x2_pkg/meshes/*. Run these scripts only if you want to create them again.

All the scripts of this section are run in the same way:

1. In Blender, select the **Scripting** tab.
2. Click on the :open_file_folder: to load the script. It'll appear on the text editor.
3. Click the :arrow_forward: to run the script.

> [!IMPORTANT]
> :file_folder: Load the script from its file, don't copy its text: the scripts use their own path to find the images, the other scripts and the output folders.

> [!TIP]
> :eyes: If you want to see the result, switch the viewport to **Material Preview**.

### Create the books with texture (cover, retro and spine)

Script: [create_books.py](environment/books/create_books.py). The books in the `.glb` format will be saved in the *src/agibot_x2_pkg/meshes/books/* folder.

<p align="center"><img src="images/books/books_stack.png" width="260"></p>

> [!NOTE]
> :link: The books' texture images used in the script are downloadable here [`books_textures.zip`](https://drive.google.com/file/d/12ZqwQXHbO_CrZTKkXI9Myd3cvk0yf5Lq/view?usp=sharing). You need to extract the content of the `.zip` folder in the *books* one (the same of the script [create_books.py](books/create_books.py)).

### Create the desk decorations

Script: [create_desk_decorations.py](environment/desk_decorations/create_desk_decorations.py). The six decorations (`pen_holder`, `paperweight_ball`, `desk_globe`, `potted_plant`, `coffee_mug` and `coaster`) in the `.glb` format will be saved in the *src/agibot_x2_pkg/meshes/desk_decorations/* folder. In the Blender scene they are lined up 0.25 m apart.

<p align="center"><img src="images/objects/objects_pair.png" width="260"></p>

> [!NOTE]
> :package: You don't need to download anything: the textures (noise, marble, wood and continents) are generated by the script and embedded in the `.glb` files. Every decoration has to fit in the gripper, so the script prints a warning in the console if its width is bigger than `GRIPPER_MAX_OPENING` (8 cm, a conservative limit: the gripper opens from 3 to about 10 cm).

### Create the bookshelf

Script: [create_bookshelf.py](environment/bookshelf/create_bookshelf.py). The wooden bookshelf in the `.glb` format will be saved as *src/agibot_x2_pkg/meshes/bookshelf.glb*.

The bookshelf has 4 shelves and it is 0.800 x 0.300 x 1.350 m (WxDxH), built with 22 mm boards. The back panel is at -Y and the front opening towards +Y. The books are placed on the shelf surfaces:

| Shelf (from the bottom) | Surface height (z) | Free height |
|---|---|---|
| 1 | 0.022 m | 0.305 m |
| 2 | 0.349 m | 0.300 m |
| 3 | 0.671 m | 0.300 m |
| 4 (top) | 0.993 m | 0.335 m |

> [!NOTE]
> :warning: These measures must match `urdf/bookshelf.urdf` and [book_placer.py](src/agibot_x2_pkg/agibot_x2_pkg/book_placer.py): if you change the boards or the free heights, update them too. Unlike the books, the bookshelf is exported **Z-up**, so no rotation is needed to use it in the URDF or in RViz.

### Create the table

Script: [create_table.py](environment/table/create_table.py). The table in the `.glb` format will be saved as *src/agibot_x2_pkg/meshes/table.glb*.

The table is a wooden top (1.20 x 0.90 m, 3 cm thick) with 4 legs (4 cm wide, set back 5 cm from the edges). The top surface is at 0.50 m, the height the arm of the robot can reach without extreme extension.

> [!NOTE]
> :straight_ruler: Width, depth and height are parameters of `build_table()`. When the table is built inside the full scene ([create_full_scene.py](environment/create_full_scene.py)) the top is at 0.75 m, near the shoulder height of the robot.

### Populate the library

Script: [create_scene.py](environment/create_scene.py). It places the books and the decorations on the shelves of the bookshelf, in one of the predefined layouts (set `ACTIVE_CONFIG` before running it), and saves the scene as *environment/library_scene.blend*. To get the library together with the table, use [create_full_scene.py](environment/create_full_scene.py) instead.

> [!NOTE]
> :link: The books are created again from their textures, so the content of [`books_textures.zip`](https://drive.google.com/file/d/12ZqwQXHbO_CrZTKkXI9Myd3cvk0yf5Lq/view?usp=sharing) must be in the *environment/books* folder.

> [!NOTE]
> :robot: The library full of books is not the environment used by the robot: see [Gazebo > Environment](#environment).