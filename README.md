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

#### LAUNCH RVIZ + GAZEBO:
In terminal > robot@docker-desktop:~/<ws_name>$
```
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py
```

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
| `o` | apri gripper (braccio attivo) |
| `c` | chiudi gripper (braccio attivo, grasp) |
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
