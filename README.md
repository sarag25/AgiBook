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
Inside folder ROS2_ContainerGUI containing bash scripts, open terminal:
```
> wsl
> ./create_container.sh ros2_gui:v0.1 <optional_work_space_name> ros2_project
```

## Workspace
#### CREATE NEW WORKSPACE
Inside container terminal:
```
> robot@docker-desktop:~$ mkdir ws_name  # if not created already
> robot@docker-desktop:~$ cd ws_name
> robot@docker-desktop:~/ws_name$ mkdir src
```
Modify CMake if needed

Creation of robot ```.urdf``` ref file, placed in ```urdf/``` dir

#### ENABLE ROS2 COMMANDS IN WS DIRECTORY:
```
> robot@docker-desktop:~/ws_name$ source /opt/ros/jazzy/setup.bash
```

#### MAKE FILES RECOGNIZABLE IN WS/SRC FOLDER:
```
> robot@docker-desktop:~/ws_name$ source install/setup.bash
```

#### COMPILE WS FOLDER STRUCTURE:

```
> robot@docker-desktop:~/ws_name$ colcon build
```
Redo at any change in workspace files

#### LAUNCH RVIZ + GAZEBO:
```
ros2 launch agibot_x2_pkg rviz_gaz.launch.py
```

## Rviz

```
> sudo apt-get update && sudo apt-get install -y ros-jazzy-joint-state-publisher-gui
> ros2 launch agibot_x2_pkg rviz.launch.py
```

## Gazebo

**Errore *[Err] [SystemLoader.cc:92] Failed to load system plugin [gz_ros2_control-system] : Could not find shared library.* **
```
> sudo apt update
> sudo apt install ros-$ROS_DISTRO-gz-ros2-control ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers
```

## If urdf file gets modified:
```
> colcon build
> source install/setup.bash
```
If an error appears, do ```> rm -rf build/agibot_x2_pkg/ install/agibot_x2_pkg/``` before.

#### FIX SYMBOL MISMATCH BY UPGRADING EVERY ROS2 PACKAGE
```
> sudo apt update
> sudo apt install --only-upgrade ros-jazzy-control-manager ros-jazzy-ros2-control ros-jazzy-gz-ros2-control ros-jazzy-rclcpp
```

