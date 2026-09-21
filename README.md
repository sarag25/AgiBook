<div align="center">

# 📚 SmartRobotics

**A humanoid robot that reads a bookshelf, asks how to sort it, and moves the books with its own hands.**

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![MoveIt](https://img.shields.io/badge/MoveIt-2-2F9E7A)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)
![Streamlit](https://img.shields.io/badge/UI-Streamlit-FF4B4B?logo=streamlit&logoColor=white)
![Status](https://img.shields.io/badge/status-simulation-2F9E7A)

<table>
<tr>
<td align="center"><img src="docs/media/scaffale_prima.jpg" width="380"><br><sub><b>Before</b>: globe and pen holder on the shelf</sub></td>
<td align="center"><img src="docs/media/scaffale_dopo.jpg" width="380"><br><sub><b>After</b>: books in alphabetical order, objects on the right</sub></td>
</tr>
</table>

</div>

## What it does

The **AgiBot X2** robot (simulated in Gazebo, no GPU) stands in front of a bookcase holding four books and two objects.

1. **Looks**: photographs the shelf with the head camera, measures each book's position, thickness and height from the depth image, and reads title and ISBN (OCR + barcode + Google Books).
2. **Asks**: a phone-sized web page asks how to sort the books (title, author, year, thickness) and shows the shelf as it is and as it will be.
3. **Plans**: computes the **minimum number of grasps** and discards moves the robot cannot perform.
4. **Moves**: grasps each book with the most suitable hand, slides it out, carries it in front of the shelf and puts it back in its slot. **Books never go on the table**: they stay in the hand or on the shelf.
5. **Puts the objects back** (globe and pen holder) on the shelf, to the right of the last book.

<p align="center"><img src="docs/media/presa_passo_passo.png" width="900"></p>

## How it works

<p align="center"><img src="docs/media/architettura.png" width="800"></p>

The robot **does not perceive** the shelf and the table: it knows them as models derived from the URDFs (`scene_config.py`). It perceives the **books**. The constants that remain are explained one by one in [docs/COSTANTI.md](docs/COSTANTI.md) (in Italian).

<p align="center"><img src="docs/media/flusso_riordino.png" width="900"></p>

**Design choices**

| Choice | Why |
|---|---|
| Ordering with **A\*** over the number of grasps | With the simulation at RTF ≈ 0.05 one grasp takes 35–50 real minutes: what matters is the number of grasps, not the complexity |
| **MoveIt only for free-space segments** | Straight-line paths (approach, exit, sideways move) use our own IK and are checked sample by sample against the scene |
| **Reachability map** (`reach_map.py`) | The planner never proposes impossible moves; the executor picks the right arm without wasting minutes of IK |
| **Close on contact** | The fingers stop at the first contact (guarded move), then the real pose is verified |

## Requirements

- Windows 11 + WSL2 + Docker Desktop (or Ubuntu 24.04), **no GPU required**
- ROS 2 **Jazzy**, Gazebo **Harmonic**, MoveIt 2 (`ros-jazzy-moveit`)
- Python 3.12; for perception, a virtual environment with `pip install -r requirements.txt`
- A `.env` file in the repository root with `GOOGLE_BOOKS_API_KEY` (and `HF_TOKEN` only for SAM3). **Never commit it.**

## Quick start

In every terminal: `cd ~/SmartRobotics && source /opt/ros/jazzy/setup.bash && source install/setup.bash`

```bash
# 0 · once
colcon build

# 1 · simulation (wait ~2 minutes: "Configured and activated legs_controller")
ros2 launch agibot_x2_pkg rviz_gaz_control.launch.py gz_gui:=false rviz:=false video:=true

# 2 · MoveIt ("You can start planning now!")
ros2 launch agibot_x2_pkg moveit.launch.py

# 3 · perception, from the venv ("LibraryManagerNode pronto", ~4 minutes)
source .venv/bin/activate
ros2 run agibot_x2_pkg library_manager_node --ros-args -p detector:=depth -p "default_sort:=''" -p plan_only:=true
```

> **Do not query ROS during startup** (`ros2 control`, `ros2 topic`…): the calls compete for the controller manager services and the spawner dies. Only watch the log.

```bash
# 4 · video of every run (stop with Ctrl+C: it finalizes the mp4 files in videos/)
ros2 run agibot_x2_pkg_py record_video --ros-args -p prefix:=demo_percezione

# 5 · the robot walks to the bookcase, photographs it and puts the objects on the table (~2 real hours)
ros2 run agibot_x2_pkg_py library_pipeline --ros-args -p phase:=ocr -p skip_books:=true
```

The book file with ISBNs (`/tmp/x2_library.json`) is produced by `-p phase:=full` (books read in the hand in front of the head).
To try things right away: `cp docs/esempi/x2_library_esempio.json /tmp/x2_library.json`.

```bash
# 6 · the user page: http://localhost:8501  (the interface text is in Italian)
ros2 run agibot_x2_pkg_py sort_ui_utente

# 7 · the executor waits for the plan from the page and records its own video
ros2 run agibot_x2_pkg_py reorder_executor --wait --video-prefix demo_riordino
#     → on the page: pick the criterion, "Avvia il riordino", confirm

# 8 · once the books are in place: objects from the table back to the shelf
ros2 run agibot_x2_pkg_py reorder_executor --objects-only --video-prefix demo_oggetti
```

**Before every round**, move away the leftovers of the previous round, otherwise the robot works on stale positions:

```bash
mkdir -p /tmp/x2_prev && mv /tmp/x2_detections*.json /tmp/x2_placed_obj*.json /tmp/x2_reorder_*.json /tmp/x2_library_riordinata.json /tmp/x2_prev/ 2>/dev/null
```

**Without the robot** (page and planner only, no ROS dependency):

```bash
X2_UI_INVIO=0 PYTHONPATH=src/agibot_x2_pkg_py python -m streamlit run src/agibot_x2_pkg_py/agibot_x2_pkg_py/sort_ui_utente.py
python3 -m agibot_x2_pkg_py.reorder_planner --library docs/esempi/x2_library_esempio.json --by author
```

## Results

Run of 2026-09-21, simulation without GPU (RTF ≈ 0.05: each grasp takes 40–50 real minutes; the videos are sped up).

| Phase | Outcome | Duration | Video (`videos/`) |
|---|---|---|---|
| Shelf photo, globe and pen holder moved to the table | success | 2897 s + 2873 s | `finale_1_percezione_*` |
| Reordering of the 4 books from the web page (title A→Z, **3 moves**) | success | 3014 s + 2823 s + 3033 s | `finale_2_riordino_*` |
| Globe from table to shelf | success (in 3 resumptions, see below) | 2824 s | `finale_3_oggetti_*`, `finale_6a_mappamondo_ripresa_*` |
| Pen holder from table to shelf, final code | **success on the first attempt** | 2913 s | `finale_7_oggetti_finale_*` |

**Final result** (poses read from Gazebo): books in order along y (Alba +0.32 · Ballata +0.21 · Hunger Games 0.00 · It −0.11), globe (y −0.27) and pen holder (y −0.32) **standing upright** to the right of the last book, table empty.

<p align="center"><img src="docs/media/scaffale_dopo.jpg" width="520"><br><sub>After: books on the shelf, objects on the right, table empty</sub></p>

**What was not perfect (disclosed)**
- **Manual interventions**: after being released on the table the pen holder had fallen over; it was set upright with `gz service set_pose` (once by hand, and once together with the globe for the clean run). Since then the drop at release is 1 cm instead of 3 and the executor checks the tilt.
- **Resumptions**: the globe and the pen holder needed `pick_test_book -p resume_put_back:=true` after the path check had stopped the robot (a finger 5 mm into the wall, then 4.6 mm into "It"). The check did its job: no collision was executed. The final pen holder run (`finale_7`) needed no resumption.
- **Books deeper than their slot**: Alba (+10 cm) and Hunger Games (+7 cm) end up against the back of the shelf; the order along y is correct. Cause not established; the executor now logs `spinto_indietro_cm` (pushed-back distance) after every move. "It" moved by 5 cm while the globe was being placed.
- **Which video matches which code**: `finale_1` (release at 3 cm) and `finale_2/3` were recorded before the last fixes (1 cm drop, measured grasp offset, object slots); the delivered code is the one used for `finale_7`, verified with `cmp` between `src/` and `install/` (the only difference after that run is the rename of the `sorting/` folder to `book_identification/`, i.e. ISBN paths only, not used in these runs).

## The user page

Designed for the phone: one column, three steps, no time shown. Choices and plan come from the planner; if a book cannot be moved, the page says so. The interface text is in Italian.

<table>
<tr>
<td align="center"><img src="docs/media/ui_prima_dell_avvio.png" width="260"><br><sub>Choice and preview</sub></td>
<td align="center"><img src="docs/media/ui_conferma.png" width="260"><br><sub>Confirmation</sub></td>
<td align="center"><img src="docs/media/ui_avanzamento.png" width="260"><br><sub>Robot progress</sub></td>
</tr>
</table>

URL parameters: `?cornice=0` removes the phone frame; `?tecnico=1` shows developer details.

## Repository layout

```
SmartRobotics/
├── README.md                  this file
├── docs/
│   ├── COSTANTI.md            every constant: removed, kept, and why (Italian)
│   ├── guida_completa.md      the old README, with all the intermediate runs (Italian)
│   ├── esempi/                sample data (books with ISBN)
│   └── media/                 diagrams and images (genera_schemi.py regenerates them)
├── src/
│   ├── agibot_x2_pkg/         URDF, meshes, launch, controllers, scene_config.py, perception
│   ├── agibot_x2_pkg_py/      nodes: pick_test_book, reorder_*, reach_map, planning_scene_builder, UI
│   └── agibot_x2_moveit_config/  MoveIt configuration generated by the Setup Assistant (alternative, not used by the pipeline)
├── environment/               Blender scripts for the 3D assets
├── book_identification/       standalone book recognition pipeline (no ROS)
└── videos/                    videos of the runs (not versioned)
```

## Tests

```bash
cd src/agibot_x2_pkg_py && PYTHONPATH=. python3 -m pytest test/ -q     # planner, map, scene: 12 tests
```

## Known limitations

- **Simulation only.** The robot also reads Gazebo ground truth (model names and poses), the attach is a detachable joint and walking is kinematic.
- The first shelf is assumed in several places: another shelf or another bookcase requires another URDF and new reachability measurements.
- A book slot in the band y +0.07…+0.18 is not reachable by either arm; "It" cannot reach −0.122 and stays where it is.
- Paths between samples are checked with linear interpolation in joint space; the controller uses a spline.

## Documentation

Project notes (architecture, solved bugs, decisions) live in the Obsidian vault; here: [constants](docs/COSTANTI.md) · [historical guide](docs/guida_completa.md).

## Credits

Project by Sara with Caterina (MoveIt configuration). Inspired by the arm demo of [MOGI-ROS](https://github.com/MOGI-ROS/Week-9-10-Simple-arm). License: see [LICENSE](LICENSE).
