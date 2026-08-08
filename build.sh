#!/bin/bash
# SmartRobotics: compila il workspace e carica l'ambiente.
# Da usare con: source ~/SmartRobotics/build.sh   (oppure alias: smartbuild)
source /opt/ros/jazzy/setup.bash
cd ~/SmartRobotics
colcon build
source install/setup.bash
