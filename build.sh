#!/bin/bash
# AgiBook: compile the workspace and load it.
# Usage (from anywhere): source <path-to-repo>/build.sh
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$REPO_DIR" || return 1
colcon build --symlink-install
source install/setup.bash