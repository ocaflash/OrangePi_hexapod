#!/bin/bash
# Запуск RViz с панелью управления суставами

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts.env"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

ros2 launch crab_description display_model.launch.py
