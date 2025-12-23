#!/bin/bash
# Запуск RViz с панелью управления суставами

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts.env"

source /opt/ros/jazzy/setup.bash

SETUP_CANDIDATES=(
  "$WORKSPACE/install/setup.bash"
  "$REPO_DIR/install/setup.bash"
  "$SCRIPT_DIR/install/setup.bash"
)

SETUP_FOUND=""
for candidate in "${SETUP_CANDIDATES[@]}"; do
  if [ -f "$candidate" ]; then
    SETUP_FOUND="$candidate"
    break
  fi
done

if [ -z "$SETUP_FOUND" ]; then
  echo "❌ Не найден install/setup.bash." >&2
  echo "Проверь WORKSPACE/REPO_DIR в scripts.env или собери workspace." >&2
  exit 1
fi

source "$SETUP_FOUND"

ros2 launch crab_description display_model.launch.py
