#!/bin/bash
# Запуск RViz и тестового движения суставов

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

colcon build --packages-select crab_description
source "$SETUP_FOUND"

# Запуск robot_state_publisher + RViz
ros2 launch crab_description robot.launch.py use_primitives:="$USE_PRIMITIVES" &
LAUNCH_PID=$!

rviz2 -d "$SCRIPT_DIR/src/crab_description/rviz/urdf.rviz" &
RVIZ_PID=$!

# Публикация тестового движения
python3 "$SCRIPT_DIR/demo_motion.py" &
DEMO_PID=$!

trap 'kill $DEMO_PID $RVIZ_PID $LAUNCH_PID 2>/dev/null' EXIT
wait
