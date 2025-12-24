#!/bin/bash
# Запуск демо-ходьбы без джойстика + RViz

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts.env"

source /opt/ros/jazzy/setup.bash

# Отключаем SHM для FastDDS (избегаем ошибок в некоторых окружениях)
export RMW_FASTRTPS_USE_SHM=0

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

colcon build --packages-select \
  crab_msgs \
  crab_description \
  crab_leg_kinematics \
  crab_body_kinematics \
  crab_gait \
  crab_joint_publisher
source "$SETUP_FOUND"

ros2 launch crab_description demo_gait.launch.py use_primitives:="$USE_PRIMITIVES"
