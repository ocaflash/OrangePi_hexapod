#!/bin/bash
# Скрипт запуска гексапода

WORKSPACE=~/ros2_ws

echo "=== Запуск Crab Hexapod ==="

# Подключаем окружение
source /opt/ros/jazzy/setup.bash
source $WORKSPACE/install/setup.bash

# Отключаем SHM для FastDDS (избегаем ошибок)
export RMW_FASTRTPS_USE_SHM=0

# Запуск всех нод через launch файл
echo "→ Запускаем ноды..."
ros2 launch crab_description hexapod.launch.py
