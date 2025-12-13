#!/bin/bash
# Скрипт запуска гексапода

WORKSPACE=~/ros2_ws

echo "=== Запуск Crab Hexapod ==="

# Останавливаем предыдущий запуск если есть
echo "→ Останавливаем предыдущие процессы..."
pkill -9 -f "ros2 launch crab_description" 2>/dev/null
pkill -9 -f "body_kinematics" 2>/dev/null
pkill -9 -f "leg_ik_service" 2>/dev/null
pkill -9 -f "gait_kinematics" 2>/dev/null
pkill -9 -f "servo_node" 2>/dev/null
pkill -9 -f "imu_control" 2>/dev/null
pkill -9 -f "teleop_joy" 2>/dev/null
pkill -9 -f "joint_publisher" 2>/dev/null
pkill -9 -f "joy_node" 2>/dev/null
sleep 1

# Подключаем окружение
source /opt/ros/jazzy/setup.bash
source $WORKSPACE/install/setup.bash

# Отключаем SHM для FastDDS (избегаем ошибок)
export RMW_FASTRTPS_USE_SHM=0

# Запуск всех нод через launch файл
echo "→ Запускаем ноды..."
ros2 launch crab_description hexapod.launch.py
