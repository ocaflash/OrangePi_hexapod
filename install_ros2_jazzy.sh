#!/bin/bash
set -e

echo "=== Обновление системы ==="
sudo apt update && sudo apt upgrade -y

echo "=== Установка базовых инструментов ==="
sudo apt install -y build-essential cmake git curl gnupg2 lsb-release \
    python3-pip python3-venv python3-colcon-common-extensions \
    python3-rosdep python3-rosdistro python3-rospkg python3-vcstool

echo "=== Настройка rosdep ==="
sudo rosdep init || true
rosdep update

echo "=== Создание рабочей директории ROS2 Jazzy ==="
mkdir -p ~/ros2_jazzy/src
cd ~/ros2_jazzy

echo "=== Загрузка описания пакетов Jazzy ==="
curl -sSL https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos -o ros2.repos
vcs import src < ros2.repos

echo "=== Установка зависимостей через rosdep ==="
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

echo "=== Сборка ROS2 Jazzy Base ==="
colcon build --symlink-install

echo "=== Настройка окружения ==="
echo "source ~/ros2_jazzy/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "=== Установка завершена! ==="
echo "Теперь можно запускать ROS2 узлы, например:"
echo "  ros2 run demo_nodes_cpp talker"

