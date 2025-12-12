#!/bin/bash
# Скрипт автоматического обновления и сборки OrangePi_hexapod

# Настройки
WORKSPACE=~/ros2_ws
REPO_DIR=$WORKSPACE/src/OrangePi_hexapod

echo "=== [$(date)] Автообновление OrangePi_hexapod ==="

# Переходим в каталог проекта
cd $REPO_DIR || { echo "❌ Не найден каталог $REPO_DIR"; exit 1; }

# Обновляем репозиторий
echo "→ Выполняем git pull..."
git pull origin main || { echo "❌ Ошибка git pull"; exit 1; }

# Переходим в workspace
cd $WORKSPACE

# Сборка пакетов
echo "→ Запускаем colcon build..."
colcon build --packages-up-to crab_imu crab_body_kinematics crab_description crab_leg_kinematics crab_gait || { echo "❌ Ошибка сборки"; exit 1; }

# Подключаем окружение
echo "→ Активируем окружение..."
source $WORKSPACE/install/setup.bash

echo "✅ Готово! Запускаем ноды..."

# Запуск нод в фоне
ros2 run crab_imu imu_control &
ros2 run crab_leg_kinematics leg_ik_service &
ros2 run crab_body_kinematics body_kinematics &

echo "Ноды запущены. Для остановки: pkill -f 'ros2 run'"