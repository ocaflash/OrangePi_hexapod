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
colcon build || { echo "❌ Ошибка сборки"; exit 1; }

# Подключаем окружение
echo "→ Активируем окружение..."
source $WORKSPACE/install/setup.bash

# Делаем скрипт запуска исполняемым
chmod +x $REPO_DIR/start_hexapod.sh

echo "✅ Сборка завершена!"
echo ""
echo "Для запуска робота:"
echo "  $REPO_DIR/start_hexapod.sh"