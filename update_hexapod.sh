#!/bin/bash
# Скрипт автоматического обновления и сборки OrangePi_hexapod

# Настройки
WORKSPACE=~/ros2_ws
REPO_DIR=$WORKSPACE/src/OrangePi_hexapod
DEFAULT_BRANCH="main"

echo "=== [$(date)] Автообновление OrangePi_hexapod ==="

# Переходим в каталог проекта
cd $REPO_DIR || { echo "❌ Не найден каталог $REPO_DIR"; exit 1; }

# Получаем список веток
echo "→ Получаем список веток..."
git fetch --all --prune

# Показываем доступные ветки
echo ""
echo "Доступные ветки:"
echo "----------------"
git branch -a | grep -v HEAD | sed 's/remotes\/origin\//  /' | sort -u
echo ""

# Выбор ветки
read -p "Введите имя ветки (Enter для '$DEFAULT_BRANCH'): " BRANCH
BRANCH=${BRANCH:-$DEFAULT_BRANCH}

# Проверяем существование ветки
if ! git show-ref --verify --quiet refs/heads/$BRANCH && \
   ! git show-ref --verify --quiet refs/remotes/origin/$BRANCH; then
    echo "❌ Ветка '$BRANCH' не найдена"
    exit 1
fi

# Переключаемся на ветку и обновляем
echo "→ Переключаемся на ветку '$BRANCH'..."
git checkout $BRANCH || { echo "❌ Ошибка переключения на ветку"; exit 1; }

# Сохраняем текущий коммит для сравнения
OLD_COMMIT=$(git rev-parse HEAD)

echo "→ Синхронизируем с удалённой веткой..."
git fetch origin $BRANCH
git reset --hard origin/$BRANCH || { echo "❌ Ошибка синхронизации"; exit 1; }

# Показываем статистику изменений
NEW_COMMIT=$(git rev-parse HEAD)
if [ "$OLD_COMMIT" != "$NEW_COMMIT" ]; then
    echo ""
    echo "📊 Статистика обновления:"
    git diff --stat $OLD_COMMIT $NEW_COMMIT
    echo ""
    COMMITS_COUNT=$(git rev-list --count $OLD_COMMIT..$NEW_COMMIT)
    echo "✓ Получено коммитов: $COMMITS_COUNT"
else
    echo "✓ Уже актуальная версия"
fi

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

echo "✅ Сборка завершена! (ветка: $BRANCH)"
echo ""
echo "Для запуска робота:"
echo "  $REPO_DIR/start_hexapod.sh"
