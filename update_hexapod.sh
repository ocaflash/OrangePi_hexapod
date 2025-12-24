#!/bin/bash
# Скрипт автоматического обновления и сборки OrangePi_hexapod

set -euo pipefail

# Настройки
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts.env"
DEFAULT_BRANCH="main"

echo "=== [$(date)] Автообновление OrangePi_hexapod ==="

# Переходим в каталог проекта
cd "$REPO_DIR" || { echo "❌ Не найден каталог $REPO_DIR"; exit 1; }

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
git checkout "$BRANCH" || { echo "❌ Ошибка переключения на ветку"; exit 1; }

# Сохраняем текущий коммит для сравнения
OLD_COMMIT=$(git rev-parse HEAD)

echo "→ Синхронизируем с удалённой веткой..."
git fetch origin "$BRANCH"
git reset --hard "origin/$BRANCH" || { echo "❌ Ошибка синхронизации"; exit 1; }

# Показываем статистику изменений
NEW_COMMIT=$(git rev-parse HEAD)
if [ "$OLD_COMMIT" != "$NEW_COMMIT" ]; then
    echo ""
    echo "📊 Статистика обновления:"
    # Основные изменения (без служебных скриптов)
    MAIN_STATS=$(git diff --stat "$OLD_COMMIT" "$NEW_COMMIT" -- . \
        ':(exclude)update_hexapod.sh' \
        ':(exclude)scripts.env' || true)
    if [ -n "$MAIN_STATS" ]; then
        echo "$MAIN_STATS"
    else
        echo "(нет изменений кроме служебных скриптов)"
    fi

    # Отдельно показываем изменения служебных скриптов, если они есть
    SCRIPT_STATS=$(git diff --stat "$OLD_COMMIT" "$NEW_COMMIT" -- update_hexapod.sh scripts.env 2>/dev/null || true)
    if [ -n "$SCRIPT_STATS" ]; then
        echo ""
        echo "🧰 Изменения служебных скриптов:"
        echo "$SCRIPT_STATS"
    fi
    echo ""
    COMMITS_COUNT=$(git rev-list --count "$OLD_COMMIT..$NEW_COMMIT")
    echo "✓ Получено коммитов: $COMMITS_COUNT"
else
    echo "✓ Уже актуальная версия"
fi

# Переходим в workspace
cd "$WORKSPACE"

# Сборка пакетов
echo "→ Запускаем colcon build..."
if ! command -v colcon >/dev/null 2>&1; then
    echo "❌ colcon не найден. Установите colcon (обычно пакет 'python3-colcon-common-extensions')."
    exit 1
fi

# Иногда после смены режима установки (copy -> --symlink-install) остаются директории там,
# где ament_cmake_python ожидает создать симлинк, и сборка падает сообщением:
#   "failed to create symbolic link ... because existing path cannot be removed: Is a directory"
# Это безопасно лечится удалением таких директорий внутри build/ перед сборкой.
COLCON_FIX_PY_SYMLINKS="${COLCON_FIX_PY_SYMLINKS:-1}"
if [ "$COLCON_FIX_PY_SYMLINKS" = "1" ]; then
    echo "→ Предочистка build/*/ament_cmake_python/*/* (защита от конфликтов symlink-install)..."
    while IFS= read -r pkg; do
        [ -n "$pkg" ] || continue
        p="$WORKSPACE/build/$pkg/ament_cmake_python/$pkg/$pkg"
        if [ -d "$p" ] && [ ! -L "$p" ]; then
            echo "  - удаляю конфликтующую директорию: $p"
            rm -rf "$p"
        fi
    done < <(colcon list --names-only 2>/dev/null || true)
fi

# Полная очистка workspace по запросу (долго, но решает любые проблемы с кэшем)
# Пример: CLEAN_BUILD=1 ./update_hexapod.sh
if [ "${CLEAN_BUILD:-0}" = "1" ]; then
    echo "→ CLEAN_BUILD=1: удаляю $WORKSPACE/build $WORKSPACE/install $WORKSPACE/log ..."
    rm -rf "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"
fi

# Heuristic: на слабых платах сборка часто 'зависает' из-за swap thrash.
# Ограничиваем параллелизм и включаем прямой вывод, чтобы было видно реальный прогресс компиляции.
CPU_CORES=1
if command -v nproc >/dev/null 2>&1; then
    CPU_CORES=$(nproc)
fi

MEM_GB=1
if [ -r /proc/meminfo ]; then
    MEM_KB=$(awk '/MemTotal:/ {print $2}' /proc/meminfo)
    MEM_GB=$(( (MEM_KB + 1048575) / 1048576 ))
fi

# Можно переопределить вручную: COLCON_WORKERS=1 ./update_hexapod.sh
COLCON_WORKERS="${COLCON_WORKERS:-}"
if [ -z "$COLCON_WORKERS" ]; then
    # ~1 worker на ~1.5GB RAM, но не больше числа ядер
    if [ "$MEM_GB" -ge 6 ]; then
        COLCON_WORKERS=4
    elif [ "$MEM_GB" -ge 4 ]; then
        COLCON_WORKERS=3
    elif [ "$MEM_GB" -ge 3 ]; then
        COLCON_WORKERS=2
    else
        COLCON_WORKERS=1
    fi
    if [ "$COLCON_WORKERS" -gt "$CPU_CORES" ]; then
        COLCON_WORKERS="$CPU_CORES"
    fi
fi

export CMAKE_BUILD_PARALLEL_LEVEL="$COLCON_WORKERS"
export MAKEFLAGS="-j$COLCON_WORKERS"

echo "→ Параллелизм сборки: workers=$COLCON_WORKERS (cpu=$CPU_CORES, mem≈${MEM_GB}GB)"
echo "→ Логи colcon: $WORKSPACE/log/latest_build (и соседние директории)"

colcon build \
  --symlink-install \
  --parallel-workers "$COLCON_WORKERS" \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF \
  || { echo "❌ Ошибка сборки"; exit 1; }

# Подключаем окружение
echo "→ Активируем окружение..."
source "$WORKSPACE/install/setup.bash"

# Делаем скрипты исполняемыми
chmod +x "$REPO_DIR/start_hexapod.sh"
chmod +x "$REPO_DIR/install_service.sh" 2>/dev/null
chmod +x "$REPO_DIR/setup.sh" 2>/dev/null

# Перезапускаем сервис если он установлен и активен
if systemctl is-active --quiet hexapod; then
    echo "→ Перезапускаем сервис hexapod..."
    sudo systemctl restart hexapod
    echo "✓ Сервис перезапущен"
fi

echo "✅ Сборка завершена! (ветка: $BRANCH)"
echo ""
if systemctl is-enabled --quiet hexapod 2>/dev/null; then
    echo "Сервис hexapod установлен. Управление:"
    echo "  sudo systemctl status hexapod   - статус"
    echo "  sudo systemctl restart hexapod  - перезапуск"
    echo "  journalctl -u hexapod -f        - логи"
else
    echo "Для запуска робота:"
    echo "  $REPO_DIR/start_hexapod.sh"
    echo ""
    echo "Для установки как сервис:"
    echo "  $REPO_DIR/install_service.sh"
fi
