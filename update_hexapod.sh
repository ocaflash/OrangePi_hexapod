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

# На устройствах мы часто делаем `chmod +x` для удобства запуска скриптов.
# Git отслеживает executable-бит, из-за чего `git checkout` каждый раз показывает "M file".
# По умолчанию игнорируем изменения filemode в этом репозитории (можно отключить: GIT_IGNORE_FILEMODE=0).
if [ "${GIT_IGNORE_FILEMODE:-1}" = "1" ]; then
    git config core.filemode false >/dev/null 2>&1 || true
fi

# Получаем список веток
echo "→ Получаем список веток..."
# Важно: в некоторых конфигурациях git remote "origin" может быть настроен так,
# что `git fetch` подтягивает только main (ограниченный fetch refspec).
# Явно подтягиваем ВСЕ ветки, чтобы список origin/* был полным.
git fetch origin --prune '+refs/heads/*:refs/remotes/origin/*'

# Выбор ветки (интерактивный, но упрощённый)
if [ "${SKIP_BRANCH_PROMPT:-0}" != "1" ]; then
    # Prefer current branch as default; fallback to DEFAULT_BRANCH if detached.
    CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
    if [ -z "$CURRENT_BRANCH" ] || [ "$CURRENT_BRANCH" = "HEAD" ]; then
        CURRENT_BRANCH="$DEFAULT_BRANCH"
    fi

    # Build list of origin branches (short names without 'origin/').
    # IMPORTANT: remote branches may contain slashes (e.g. origin/ver-3/convert_to_ros2),
    # so we must NOT use refs/remotes/origin/* (it matches only one path segment).
    mapfile -t BRANCH_LIST < <(git for-each-ref --format='%(refname:short)' refs/remotes/origin \
        | sed 's#^origin/##' \
        | grep -vE '^(HEAD|origin)$' \
        | grep -vE '^$' \
        | sort -u)

echo ""
    echo "Доступные ветки (origin):"
    echo "------------------------"
    for i in "${!BRANCH_LIST[@]}"; do
        idx=$((i + 1))
        b="${BRANCH_LIST[$i]}"
        if [ "$b" = "$CURRENT_BRANCH" ]; then
            echo "  $idx) $b  [current]"
        else
            echo "  $idx) $b"
        fi
    done
echo ""

    read -p "Выберите ветку (Enter = '$CURRENT_BRANCH', номер или имя): " BRANCH_INPUT
    if [ -z "${BRANCH_INPUT}" ]; then
        BRANCH="$CURRENT_BRANCH"
    elif [[ "${BRANCH_INPUT}" =~ ^[0-9]+$ ]]; then
        n="${BRANCH_INPUT}"
        if [ "$n" -ge 1 ] && [ "$n" -le "${#BRANCH_LIST[@]}" ]; then
            BRANCH="${BRANCH_LIST[$((n - 1))]}"
        else
            echo "❌ Неверный номер ветки: $n"
            exit 1
        fi
    else
        BRANCH="${BRANCH_INPUT}"
    fi
else
    # Для автоматического перезапуска после обновления самого скрипта
BRANCH=${BRANCH:-$DEFAULT_BRANCH}
    echo "→ Используем ветку (без запроса): '$BRANCH'"
fi

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

# Если в апдейте обновились служебные скрипты, перезапускаем обновление новой версией.
# Иначе текущий процесс продолжит выполняться "старым" кодом до завершения.
NEW_COMMIT=$(git rev-parse HEAD)
if [ "${SELF_REEXEC:-0}" != "1" ]; then
    if git diff --name-only "$OLD_COMMIT" "$NEW_COMMIT" -- update_hexapod.sh scripts.env 2>/dev/null | grep -q .; then
        echo "🧰 Обновлены служебные скрипты. Перезапускаю обновление новой версией..."
        export SELF_REEXEC=1
        export SKIP_BRANCH_PROMPT=1
        export BRANCH
        exec "$REPO_DIR/update_hexapod.sh" "$@"
    fi
fi

# Показываем статистику изменений
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
# colcon-generated setup.* scripts may reference variables like COLCON_TRACE without defaults.
# With `set -u` (nounset) enabled this can abort the script. Temporarily disable nounset for sourcing.
set +u
source "$WORKSPACE/install/setup.bash"
set -u

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
