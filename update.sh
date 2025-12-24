#!/usr/bin/env bash
set -euo pipefail

TARGET_SCRIPT="$HOME/ros2_ws/src/OrangePi_hexapod/update_hexapod.sh"

if [ ! -f "$TARGET_SCRIPT" ]; then
  echo "❌ Не найден скрипт обновления: $TARGET_SCRIPT" >&2
  echo "Проверьте, что репозиторий находится в ~/ros2_ws/src/OrangePi_hexapod" >&2
  exit 1
fi

chmod +x "$TARGET_SCRIPT" 2>/dev/null || true
exec "$TARGET_SCRIPT" "$@"

