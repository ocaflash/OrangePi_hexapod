#!/bin/bash
# Установка hexapod как systemd сервиса

# Определяем директорию скрипта
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts.env"
SERVICE_FILE="$SCRIPT_DIR/hexapod.service"
SERVICE_NAME="hexapod"

echo "=== Установка сервиса $SERVICE_NAME ==="

# Проверяем наличие файла сервиса
if [ ! -f "$SERVICE_FILE" ]; then
    echo "❌ Файл $SERVICE_FILE не найден"
    exit 1
fi

# Останавливаем сервис если запущен
echo "→ Останавливаем существующий сервис..."
sudo systemctl stop $SERVICE_NAME 2>/dev/null

# Копируем файл сервиса
echo "→ Копируем файл сервиса..."
sudo cp $SERVICE_FILE /etc/systemd/system/

# Перезагружаем systemd
echo "→ Перезагружаем systemd..."
sudo systemctl daemon-reload

# Включаем автозапуск
echo "→ Включаем автозапуск..."
sudo systemctl enable $SERVICE_NAME

echo ""
echo "✅ Сервис установлен!"
echo ""
echo "Команды управления:"
echo "  sudo systemctl start hexapod    - запустить"
echo "  sudo systemctl stop hexapod     - остановить"
echo "  sudo systemctl restart hexapod  - перезапустить"
echo "  sudo systemctl status hexapod   - статус"
echo "  journalctl -u hexapod -f        - логи в реальном времени"
echo ""
echo "Для отключения автозапуска:"
echo "  sudo systemctl disable hexapod"
