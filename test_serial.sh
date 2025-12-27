#!/bin/bash
# Простой тест отправки команд на Maestro через /dev/ttyS5

PORT="/dev/ttyS5"
BAUD=115200

echo "=== Тест связи с Maestro ==="
echo "Порт: $PORT"
echo "Скорость: $BAUD"

# Проверяем существование порта
if [ ! -e "$PORT" ]; then
    echo "ОШИБКА: Порт $PORT не существует!"
    exit 1
fi

# Настраиваем порт
stty -F $PORT $BAUD cs8 -cstopb -parenb raw -echo

echo "Порт настроен"

# Отправляем команду MiniSSC для канала 0: нейтраль (127)
# Формат: 0xFF, channel, position
echo "Отправляю MiniSSC команду: канал 0, позиция 127 (нейтраль)"
printf '\xff\x00\x7f' > $PORT

sleep 0.5

echo "Отправляю MiniSSC команду: канал 0, позиция 60"
printf '\xff\x00\x3c' > $PORT

sleep 0.5

echo "Отправляю MiniSSC команду: канал 0, позиция 194"
printf '\xff\x00\xc2' > $PORT

sleep 0.5

echo "Отправляю MiniSSC команду: канал 0, позиция 127 (нейтраль)"
printf '\xff\x00\x7f' > $PORT

echo ""
echo "=== Тест завершён ==="
echo "Если сервопривод на канале 0 двигался - связь работает!"
echo "Если нет - проверьте:"
echo "  1. Правильность подключения TX/RX"
echo "  2. Настройки Maestro (Serial mode, baud rate)"
echo "  3. Питание сервоприводов"
