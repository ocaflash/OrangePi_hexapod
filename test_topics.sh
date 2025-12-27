#!/bin/bash
# Скрипт для диагностики проблем с джойстиком и походкой

echo "=== Проверка топиков ==="
echo ""

echo "1. Проверка /joy (данные от джойстика):"
timeout 2 ros2 topic echo /joy --once 2>/dev/null || echo "   НЕТ ДАННЫХ - джойстик не подключен или joy_node не работает"
echo ""

echo "2. Проверка /teleop/gait_control (команды походки):"
timeout 2 ros2 topic echo /teleop/gait_control --once 2>/dev/null || echo "   НЕТ ДАННЫХ"
echo ""

echo "3. Проверка /teleop/body_command (команды тела):"
timeout 2 ros2 topic echo /teleop/body_command --once 2>/dev/null || echo "   НЕТ ДАННЫХ"
echo ""

echo "4. Проверка joints_to_controller (углы суставов):"
timeout 2 ros2 topic echo /joints_to_controller --once 2>/dev/null || echo "   НЕТ ДАННЫХ"
echo ""

echo "=== Список активных нод ==="
ros2 node list
echo ""

echo "=== Список топиков ==="
ros2 topic list
echo ""

echo "=== Проверка устройств джойстика ==="
ls -la /dev/input/js* 2>/dev/null || echo "Джойстики не найдены в /dev/input/"
echo ""

echo "=== Готово ==="
