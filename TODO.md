# Hexapod TODO List

## Pending Tasks
- [ ] Проверить работу гироскопа DS4 (нужен ds4drv или проверить axes в /joy)

## Completed
- [x] Миграция ROS1 -> ROS2 Jazzy
- [x] Исправление baud rate Maestro (115200)
- [x] Исправление async service calls в body_kinematics
- [x] Добавление deadzone для джойстика
- [x] Исправление движения после Stand Up (gait_kinematics scale check)
- [x] Плавное движение сервоприводов (max_speed limiting)
- [x] Добавлена поддержка гироскопа DS4 (кнопка Square)
