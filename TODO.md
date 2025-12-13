# Hexapod TODO List

## Pending Tasks
- [ ] Калибровка гироскопа DS4 (подобрать коэффициенты)

## Completed
- [x] Миграция ROS1 -> ROS2 Jazzy
- [x] Исправление baud rate Maestro (115200)
- [x] Исправление async service calls в body_kinematics
- [x] Добавление deadzone для джойстика
- [x] Исправление движения после Stand Up (gait_kinematics scale check)
- [x] Плавное движение сервоприводов (max_speed limiting)
- [x] Интегрированный ds4_imu_publisher для чтения гироскопа DS4 через evdev
- [x] Управление наклоном через гироскоп DS4 (кнопка Square)

## Controls
- OPTIONS: Stand up / Seat down
- L1 + стики: Roll/Pitch/Yaw через стики
- R1 + стики: Смещение X/Y/Z
- Square (зажать): Управление наклоном через гироскоп контроллера
- Левый стик: Направление движения (gait)
- Правый стик: Скорость и поворот (gait)
- Triangle: Переключение типа походки (Ripple/Tripod)
