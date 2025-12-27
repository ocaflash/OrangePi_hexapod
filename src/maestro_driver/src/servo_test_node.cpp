/**
 * Диагностический узел для тестирования сервоприводов по одному
 * Запуск: ros2 run maestro_driver servo_test_node
 * 
 * Двигает каждый сервопривод по очереди, чтобы определить маппинг каналов
 */

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include "PolstroSerialInterface.h"

class ServoTestNode : public rclcpp::Node {
public:
    ServoTestNode() : Node("servo_test_node") {
        this->declare_parameter<std::string>("port_name", "/dev/ttyS5");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        maestro_ = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);

        if (maestro_ && maestro_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Maestro connected on %s", port_name.c_str());
            
            // Проверяем ошибки Maestro
            unsigned short errors = 0;
            if (maestro_->getErrors(errors)) {
                if (errors != 0) {
                    RCLCPP_WARN(this->get_logger(), "Maestro has errors: 0x%04X", errors);
                    if (errors & 0x0001) RCLCPP_WARN(this->get_logger(), "  - Serial signal error");
                    if (errors & 0x0002) RCLCPP_WARN(this->get_logger(), "  - Serial overrun error");
                    if (errors & 0x0004) RCLCPP_WARN(this->get_logger(), "  - Serial buffer full");
                    if (errors & 0x0008) RCLCPP_WARN(this->get_logger(), "  - Serial CRC error");
                    if (errors & 0x0010) RCLCPP_WARN(this->get_logger(), "  - Serial protocol error");
                    if (errors & 0x0020) RCLCPP_WARN(this->get_logger(), "  - Serial timeout");
                    if (errors & 0x0040) RCLCPP_WARN(this->get_logger(), "  - Script stack error");
                    if (errors & 0x0080) RCLCPP_WARN(this->get_logger(), "  - Script call stack error");
                    if (errors & 0x0100) RCLCPP_WARN(this->get_logger(), "  - Script program counter error");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Maestro: no errors");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Could not read Maestro errors (maybe wrong protocol?)");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Maestro on %s", port_name.c_str());
            return;
        }

        // Сначала установим все сервы в нейтраль (используем Compact Protocol)
        RCLCPP_INFO(this->get_logger(), "Setting all servos to neutral (6000 qus = 1.5ms)...");
        for (int ch = 0; ch < 18; ch++) {
            maestro_->setTarget(ch, 6000);  // 6000 quarter-microseconds = 1.5ms = neutral
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Тестируем каждый канал
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "=== НАЧАЛО ТЕСТА СЕРВОПРИВОДОВ ===");
        RCLCPP_INFO(this->get_logger(), "Ожидаемый маппинг:");
        RCLCPP_INFO(this->get_logger(), "  Ch 0-2:   R1 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 3-5:   R2 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 6-8:   R3 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 9-11:  L1 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 12-14: L2 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 15-17: L3 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "");

        for (int ch = 0; ch < 18; ch++) {
            std::string leg_name;
            std::string joint_name;
            
            int leg_idx = ch / 3;
            int joint_idx = ch % 3;
            
            switch (leg_idx) {
                case 0: leg_name = "R1"; break;
                case 1: leg_name = "R2"; break;
                case 2: leg_name = "R3"; break;
                case 3: leg_name = "L1"; break;
                case 4: leg_name = "L2"; break;
                case 5: leg_name = "L3"; break;
            }
            
            switch (joint_idx) {
                case 0: joint_name = "coxa"; break;
                case 1: joint_name = "femur"; break;
                case 2: joint_name = "tibia"; break;
            }

            RCLCPP_INFO(this->get_logger(), ">>> Канал %d: Ожидается %s %s", ch, leg_name.c_str(), joint_name.c_str());
            RCLCPP_INFO(this->get_logger(), "    Двигаю: neutral -> -30deg -> +30deg -> neutral");

            // Используем Compact Protocol (setTarget) вместо MiniSSC
            // Значения в quarter-microseconds: 4000 = 1ms, 6000 = 1.5ms (neutral), 8000 = 2ms
            
            // Нейтраль
            maestro_->setTarget(ch, 6000);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            // Двигаем в одну сторону (1.25ms = 5000 qus)
            maestro_->setTarget(ch, 5000);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Двигаем в другую сторону (1.75ms = 7000 qus)
            maestro_->setTarget(ch, 7000);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Возвращаем в нейтраль
            maestro_->setTarget(ch, 6000);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            RCLCPP_INFO(this->get_logger(), "");
        }

        RCLCPP_INFO(this->get_logger(), "=== ТЕСТ ЗАВЕРШЁН ===");
        RCLCPP_INFO(this->get_logger(), "Запишите, какой канал соответствует какому сервоприводу!");
    }

    ~ServoTestNode() {
        if (maestro_) {
            delete maestro_;
        }
    }

private:
    Polstro::SerialInterface* maestro_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoTestNode>();
    // Не нужен spin - тест выполняется в конструкторе
    rclcpp::shutdown();
    return 0;
}
