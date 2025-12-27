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
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Maestro on %s", port_name.c_str());
            return;
        }

        // Сначала установим все сервы в нейтраль
        RCLCPP_INFO(this->get_logger(), "Setting all servos to neutral (127)...");
        for (int ch = 0; ch < 18; ch++) {
            maestro_->setTargetMSS(ch, 127);
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
            RCLCPP_INFO(this->get_logger(), "    Двигаю 127 -> 90 -> 164 -> 127");

            // Двигаем в одну сторону
            maestro_->setTargetMSS(ch, 90);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Двигаем в другую сторону
            maestro_->setTargetMSS(ch, 164);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Возвращаем в нейтраль
            maestro_->setTargetMSS(ch, 127);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

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
