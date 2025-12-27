/**
 * Диагностический узел для тестирования сервоприводов
 * Простая версия без чтения ответов от Maestro
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
        this->declare_parameter<std::string>("protocol", "compact");  // "compact" or "mini_ssc"
        this->declare_parameter<int>("channel", -1);  // -1 = test all, 0-17 = test specific

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        std::string protocol = this->get_parameter("protocol").as_string();
        int test_channel = this->get_parameter("channel").as_int();

        RCLCPP_INFO(this->get_logger(), "Opening %s at %d baud, protocol=%s", 
                    port_name.c_str(), baud_rate, protocol.c_str());

        maestro_ = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);

        if (!maestro_ || !maestro_->isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Maestro on %s", port_name.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Maestro connected!");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "=== ТЕСТ СЕРВОПРИВОДОВ ===");
        RCLCPP_INFO(this->get_logger(), "Ожидаемый маппинг каналов:");
        RCLCPP_INFO(this->get_logger(), "  Ch 0-2:   R1 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 3-5:   R2 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 6-8:   R3 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 9-11:  L1 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 12-14: L2 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "  Ch 15-17: L3 (coxa, femur, tibia)");
        RCLCPP_INFO(this->get_logger(), "");

        // Определяем какие каналы тестировать
        int start_ch = (test_channel >= 0) ? test_channel : 0;
        int end_ch = (test_channel >= 0) ? test_channel + 1 : 18;

        for (int ch = start_ch; ch < end_ch && ch < 18; ch++) {
            std::string leg_name, joint_name;
            int leg_idx = ch / 3;
            int joint_idx = ch % 3;
            
            const char* legs[] = {"R1", "R2", "R3", "L1", "L2", "L3"};
            const char* joints[] = {"coxa", "femur", "tibia"};
            leg_name = legs[leg_idx];
            joint_name = joints[joint_idx];

            RCLCPP_INFO(this->get_logger(), ">>> Канал %d: %s %s", ch, leg_name.c_str(), joint_name.c_str());

            if (protocol == "mini_ssc") {
                // MiniSSC protocol: 0xFF, channel, position (0-254, 127=neutral)
                RCLCPP_INFO(this->get_logger(), "    MiniSSC: 127 -> 80 -> 174 -> 127");
                
                maestro_->setTargetMSS(ch, 127);
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
                
                maestro_->setTargetMSS(ch, 80);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                
                maestro_->setTargetMSS(ch, 174);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                
                maestro_->setTargetMSS(ch, 127);
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
            } else {
                // Compact protocol: 0x84, channel, target_low, target_high
                // Target in quarter-microseconds: 4000=1ms, 6000=1.5ms, 8000=2ms
                RCLCPP_INFO(this->get_logger(), "    Compact: 6000 -> 4500 -> 7500 -> 6000 (qus)");
                
                maestro_->setTarget(ch, 6000);
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
                
                maestro_->setTarget(ch, 4500);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                
                maestro_->setTarget(ch, 7500);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                
                maestro_->setTarget(ch, 6000);
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
            }
            
            RCLCPP_INFO(this->get_logger(), "");
        }

        RCLCPP_INFO(this->get_logger(), "=== ТЕСТ ЗАВЕРШЁН ===");
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
    rclcpp::shutdown();
    return 0;
}
